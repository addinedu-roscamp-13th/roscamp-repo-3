"""MyCobot과 그리퍼의 저수준 제어. 복잡한 비전/기하계산은 포함하지 않습니다."""
from __future__ import annotations

import math
import time
from threading import Event
from typing import Callable, Iterable

from pymycobot.common import ProtocolCode
from pymycobot.mycobot280 import MyCobot280

from . import config


JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (-168.0, 168.0),  # J1
    (-135.0, 135.0),  # J2
    (-150.0, 150.0),  # J3
    (-145.0, 145.0),  # J4
    (-155.0, 160.0),  # J5
    (-180.0, 180.0),  # J6
)


def _angle_difference_deg(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


class WaSaBArmController:
    def __init__(self) -> None:
        self.mc = MyCobot280(config.PORT, config.BAUD)
        self.last_wait_timeout_reason: str | None = None
        self.mc.thread_lock = True
        self.mc.focus_all_servos()

        # 던지는 구간에서는 "최신 명령 우선"으로 동작시켜
        # 뒤늦게 온 그리퍼 명령이 관절 이동 완료 뒤로 밀리지 않게 합니다.
        self.mc.set_fresh_mode(1)
        self.set_flange_mode()

    def set_flange_mode(self) -> None:
        self.mc.set_reference_frame(0)
        self.mc.set_end_type(0)

    def get_flange_coords(self) -> list[float]:
        # Flange mode is established at initialization and before every
        # Cartesian command. Re-sending two mode commands on every 50ms pose
        # poll triples serial traffic and can delay motion feedback.
        coords = self.mc.get_coords()
        if not isinstance(coords, list) or len(coords) != 6:
            raise RuntimeError(f"get_coords failed: {coords}")

        result = [float(v) for v in coords]
        if not all(math.isfinite(v) for v in result):
            raise RuntimeError(f"get_coords returned non-finite values: {coords}")
        return result

    def set_gripper_value(
        self,
        value: int,
        label: str,
        *,
        settle: bool = True,
        speed: int | None = None,
        settle_sec: float | None = None,
    ) -> None:
        method = getattr(self.mc, "set_gripper_value", None)
        if method is None:
            raise RuntimeError("set_gripper_value() is unavailable in this pymycobot version")

        value = max(0, min(100, int(round(value))))
        gripper_speed = config.GRIPPER_SPEED if speed is None else int(speed)
        print(
            f"[GRIPPER] {label}: value={value}, "
            f"speed={gripper_speed}, settle={settle}"
        )
        try:
            method(value, gripper_speed)
        except Exception as exc:
            raise RuntimeError(f"gripper {label} command failed: {exc}") from exc

        if settle:
            time.sleep(config.GRIPPER_SETTLE_SEC if settle_sec is None else float(settle_sec))

    def open_gripper(self, *, speed: int | None = None, settle_sec: float | None = None) -> None:
        self.set_gripper_value(
            config.GRIPPER_OPEN_VALUE,
            "open",
            settle=True,
            speed=speed,
            settle_sec=settle_sec,
        )

    def close_gripper(self) -> None:
        self.set_gripper_value(config.GRIPPER_CLOSE_VALUE, "close", settle=True)

    def get_gripper_value(self) -> int | None:
        method = getattr(self.mc, "get_gripper_value", None)
        if method is None:
            return None
        try:
            raw = method()
        except Exception as exc:
            print("[GRIPPER] get value failed:", exc)
            return None

        if isinstance(raw, list):
            if not raw:
                return None
            raw = raw[0]
        try:
            value = int(round(float(raw)))
        except (TypeError, ValueError):
            return None
        return max(0, min(100, value))

    def is_gripper_open(self) -> bool | None:
        value = self.get_gripper_value()
        if value is None:
            return None
        midpoint = (config.GRIPPER_OPEN_VALUE + config.GRIPPER_CLOSE_VALUE) / 2.0
        if config.GRIPPER_OPEN_VALUE >= config.GRIPPER_CLOSE_VALUE:
            return value >= midpoint
        return value <= midpoint

    def ensure_gripper_open(self) -> None:
        is_open = self.is_gripper_open()
        if is_open is True:
            print("[GRIPPER] already open")
            return
        if is_open is None:
            print("[GRIPPER] state unknown; opening before pick")
        else:
            print("[GRIPPER] closed; opening before pick")
        self.open_gripper()

    def open_gripper_async_now(self) -> None:
        """그리퍼 열기 패킷만 즉시 전송합니다.

        set_gripper_value()는 응답을 기다리는 일반 호출입니다.
        이 메서드는 내부 _mesg(..., _async=True)를 사용해 응답 대기 없이
        SET_GRIPPER_VALUE 패킷을 시리얼에 바로 기록합니다.
        """
        value = max(0, min(100, int(round(config.GRIPPER_OPEN_VALUE))))
        print(
            f"[GRIPPER] async open command: "
            f"value={value}, speed={config.GRIPPER_SPEED}"
        )
        self.mc._mesg(
            ProtocolCode.SET_GRIPPER_VALUE,
            value,
            config.GRIPPER_SPEED,
            _async=True,
        )

    def _validate_joint_angles(self, values: Iterable[float], label: str) -> list[float]:
        angles = [float(v) for v in values]

        if len(angles) != 6:
            raise ValueError(f"{label} must contain six joint angles")
        if not all(math.isfinite(v) for v in angles):
            raise ValueError(f"{label} contains non-finite values")

        for index, (angle, limits) in enumerate(zip(angles, JOINT_LIMITS_DEG), start=1):
            if not limits[0] <= angle <= limits[1]:
                raise ValueError(
                    f"{label}: J{index}={angle:.2f} is outside allowed range {limits}"
                )
        return angles

    def _angles_reached(self, target_angles: Iterable[float], tolerance_deg: float) -> bool:
        target = self._validate_joint_angles(target_angles, "target_angles")
        current = self.mc.get_angles()

        if not isinstance(current, list) or len(current) != 6:
            return False

        try:
            current = [float(v) for v in current]
        except (TypeError, ValueError):
            return False

        if not all(math.isfinite(v) for v in current):
            return False

        return all(
            _angle_difference_deg(now, goal) <= tolerance_deg
            for now, goal in zip(current, target)
        )

    def wait_until_joint_angles(
        self,
        target_angles: Iterable[float],
        timeout_sec: float,
        tolerance_deg: float,
        abort_event: Event | None = None,
    ) -> bool:
        target = self._validate_joint_angles(target_angles, "target_angles")
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            if abort_event is not None and abort_event.is_set():
                return False

            if self._angles_reached(target, tolerance_deg):
                print("[ROBOT] joint target reached:", target)
                return True

            time.sleep(0.03)

        print("[ROBOT] joint target wait timeout:", target)
        return False

    def get_joint_angles(self) -> list[float]:
        angles = self.mc.get_angles()
        if not isinstance(angles, list) or len(angles) != 6:
            raise RuntimeError(f"get_angles failed: {angles}")

        result = [float(v) for v in angles]
        if not all(math.isfinite(v) for v in result):
            raise RuntimeError(f"get_angles returned non-finite values: {angles}")
        return self._validate_joint_angles(result, "current_angles")

    def send_joint_angles(
        self,
        target_angles: Iterable[float],
        speed: int | None = None,
        *,
        async_command: bool = True,
    ) -> None:
        angles = self._validate_joint_angles(target_angles, "target_angles")
        move_speed = config.MOVE_SPEED if speed is None else int(speed)
        print("[ROBOT] send_angles:", [round(v, 2) for v in angles], f"speed={move_speed}")
        self.mc.send_angles(angles, move_speed, _async=async_command)

    def send_joint_angle(
        self,
        joint_id: int,
        angle_deg: float,
        speed: int | None = None,
    ) -> None:
        if not 1 <= int(joint_id) <= 6:
            raise ValueError("joint_id must be in the range 1..6")
        angle = float(angle_deg)
        limits = JOINT_LIMITS_DEG[int(joint_id) - 1]
        if not limits[0] <= angle <= limits[1]:
            raise ValueError(
                f"J{joint_id}={angle:.2f} is outside allowed range {limits}"
            )
        move_speed = config.MOVE_SPEED if speed is None else int(speed)
        print("[ROBOT] send_angle:", f"J{joint_id}={angle:.2f}", f"speed={move_speed}")
        self.mc.send_angle(int(joint_id), angle, move_speed)

    def wait_until_flange_pose(
        self,
        target_coords: Iterable[float],
        timeout_sec: float | None = None,
        abort_event: Event | None = None,
        progress_callback: Callable[[], None] | None = None,
        position_tolerance_mm: float | None = None,
        angle_tolerance_deg: float | None = None,
    ) -> bool:
        target = [float(v) for v in target_coords]

        if len(target) != 6:
            raise ValueError("target pose must contain six values")
        if not all(math.isfinite(v) for v in target):
            raise ValueError("target pose contains non-finite values")

        timeout = config.MOVE_TIMEOUT_SEC if timeout_sec is None else float(timeout_sec)
        position_tolerance = (
            config.POSE_POSITION_TOL_MM
            if position_tolerance_mm is None
            else float(position_tolerance_mm)
        )
        angle_tolerance = (
            config.POSE_ANGLE_TOL_DEG
            if angle_tolerance_deg is None
            else float(angle_tolerance_deg)
        )
        if position_tolerance <= 0 or angle_tolerance <= 0:
            raise ValueError("pose tolerances must be positive")
        started_at = time.monotonic()
        deadline = started_at + timeout
        last_current: list[float] | None = None
        last_position_error: float | None = None
        last_angle_error: float | None = None
        worst_position_axis: str | None = None
        worst_angle_axis: str | None = None
        axis_names = ("X", "Y", "Z", "RX", "RY", "RZ")
        self.last_wait_timeout_reason = None

        while time.monotonic() < deadline:
            if abort_event is not None and abort_event.is_set():
                self.last_wait_timeout_reason = "aborted by stop request"
                return False
            if progress_callback is not None:
                progress_callback()

            current = self.get_flange_coords()
            position_errors = [abs(current[i] - target[i]) for i in range(3)]
            angle_errors = [
                _angle_difference_deg(current[i], target[i])
                for i in range(3, 6)
            ]
            position_error = max(position_errors)
            angle_error = max(angle_errors)
            last_current = current
            last_position_error = position_error
            last_angle_error = angle_error
            worst_position_axis = axis_names[position_errors.index(position_error)]
            worst_angle_axis = axis_names[3 + angle_errors.index(angle_error)]

            if (
                position_error <= position_tolerance
                and angle_error <= angle_tolerance
            ):
                self.last_wait_timeout_reason = None
                print(
                    "[ROBOT] target reached: "
                    f"pos={position_error:.2f} mm, angle={angle_error:.2f} deg, "
                    f"elapsed={time.monotonic() - started_at:.2f}s"
                )
                return True

            time.sleep(config.MOVE_POLL_SEC)

        if last_current is None:
            reason = f"no valid flange pose was read within {timeout:.1f}s"
        else:
            reason = (
                f"pose did not reach tolerance within {timeout:.1f}s; "
                f"pos_error={last_position_error:.2f}mm"
                f"({worst_position_axis}, tol={position_tolerance:.2f}mm), "
                f"angle_error={last_angle_error:.2f}deg"
                f"({worst_angle_axis}, tol={angle_tolerance:.2f}deg), "
                f"target={[round(v, 2) for v in target]}, "
                f"current={[round(v, 2) for v in last_current]}"
            )
        self.last_wait_timeout_reason = reason
        print(f"[ROBOT] target wait timeout : {reason}")
        return False

    def send_flange_coords(
        self,
        target_coords: list[float],
        speed: int | None = None,
        mode: int | None = None,
    ) -> None:
        self.set_flange_mode()
        move_speed = config.MOVE_SPEED if speed is None else int(speed)
        move_mode = config.MOVE_MODE if mode is None else int(mode)
        print("[ROBOT] send_coords(Flange):", target_coords, f"speed={move_speed}", f"mode={move_mode}")
        try:
            self.mc.send_coords(target_coords, move_speed, move_mode)
        except Exception as exc:
            self.last_wait_timeout_reason = (
                f"send_coords rejected target={target_coords}: "
                f"{type(exc).__name__}: {exc}"
            )
            print("[ROBOT]", self.last_wait_timeout_reason)

    def send_flange_coords_and_wait(
        self,
        target_coords: list[float],
        progress_callback: Callable[[], None] | None = None,
        speed: int | None = None,
        mode: int | None = None,
        abort_event: Event | None = None,
        position_tolerance_mm: float | None = None,
        angle_tolerance_deg: float | None = None,
    ) -> bool:
        self.set_flange_mode()
        move_speed = config.MOVE_SPEED if speed is None else int(speed)
        move_mode = config.MOVE_MODE if mode is None else int(mode)
        print("[ROBOT] send_coords(Flange):", target_coords, f"speed={move_speed}", f"mode={move_mode}")
        try:
            self.mc.send_coords(target_coords, move_speed, move_mode)
        except Exception as exc:
            self.last_wait_timeout_reason = (
                f"send_coords rejected target={target_coords}: "
                f"{type(exc).__name__}: {exc}"
            )
            print("[ROBOT]", self.last_wait_timeout_reason)
            return False
        return self.wait_until_flange_pose(
            target_coords,
            abort_event=abort_event,
            progress_callback=progress_callback,
            position_tolerance_mm=position_tolerance_mm,
            angle_tolerance_deg=angle_tolerance_deg,
        )

    def move_home_and_open_gripper(self) -> bool:
        self.open_gripper()
        return self._move_home()

    def _move_home(
        self,
        progress_callback: Callable[[], None] | None = None,
        abort_event: Event | None = None,
    ) -> bool:
        if config.HOME_JOINT_ANGLES is not None:
            self.send_joint_angles(
                config.HOME_JOINT_ANGLES,
                speed=config.MOVE_SPEED,
                async_command=True,
            )
            reached = self.wait_until_joint_angles(
                config.HOME_JOINT_ANGLES,
                timeout_sec=config.MOVE_TIMEOUT_SEC,
                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                abort_event=abort_event,
            )
            if reached and config.HOME_SETTLE_SEC > 0:
                print(
                    "[ROBOT] HOME camera settle:",
                    f"{config.HOME_SETTLE_SEC:.1f}s",
                )
                deadline = time.monotonic() + config.HOME_SETTLE_SEC
                while time.monotonic() < deadline:
                    if abort_event is not None and abort_event.is_set():
                        return False
                    time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
            return reached

        self.set_flange_mode()
        reached = self.send_flange_coords_and_wait(
            config.HOME_FLANGE_COORDS,
            progress_callback=progress_callback,
            abort_event=abort_event,
        )
        if reached and config.HOME_SETTLE_SEC > 0:
            print(
                "[ROBOT] HOME camera settle:",
                f"{config.HOME_SETTLE_SEC:.1f}s",
            )
            deadline = time.monotonic() + config.HOME_SETTLE_SEC
            while time.monotonic() < deadline:
                if abort_event is not None and abort_event.is_set():
                    return False
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return reached

    def move_home_keep_gripper_closed(
        self,
        progress_callback: Callable[[], None] | None = None,
        abort_event: Event | None = None,
    ) -> bool:
        return self._move_home(
            progress_callback=progress_callback,
            abort_event=abort_event,
        )

    def move_gesture_home(
        self,
        abort_event: Event | None = None,
    ) -> bool:
        if config.GESTURE_HOME_JOINT_ANGLES is not None:
            self.send_joint_angles(
                config.GESTURE_HOME_JOINT_ANGLES,
                speed=config.GESTURE_HOME_SPEED,
                async_command=True,
            )
            return self.wait_until_joint_angles(
                config.GESTURE_HOME_JOINT_ANGLES,
                timeout_sec=config.MOVE_TIMEOUT_SEC,
                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                abort_event=abort_event,
            )

        return self.send_flange_coords_and_wait(
            config.GESTURE_HOME_FLANGE_COORDS,
            speed=config.GESTURE_HOME_SPEED,
            abort_event=abort_event,
        )

    def stop_motion(self) -> None:
        print("[ROBOT] stop motion")
        self.mc.stop()

    def release_all_servos(self) -> None:
        print("[ROBOT] release all servos")
        self.mc.release_all_servos()

    def focus_all_servos(self) -> None:
        print("[ROBOT] focus all servos")
        self.mc.focus_all_servos()
        self.set_flange_mode()

    def move_manual_flange_coords(self) -> bool:
        self.focus_all_servos()
        return self.send_flange_coords_and_wait(config.MANUAL_FLANGE_COORDS)

    def move_place_flange_coords(self) -> bool:
        self.focus_all_servos()
        return self.send_flange_coords_and_wait(config.PLACE_FLANGE_COORDS)
