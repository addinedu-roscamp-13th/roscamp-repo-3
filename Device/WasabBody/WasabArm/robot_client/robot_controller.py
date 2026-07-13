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
        self.set_flange_mode()
        coords = self.mc.get_coords()
        if not isinstance(coords, list) or len(coords) != 6:
            raise RuntimeError(f"get_coords failed: {coords}")

        result = [float(v) for v in coords]
        if not all(math.isfinite(v) for v in result):
            raise RuntimeError(f"get_coords returned non-finite values: {coords}")
        return result

    def set_gripper_value(self, value: int, label: str, *, settle: bool = True) -> None:
        method = getattr(self.mc, "set_gripper_value", None)
        if method is None:
            raise RuntimeError("set_gripper_value() is unavailable in this pymycobot version")

        value = max(0, min(100, int(round(value))))
        print(
            f"[GRIPPER] {label}: value={value}, "
            f"speed={config.GRIPPER_SPEED}, settle={settle}"
        )
        try:
            method(value, config.GRIPPER_SPEED)
        except Exception as exc:
            raise RuntimeError(f"gripper {label} command failed: {exc}") from exc

        if settle:
            time.sleep(config.GRIPPER_SETTLE_SEC)

    def open_gripper(self) -> None:
        self.set_gripper_value(config.GRIPPER_OPEN_VALUE, "open", settle=True)

    def close_gripper(self) -> None:
        self.set_gripper_value(config.GRIPPER_CLOSE_VALUE, "close", settle=True)

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
    ) -> bool:
        target = [float(v) for v in target_coords]

        if len(target) != 6:
            raise ValueError("target pose must contain six values")
        if not all(math.isfinite(v) for v in target):
            raise ValueError("target pose contains non-finite values")

        timeout = config.MOVE_TIMEOUT_SEC if timeout_sec is None else float(timeout_sec)
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if abort_event is not None and abort_event.is_set():
                return False
            if progress_callback is not None:
                progress_callback()

            current = self.get_flange_coords()
            position_error = max(abs(current[i] - target[i]) for i in range(3))
            angle_error = max(
                _angle_difference_deg(current[i], target[i])
                for i in range(3, 6)
            )

            if (
                position_error <= config.POSE_POSITION_TOL_MM
                and angle_error <= config.POSE_ANGLE_TOL_DEG
            ):
                print(
                    "[ROBOT] target reached: "
                    f"pos={position_error:.2f} mm, angle={angle_error:.2f} deg"
                )
                return True

            time.sleep(config.MOVE_POLL_SEC)

        print("[ROBOT] target wait timeout")
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
        print("[ROBOT] send_coords(Flange):", target_coords, f"speed={move_speed}")
        self.mc.send_coords(target_coords, move_speed, move_mode)

    def send_flange_coords_and_wait(
        self,
        target_coords: list[float],
        progress_callback: Callable[[], None] | None = None,
        speed: int | None = None,
        mode: int | None = None,
        abort_event: Event | None = None,
    ) -> bool:
        self.set_flange_mode()
        move_speed = config.MOVE_SPEED if speed is None else int(speed)
        move_mode = config.MOVE_MODE if mode is None else int(mode)
        print("[ROBOT] send_coords(Flange):", target_coords, f"speed={move_speed}")
        self.mc.send_coords(target_coords, move_speed, move_mode)
        return self.wait_until_flange_pose(
            target_coords,
            abort_event=abort_event,
            progress_callback=progress_callback,
        )

    def move_home_and_open_gripper(self) -> bool:
        self.set_flange_mode()
        self.open_gripper()
        return self.send_flange_coords_and_wait(config.HOME_FLANGE_COORDS)

    def move_home_keep_gripper_closed(
        self,
        progress_callback: Callable[[], None] | None = None,
        abort_event: Event | None = None,
    ) -> bool:
        self.set_flange_mode()
        return self.send_flange_coords_and_wait(
            config.HOME_FLANGE_COORDS,
            progress_callback=progress_callback,
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

    @staticmethod
    def _wait_delay_or_abort(delay_sec: float, abort_event: Event | None) -> bool:
        deadline = time.monotonic() + delay_sec

        while time.monotonic() < deadline:
            if abort_event is not None and abort_event.is_set():
                return False
            time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))

        return True

    def _get_throw_final_coords(self) -> list[float]:
        raw = getattr(
            config,
            "THROW_FINAL_FLANGE_COORDS",
            getattr(config, "THROW_FINAL_COORDS", None),
        )
        if raw is None:
            raise ValueError(
                "config.py needs THROW_FINAL_FLANGE_COORDS "
                "or legacy THROW_FINAL_COORDS"
            )

        coords = [float(v) for v in raw]
        if len(coords) != 6 or not all(math.isfinite(v) for v in coords):
            raise ValueError("throw final coords must contain six finite values")
        return coords

    def _get_throw_final_speed(self) -> int:
        return int(
            getattr(
                config,
                "THROW_FINAL_MOVE_SPEED",
                getattr(config, "THROW_FINAL_COORD_SPEED", config.MOVE_SPEED),
            )
        )

    def _get_throw_final_mode(self) -> int:
        return int(
            getattr(
                config,
                "THROW_FINAL_MOVE_MODE",
                getattr(config, "THROW_FINAL_COORD_MODE", config.MOVE_MODE),
            )
        )

    def execute_throw_mode(
        self,
        abort_event: Event | None = None,
    ) -> tuple[bool, str, bool]:
        """실제 비동기 던지기 동작.

        1) 시작 자세 도착까지 대기
        2) 종료 자세 이동을 _async=True으로 전송
        3) 지정 지연 후 그리퍼 열기 패킷을 _async=True으로 전송
        4) 종료 자세 도착 확인
        5) 최종 Flange 좌표 이동을 _async=True으로 전송
        """
        released = False

        try:
            start_angles = self._validate_joint_angles(
                config.THROW_START_ANGLES,
                "THROW_START_ANGLES",
            )
            end_angles = self._validate_joint_angles(
                config.THROW_END_ANGLES,
                "THROW_END_ANGLES",
            )
            final_coords = self._get_throw_final_coords()
            final_speed = self._get_throw_final_speed()
            final_mode = self._get_throw_final_mode()
        except (AttributeError, TypeError, ValueError) as exc:
            return False, f"Throw configuration error: {exc}", released

        if not 1 <= int(config.THROW_PREP_SPEED) <= 100:
            return False, "THROW_PREP_SPEED must be in 1..100", released
        if not 1 <= int(config.THROW_SPEED) <= 100:
            return False, "THROW_SPEED must be in 1..100", released
        if not 1 <= final_speed <= 100:
            return False, "THROW_FINAL_MOVE_SPEED must be in 1..100", released
        if final_mode not in (0, 1):
            return False, "THROW_FINAL_MOVE_MODE must be 0 or 1", released

        if abort_event is not None and abort_event.is_set():
            return False, "Throw cancelled before start", released

        # 1) 시작 자세까지 이동: 명령은 비동기, 도착 여부만 여기서 기다립니다.
        print("[THROW] move to start pose:", start_angles)
        self.mc.send_angles(
            start_angles,
            int(config.THROW_PREP_SPEED),
            _async=True,
        )

        if not self.wait_until_joint_angles(
            start_angles,
            float(config.THROW_PREP_TIMEOUT_SEC),
            float(config.THROW_ANGLE_TOLERANCE_DEG),
            abort_event,
        ):
            return False, "Throw start pose timeout/cancelled", released

        if abort_event is not None and abort_event.is_set():
            return False, "Throw cancelled before release motion", released

        # 2) 팔은 이 줄 이후 계속 end_angles 방향으로 움직입니다.
        print("[THROW] start release motion:", end_angles)
        throw_command_time = time.monotonic()
        self.mc.send_angles(
            end_angles,
            int(config.THROW_SPEED),
            _async=True,
        )

        # 3) 이 대기는 백그라운드 throw worker 안에서만 일어납니다.
        # 팔 이동은 이미 시작됐으며, main UI 루프도 계속 동작합니다.
        if not self._wait_delay_or_abort(
            float(config.THROW_GRIPPER_OPEN_DELAY_SEC),
            abort_event,
        ):
            return False, "Throw cancelled before gripper release", released

        elapsed = time.monotonic() - throw_command_time
        print(f"[THROW] async gripper release command at t={elapsed:.3f}s")

        # 종료 위치 도착 여부를 검사하지 않고 무조건 즉시 패킷을 전송합니다.
        self.open_gripper_async_now()
        released = True

        # 4) 그리퍼 명령 전송 이후에만 종료 자세 도착을 확인합니다.
        if not self.wait_until_joint_angles(
            end_angles,
            float(config.THROW_END_TIMEOUT_SEC),
            float(config.THROW_ANGLE_TOLERANCE_DEG),
            abort_event,
        ):
            return False, "Throw end pose timeout/cancelled", released

        if abort_event is not None and abort_event.is_set():
            return False, "Throw cancelled before final move", released

        # 5) 최종 좌표도 비동기 전송 후, worker에서만 도착 여부를 확인합니다.
        self.set_flange_mode()
        print("[THROW] move to final flange pose:", final_coords)
        self.mc.send_coords(
            final_coords,
            final_speed,
            final_mode,
            _async=True,
        )

        if not self.wait_until_flange_pose(
            final_coords,
            timeout_sec=float(config.THROW_FINAL_TIMEOUT_SEC),
            abort_event=abort_event,
        ):
            return False, "Throw final pose timeout/cancelled", released

        return True, "Throw sequence completed", released