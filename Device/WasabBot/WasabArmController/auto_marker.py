import os
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import sys
import json
import time
import select
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np
from pymycobot.mycobot280 import MyCobot280


# =========================================================
# 1. 사용자 설정
# =========================================================

PORT = "/dev/ttyJETCOBOT"
BAUD = 1000000
CAMERA_ID = 0

# 수동으로 잘 찍힌 샘플 JSON
SOURCE_JSON = "saved_robot_charuco_points.json"

SPEED = 10
WAIT_AFTER_MOVE_SEC = 3.0

# None이면 q 누를 때까지 계속 촬영
# 예: 30개만 찍고 싶으면 30
MAX_AUTO_SAMPLES = 30

# ChArUco 보드 설정
SQUARES_X = 11
SQUARES_Y = 8
SQUARE_LENGTH_MM = 15.0
MARKER_LENGTH_MM = 11.0
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50

MIN_CHARUCO_CORNERS = 6
MIN_INTRINSIC_SAMPLES = 10
MIN_HANDEYE_SAMPLES = 10

# 기존 K를 무시하고 이번 자동 샘플로 새로 K를 구할지 여부
FORCE_RECALIBRATE_INTRINSIC = True

INTRINSIC_FILE = "camera_intrinsic_charuco.npz"

# =========================================================
# Eye-in-hand 설정
# =========================================================
# 현재 전제:
#   - 카메라는 로봇팔 끝단/그리퍼 쪽에 고정됨
#   - ChArUco 보드는 바닥/책상에 고정됨
#   - 로봇팔을 여러 pose로 이동 후 완전히 정지한 상태에서 촬영
#
# OpenCV hand-eye 입력:
#   get_coords() -> ^bT_g, 즉 gripper 좌표계를 base 좌표계로 변환하는 pose
#   solvePnP()   -> ^cT_t, 즉 target/ChArUco 좌표계를 camera 좌표계로 변환하는 pose
#
# OpenCV hand-eye 출력:
#   ^gT_c, 즉 camera 좌표계를 gripper 좌표계로 변환하는 pose
#
# 검증:
#   ^bT_t_i = ^bT_g_i @ ^gT_c @ ^cT_t_i
#   ChArUco 보드는 바닥에 고정되어 있으므로 ^bT_t_i가 모든 샘플에서 거의 같아야 함.
CALIBRATION_MODE = "eye_in_hand"

# MyCobot get_coords()의 rx, ry, rz를 회전행렬로 바꾸는 Euler 순서 후보.
# 결과가 좋지 않으면 후보별 결과를 비교해 가장 std/RMS가 낮은 convention을 선택합니다.
# 한 번 맞는 convention을 찾으면 EULER_ORDER_CANDIDATES = ["찾은값"] 처럼 고정하세요.
EULER_ORDER_CANDIDATES = ["zyx", "zxy"]

# hand-eye 방법
HANDEYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}

# "AUTO_BEST"이면 target 위치 RMS가 가장 작은 method/euler 조합을 선택.
# 특정 방법만 고정하고 싶으면 "TSAI", "PARK", "HORAUD", "DANIILIDIS" 중 하나로 변경.
SELECTED_HANDEYE_METHOD = "AUTO_BEST"

# hand-eye에 사용할 샘플의 solvePnP reprojection error 필터.
# None이면 필터링하지 않음. 보통 1~2 px 초과 샘플은 제외하는 것을 권장.
MAX_REPROJECTION_ERROR_PX_FOR_HANDEYE = 2.0

# send_angles 이후 로봇 정지 확인을 시도할지 여부.
WAIT_UNTIL_ROBOT_STOPS = True
ROBOT_STOP_TIMEOUT_SEC = 15.0
ROBOT_STABLE_SEC = 0.8

# 샘플링 방식: interpolation 또는 gaussian
SAMPLING_MODE = "gaussian"

ANGLE_MARGIN_DEG = 2.0
NOISE_SCALE = 0.08


# =========================================================
# 2. 자동 저장 경로
# =========================================================

RUN_ID = datetime.now().strftime("%Y%m%d_%H%M%S")

AUTO_IMAGE_DIR = Path(f"auto_charuco_captures_{RUN_ID}")
AUTO_IMAGE_DIR.mkdir(exist_ok=True)

AUTO_OUTPUT_JSON = f"auto_robot_charuco_points_{RUN_ID}.json"
AUTO_OUTPUT_NPZ = f"auto_handeye_charuco_samples_{RUN_ID}.npz"
AUTO_INTRINSIC_FILE = f"auto_camera_intrinsic_charuco_{RUN_ID}.npz"
AUTO_HANDEYE_FILE = f"auto_handeye_result_{RUN_ID}.json"
AUTO_HANDEYE_NPZ = f"auto_handeye_result_{RUN_ID}.npz"


# =========================================================
# 3. ChArUco 유틸
# =========================================================

def get_aruco_dictionary():
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)


def create_charuco_board():
    aruco_dict = get_aruco_dictionary()

    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_MM,
        MARKER_LENGTH_MM,
        aruco_dict
    )

    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(True)

    return board, aruco_dict


def get_charuco_object_points(board, charuco_ids):
    ids = np.array(charuco_ids, dtype=np.int32).reshape(-1)

    if hasattr(board, "getChessboardCorners"):
        chessboard_corners = np.asarray(
            board.getChessboardCorners(),
            dtype=np.float32
        ).reshape(-1, 3)

        return chessboard_corners[ids].astype(np.float32)

    obj_points = []
    inner_cols = SQUARES_X - 1

    for corner_id in ids:
        y = corner_id // inner_cols
        x = corner_id % inner_cols

        X = (x + 1) * SQUARE_LENGTH_MM
        Y = (y + 1) * SQUARE_LENGTH_MM
        Z = 0.0

        obj_points.append([X, Y, Z])

    return np.array(obj_points, dtype=np.float32)


def compute_reprojection_error_px(obj_points, img_points, rvec, tvec, K, dist):
    """
    solvePnP 결과가 이미지상 ChArUco corner를 얼마나 잘 재투영하는지 평균 pixel error로 계산.
    값이 클수록 해당 샘플의 target2cam pose가 부정확할 가능성이 큼.
    """
    projected, _ = cv2.projectPoints(
        obj_points.astype(np.float32),
        np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        K,
        dist
    )

    projected = projected.reshape(-1, 2)
    img_points = img_points.reshape(-1, 2)

    errors = np.linalg.norm(projected - img_points, axis=1)
    return float(np.mean(errors))


def detect_charuco(frame, board, aruco_dict, K=None, dist=None):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    annotated = frame.copy()

    result = {
        "marker_count": 0,
        "marker_ids": None,
        "charuco_corner_count": 0,
        "charuco_ids": None,
        "charuco_corners_px": None,
        "pose_ok": False,
        "rvec_target2cam": None,
        "tvec_target2cam_mm": None,
        "reprojection_error_px": None,
        "object_points": None,
        "image_points": None,
    }

    charuco_corners = None
    charuco_ids = None
    marker_corners = None
    marker_ids = None

    if hasattr(cv2.aruco, "CharucoDetector"):
        detector = cv2.aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray,
            aruco_dict,
            parameters=params
        )

        if marker_ids is not None and len(marker_ids) > 0:
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                board
            )

    if marker_ids is not None:
        result["marker_count"] = int(len(marker_ids))
        result["marker_ids"] = marker_ids.flatten().astype(int).tolist()

        cv2.aruco.drawDetectedMarkers(
            annotated,
            marker_corners,
            marker_ids
        )

    if charuco_corners is None or charuco_ids is None:
        return result, annotated

    charuco_ids_flat = charuco_ids.flatten().astype(int)

    result["charuco_corner_count"] = int(len(charuco_ids_flat))
    result["charuco_ids"] = charuco_ids_flat.tolist()
    result["charuco_corners_px"] = charuco_corners.reshape(-1, 2).astype(float).tolist()

    cv2.aruco.drawDetectedCornersCharuco(
        annotated,
        charuco_corners,
        charuco_ids,
        (0, 255, 0)
    )

    if len(charuco_ids_flat) < MIN_CHARUCO_CORNERS:
        return result, annotated

    obj_points = get_charuco_object_points(board, charuco_ids_flat)
    img_points = charuco_corners.reshape(-1, 2).astype(np.float32)

    result["object_points"] = obj_points
    result["image_points"] = img_points

    if K is not None and dist is not None:
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            K,
            dist,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if ok:
            reproj_error = compute_reprojection_error_px(
                obj_points=obj_points,
                img_points=img_points,
                rvec=rvec,
                tvec=tvec,
                K=K,
                dist=dist
            )

            result["pose_ok"] = True
            result["rvec_target2cam"] = rvec.reshape(3).astype(float).tolist()
            result["tvec_target2cam_mm"] = tvec.reshape(3).astype(float).tolist()
            result["reprojection_error_px"] = reproj_error

            try:
                cv2.drawFrameAxes(
                    annotated,
                    K,
                    dist,
                    rvec,
                    tvec,
                    SQUARE_LENGTH_MM * 2.0
                )
            except Exception:
                pass

    return result, annotated


# =========================================================
# 4. 입력 데이터 / intrinsic 로드
# =========================================================

def load_previous_success_angles(json_path):
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    angles = []

    for item in data:
        if "angles" not in item:
            continue

        charuco = item.get("charuco", {})

        if not charuco.get("pose_ok", False):
            continue

        if charuco.get("charuco_corner_count", 0) < MIN_CHARUCO_CORNERS:
            continue

        a = item["angles"]

        if isinstance(a, list) and len(a) == 6:
            angles.append(a)

    if len(angles) < 3:
        raise RuntimeError("유효한 angles 샘플이 너무 적습니다.")

    angles = np.array(angles, dtype=np.float64)

    print("\n=== 기존 성공 angles 분포 ===")
    print("sample count:", len(angles))
    print("min:", np.round(angles.min(axis=0), 3))
    print("max:", np.round(angles.max(axis=0), 3))
    print("mean:", np.round(angles.mean(axis=0), 3))
    print("std:", np.round(angles.std(axis=0), 3))

    return angles


def load_intrinsic():
    if FORCE_RECALIBRATE_INTRINSIC:
        print("[정보] FORCE_RECALIBRATE_INTRINSIC=True")
        print("[정보] 기존 K, dist를 사용하지 않고 이번 샘플로 새로 계산합니다.")
        return None, None

    if not os.path.exists(INTRINSIC_FILE):
        print(f"[경고] {INTRINSIC_FILE} 없음")
        print("이번 자동 샘플로 K, dist를 계산합니다.")
        return None, None

    data = np.load(INTRINSIC_FILE)
    K = data["K"]
    dist = data["dist"]

    print("\n=== intrinsic loaded ===")
    print("K:")
    print(K)
    print("dist:")
    print(dist)

    return K, dist


# =========================================================
# 5. 분포 기반 각도 샘플링
# =========================================================

def sample_angle_interpolation(angle_samples):
    n = len(angle_samples)

    i, j = np.random.choice(n, size=2, replace=True)

    a = angle_samples[i]
    b = angle_samples[j]

    alpha = np.random.uniform(0.0, 1.0)
    sampled = alpha * a + (1.0 - alpha) * b

    std = angle_samples.std(axis=0)
    noise = np.random.normal(
        loc=0.0,
        scale=std * NOISE_SCALE
    )

    sampled = sampled + noise

    min_angles = angle_samples.min(axis=0) - ANGLE_MARGIN_DEG
    max_angles = angle_samples.max(axis=0) + ANGLE_MARGIN_DEG

    sampled = np.clip(sampled, min_angles, max_angles)

    return sampled.tolist()


def sample_angle_gaussian(angle_samples):
    mean = angle_samples.mean(axis=0)
    cov = np.cov(angle_samples.T)
    cov = cov + np.eye(6) * 1e-4

    sampled = np.random.multivariate_normal(mean, cov)

    min_angles = angle_samples.min(axis=0) - ANGLE_MARGIN_DEG
    max_angles = angle_samples.max(axis=0) + ANGLE_MARGIN_DEG

    sampled = np.clip(sampled, min_angles, max_angles)

    return sampled.tolist()


def sample_next_angles(angle_samples):
    if SAMPLING_MODE == "gaussian":
        return sample_angle_gaussian(angle_samples)

    return sample_angle_interpolation(angle_samples)


# =========================================================
# 6. q 입력 처리 / 대기
# =========================================================

def read_terminal_command_nonblocking():
    try:
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip()
    except Exception:
        return None

    return None


def check_quit_key(window_enabled=True):
    terminal_cmd = read_terminal_command_nonblocking()

    if terminal_cmd is not None and terminal_cmd.lower() == "q":
        return True

    if window_enabled:
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            return True

    return False


def wait_with_preview_and_quit_check(seconds, cap, board, aruco_dict, K, dist, window_name):
    start = time.time()

    last_frame = None
    last_annotated = None
    last_result = None

    while time.time() - start < seconds:
        ret, frame = cap.read()

        if ret and frame is not None:
            result, annotated = detect_charuco(
                frame,
                board,
                aruco_dict,
                K=K,
                dist=dist
            )

            remaining = max(0.0, seconds - (time.time() - start))

            cv2.putText(
                annotated,
                f"Waiting after move: {remaining:.1f}s",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.putText(
                annotated,
                f"Markers: {result['marker_count']} | Corners: {result['charuco_corner_count']}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            if K is None or dist is None:
                status = "K, dist: not loaded"
                color = (0, 0, 255)
            else:
                status = "K, dist: loaded"
                color = (0, 255, 0)

            cv2.putText(
                annotated,
                status,
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2
            )

            cv2.putText(
                annotated,
                "Press q to stop",
                (20, annotated.shape[0] - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            cv2.imshow(window_name, annotated)

            last_frame = frame.copy()
            last_annotated = annotated.copy()
            last_result = result

        if check_quit_key(window_enabled=True):
            return True, last_frame, last_annotated, last_result

        time.sleep(0.03)

    return False, last_frame, last_annotated, last_result


def wait_until_robot_stops(mc, stable_time=ROBOT_STABLE_SEC, timeout=ROBOT_STOP_TIMEOUT_SEC):
    """
    send_angles 이후 로봇이 완전히 멈출 때까지 대기.
    is_moving()이 지원되지 않거나 신뢰하기 어려운 경우에는 False를 반환하지 않고
    경고만 출력되도록 main에서 WAIT_AFTER_MOVE_SEC 추가 대기를 병행합니다.
    """
    start = time.time()
    stable_start = None

    while time.time() - start < timeout:
        try:
            moving = mc.is_moving()
        except Exception as e:
            print("[경고] is_moving() 확인 실패:", e)
            return False

        if moving == 0:
            if stable_start is None:
                stable_start = time.time()

            if time.time() - stable_start >= stable_time:
                return True
        else:
            stable_start = None

        time.sleep(0.05)

    return False


# =========================================================
# 7. Intrinsic calibration / pose 재계산
# =========================================================

def calibrate_intrinsic_from_samples(samples, image_size, board):
    all_charuco_corners = []
    all_charuco_ids = []

    for sample in samples:
        corners = sample.get("_charuco_corners_np")
        ids = sample.get("_charuco_ids_np")

        if corners is None or ids is None:
            continue

        if len(ids) < MIN_CHARUCO_CORNERS:
            continue

        all_charuco_corners.append(corners.astype(np.float32))
        all_charuco_ids.append(ids.astype(np.int32))

    print("\n=== ChArUco intrinsic calibration 준비 ===")
    print("사용 가능한 샘플 수:", len(all_charuco_corners))
    print("image_size:", image_size)

    if len(all_charuco_corners) < MIN_INTRINSIC_SAMPLES:
        print("[경고] K, dist 계산용 샘플이 부족합니다.")
        print(f"최소 {MIN_INTRINSIC_SAMPLES}장 이상 필요합니다.")
        return None, None, None

    if hasattr(cv2.aruco, "calibrateCameraCharuco"):
        print("[정보] cv2.aruco.calibrateCameraCharuco() 사용")

        rms, K, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=all_charuco_corners,
            charucoIds=all_charuco_ids,
            board=board,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None
        )

    else:
        print("[정보] calibrateCameraCharuco() 없음. cv2.calibrateCamera() fallback 사용")

        object_points = []
        image_points = []

        for sample in samples:
            obj = sample.get("_object_points")
            img = sample.get("_image_points")

            if obj is None or img is None:
                continue

            if len(obj) < MIN_CHARUCO_CORNERS:
                continue

            object_points.append(obj.reshape(-1, 1, 3).astype(np.float32))
            image_points.append(img.reshape(-1, 1, 2).astype(np.float32))

        if len(object_points) < MIN_INTRINSIC_SAMPLES:
            print("[경고] fallback calibration용 샘플이 부족합니다.")
            return None, None, None

        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None
        )

    np.savez(
        AUTO_INTRINSIC_FILE,
        K=K,
        dist=dist,
        rms_error=rms,
        image_size=image_size
    )

    # 최신 intrinsic을 기본 이름으로도 저장
    np.savez(
        INTRINSIC_FILE,
        K=K,
        dist=dist,
        rms_error=rms,
        image_size=image_size
    )

    print("\n=== ChArUco Intrinsic Calibration 완료 ===")
    print("RMS reprojection error:", rms)
    print("K:")
    print(K)
    print("dist:")
    print(dist)
    print("저장 완료:", AUTO_INTRINSIC_FILE)
    print("기본 intrinsic 파일도 갱신:", INTRINSIC_FILE)

    return K, dist, rms


def recompute_pose_for_samples(samples, K, dist):
    pose_count = 0

    for sample in samples:
        obj = sample.get("_object_points")
        img = sample.get("_image_points")

        if obj is None or img is None:
            continue

        if len(obj) < MIN_CHARUCO_CORNERS:
            continue

        ok, rvec, tvec = cv2.solvePnP(
            obj.astype(np.float32),
            img.astype(np.float32),
            K,
            dist,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if ok:
            reproj_error = compute_reprojection_error_px(
                obj_points=obj,
                img_points=img,
                rvec=rvec,
                tvec=tvec,
                K=K,
                dist=dist
            )

            sample["charuco"]["pose_ok"] = True
            sample["charuco"]["rvec_target2cam"] = rvec.reshape(3).astype(float).tolist()
            sample["charuco"]["tvec_target2cam_mm"] = tvec.reshape(3).astype(float).tolist()
            sample["charuco"]["reprojection_error_px"] = reproj_error
            pose_count += 1

    print(f"[정보] K, dist 기반 pose 재계산 완료: {pose_count}개")


# =========================================================
# 8. Eye-in-hand Hand-eye calibration
# =========================================================

def rot_x(rad):
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ], dtype=np.float64)


def rot_y(rad):
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ], dtype=np.float64)


def rot_z(rad):
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ], dtype=np.float64)


def euler_to_R(rx_deg, ry_deg, rz_deg, order="xyz"):
    """
    [rx, ry, rz] degree를 회전행렬로 변환.

    order="xyz"이면 R = Rx(rx) @ Ry(ry) @ Rz(rz)
    order="zyx"이면 R = Rz(rz) @ Ry(ry) @ Rx(rx)

    myCobot firmware/문서/좌표 convention에 따라 실제 해석이 다를 수 있으므로
    본 코드는 여러 order 후보를 테스트하고 target consistency가 가장 좋은 결과를 고릅니다.
    """
    rx = np.deg2rad(rx_deg)
    ry = np.deg2rad(ry_deg)
    rz = np.deg2rad(rz_deg)

    R_map = {
        "x": rot_x(rx),
        "y": rot_y(ry),
        "z": rot_z(rz),
    }

    R = np.eye(3, dtype=np.float64)

    for axis in order.lower():
        R = R @ R_map[axis]

    return R


def mycobot_coords_to_R_t(coords, euler_order):
    """
    Eye-in-hand 기준 robot pose 생성.

    MyCobot get_coords() = [x, y, z, rx, ry, rz]
    여기서는 이 값을 ^bT_g로 해석합니다.

    OpenCV calibrateHandEye 인자 이름:
        R_gripper2base, t_gripper2base

    실제 의미:
        ^bR_g, ^bt_g
        gripper 좌표계의 점을 base 좌표계로 변환하는 pose.
    """
    x, y, z, rx, ry, rz = np.asarray(coords, dtype=np.float64).reshape(6)

    R_b_g = euler_to_R(rx, ry, rz, euler_order)
    t_b_g = np.array([[x], [y], [z]], dtype=np.float64)

    return R_b_g, t_b_g


def make_T(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_T(T):
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]

    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def get_euler_orders_to_test():
    orders = []

    for order in EULER_ORDER_CANDIDATES:
        order = str(order).lower()
        if sorted(order) != ["x", "y", "z"]:
            print("[경고] 잘못된 Euler order 무시:", order)
            continue
        if order not in orders:
            orders.append(order)

    if not orders:
        orders = ["xyz"]

    return orders


def collect_handeye_inputs(samples, euler_order):
    """
    eye-in-hand hand-eye 입력 구성.

    robot:
        get_coords() -> ^bT_g

    vision:
        solvePnP / ChArUco -> ^cT_t

    calibrateHandEye 입력:
        R_gripper2base = ^bR_g
        t_gripper2base = ^bt_g
        R_target2cam   = ^cR_t
        t_target2cam   = ^ct_t

    calibrateHandEye 출력:
        ^gT_c
    """
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []
    valid_indices = []
    skipped_by_reproj = []

    for sample in samples:
        coords = sample.get("coords_after_move")
        charuco = sample.get("charuco", {})

        if coords is None or len(coords) != 6:
            continue

        if not charuco.get("pose_ok", False):
            continue

        if charuco.get("charuco_corner_count", 0) < MIN_CHARUCO_CORNERS:
            continue

        reproj_error = charuco.get("reprojection_error_px", None)

        if (
            MAX_REPROJECTION_ERROR_PX_FOR_HANDEYE is not None
            and reproj_error is not None
            and reproj_error > MAX_REPROJECTION_ERROR_PX_FOR_HANDEYE
        ):
            skipped_by_reproj.append((sample.get("index"), reproj_error))
            continue

        rvec = np.array(charuco["rvec_target2cam"], dtype=np.float64).reshape(3, 1)
        tvec = np.array(charuco["tvec_target2cam_mm"], dtype=np.float64).reshape(3, 1)

        # solvePnP 결과: ^cT_t
        R_c_t, _ = cv2.Rodrigues(rvec)

        # myCobot get_coords 결과: ^bT_g
        R_b_g, t_b_g = mycobot_coords_to_R_t(coords, euler_order=euler_order)

        R_gripper2base.append(R_b_g)
        t_gripper2base.append(t_b_g)
        R_target2cam.append(R_c_t)
        t_target2cam.append(tvec)

        valid_indices.append(sample["index"])

    return {
        "R_gripper2base": R_gripper2base,
        "t_gripper2base": t_gripper2base,
        "R_target2cam": R_target2cam,
        "t_target2cam": t_target2cam,
        "valid_indices": valid_indices,
        "skipped_by_reproj": skipped_by_reproj,
    }


def validate_handeye_target_consistency(
    R_gripper2base,
    t_gripper2base,
    R_target2cam,
    t_target2cam,
    T_gripper_camera
):
    """
    Eye-in-hand 검증.

    ChArUco target은 바닥/책상에 고정되어 있으므로
    각 샘플에서 계산한 base 기준 target pose가 거의 같아야 합니다.

    공식:
        ^bT_t_i = ^bT_g_i @ ^gT_c @ ^cT_t_i

    여기서:
        ^bT_g_i = myCobot get_coords()
        ^gT_c   = calibrateHandEye 출력
        ^cT_t_i = ChArUco solvePnP 결과
    """
    target_positions = []
    target_transforms = []

    for R_b_g, t_b_g, R_c_t, t_c_t in zip(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam
    ):
        T_base_gripper = make_T(R_b_g, t_b_g)
        T_camera_target = make_T(R_c_t, t_c_t)

        T_base_target = T_base_gripper @ T_gripper_camera @ T_camera_target

        target_transforms.append(T_base_target)
        target_positions.append(T_base_target[:3, 3])

    target_positions = np.array(target_positions, dtype=np.float64)

    mean = target_positions.mean(axis=0)
    std = target_positions.std(axis=0)

    errors = np.linalg.norm(target_positions - mean, axis=1)

    return {
        "target_position_mean_mm": mean.tolist(),
        "target_position_std_mm": std.tolist(),
        "target_position_error_mean_mm": float(errors.mean()),
        "target_position_error_rms_mm": float(np.sqrt(np.mean(errors ** 2))),
        "target_position_error_max_mm": float(errors.max()),
        "target_position_samples_mm": target_positions.tolist(),
    }


def std_quality_message(std_mm, rms_mm):
    max_std = float(np.max(np.asarray(std_mm, dtype=np.float64)))

    if max_std <= 2.0:
        level = "매우 좋음"
    elif max_std <= 5.0:
        level = "좋음 - pick-and-place 시연에 적당"
    elif max_std <= 10.0:
        level = "보통 - 집기 위치 보정 권장"
    elif max_std <= 20.0:
        level = "나쁨 - 캘리브레이션 재수행 권장"
    else:
        level = "매우 나쁨 - 좌표계/Euler/보드/샘플 오류 가능성 큼"

    return {
        "max_std_mm": max_std,
        "rms_mm": float(rms_mm),
        "level": level,
    }


def result_score(validation):
    """
    자동 선택 기준.
    RMS를 주 기준으로, max std를 보조 페널티로 사용.
    """
    std = np.asarray(validation["target_position_std_mm"], dtype=np.float64)
    rms = float(validation["target_position_error_rms_mm"])
    return rms + 0.25 * float(np.max(std))


def run_handeye_calibration(samples):
    print("\n=== Eye-in-hand Hand-eye calibration 준비 ===")
    print("mode:", CALIBRATION_MODE)
    print("전제: camera attached to gripper, ChArUco target fixed on floor/table")
    print("검증식: ^bT_t_i = ^bT_g_i @ ^gT_c @ ^cT_t_i")
    print("Euler order candidates:", get_euler_orders_to_test())
    print("reprojection error filter [px]:", MAX_REPROJECTION_ERROR_PX_FOR_HANDEYE)

    all_results = {}

    for euler_order in get_euler_orders_to_test():
        inputs = collect_handeye_inputs(samples, euler_order=euler_order)

        R_gripper2base = inputs["R_gripper2base"]
        t_gripper2base = inputs["t_gripper2base"]
        R_target2cam = inputs["R_target2cam"]
        t_target2cam = inputs["t_target2cam"]
        valid_indices = inputs["valid_indices"]

        print(f"\n=== Euler order: {euler_order} ===")
        print("valid sample count:", len(valid_indices))

        if inputs["skipped_by_reproj"]:
            print("reprojection error로 제외된 샘플:", inputs["skipped_by_reproj"])

        if len(valid_indices) < MIN_HANDEYE_SAMPLES:
            print("[경고] hand-eye calibration 샘플이 부족합니다.")
            print(f"최소 {MIN_HANDEYE_SAMPLES}개 이상 필요합니다.")
            continue

        for method_name, method in HANDEYE_METHODS.items():
            try:
                R_g_c, t_g_c = cv2.calibrateHandEye(
                    R_gripper2base,
                    t_gripper2base,
                    R_target2cam,
                    t_target2cam,
                    method=method
                )

                T_g_c = make_T(R_g_c, t_g_c)
                T_c_g = invert_T(T_g_c)

                validation = validate_handeye_target_consistency(
                    R_gripper2base,
                    t_gripper2base,
                    R_target2cam,
                    t_target2cam,
                    T_g_c
                )

                quality = std_quality_message(
                    validation["target_position_std_mm"],
                    validation["target_position_error_rms_mm"]
                )

                key = f"{euler_order}:{method_name}"

                all_results[key] = {
                    "euler_order": euler_order,
                    "method": method_name,
                    "T_gripper_camera": T_g_c,
                    "T_camera_gripper": T_c_g,
                    "R_gripper_camera": R_g_c,
                    "t_gripper_camera": t_g_c.reshape(3),
                    "valid_indices": valid_indices,
                    "validation": validation,
                    "quality": quality,
                    "score": result_score(validation),
                }

                print(f"\n--- {method_name} / euler={euler_order} ---")
                print("T_gripper_camera (^gT_c):")
                print(T_g_c)
                print("camera position in gripper [mm]:", T_g_c[:3, 3].tolist())
                print("target position std in base [mm]:", validation["target_position_std_mm"])
                print("target position RMS in base [mm]:", validation["target_position_error_rms_mm"])
                print("quality:", quality["level"])

            except Exception as e:
                print(f"[경고] {method_name} / euler={euler_order} hand-eye 실패:", e)

    if not all_results:
        print("[실패] 모든 hand-eye method가 실패했습니다.")
        return None

    # 선택
    if SELECTED_HANDEYE_METHOD != "AUTO_BEST":
        method_candidates = {
            key: value
            for key, value in all_results.items()
            if value["method"] == SELECTED_HANDEYE_METHOD
        }

        if method_candidates:
            selected_key = min(method_candidates.keys(), key=lambda k: method_candidates[k]["score"])
        else:
            print(f"[경고] SELECTED_HANDEYE_METHOD={SELECTED_HANDEYE_METHOD} 결과가 없어 AUTO_BEST로 선택합니다.")
            selected_key = min(all_results.keys(), key=lambda k: all_results[k]["score"])
    else:
        selected_key = min(all_results.keys(), key=lambda k: all_results[k]["score"])

    selected_result = all_results[selected_key]

    print("\n=== Eye-in-hand Hand-eye Calibration 결과 요약 ===")
    for key, item in sorted(all_results.items(), key=lambda kv: kv[1]["score"]):
        val = item["validation"]
        print(
            f"{key:16s} | "
            f"RMS={val['target_position_error_rms_mm']:.3f} mm | "
            f"std={np.round(val['target_position_std_mm'], 3)} | "
            f"score={item['score']:.3f}"
        )

    print("\nselected:", selected_key)
    print("selected quality:", selected_result["quality"]["level"])

    output = {
        "run_id": RUN_ID,
        "calibration_mode": "eye_in_hand",
        "description": {
            "setup": "camera attached to gripper, ChArUco target fixed on floor/table",
            "robot_pose": "myCobot get_coords() interpreted as ^bT_g, gripper to base",
            "vision_pose": "ChArUco solvePnP interpreted as ^cT_t, target to camera",
            "handeye_output": "^gT_c, camera to gripper",
            "validation": "^bT_t_i = ^bT_g_i @ ^gT_c @ ^cT_t_i should be nearly constant because target is fixed",
        },
        "selected_key": selected_key,
        "selected_method": selected_result["method"],
        "selected_euler_order": selected_result["euler_order"],
        "valid_indices": selected_result["valid_indices"],
        "max_reprojection_error_filter_px": MAX_REPROJECTION_ERROR_PX_FOR_HANDEYE,
        "all_results": {},
    }

    for key, item in all_results.items():
        output["all_results"][key] = {
            "euler_order": item["euler_order"],
            "method": item["method"],
            "T_gripper_camera": item["T_gripper_camera"].tolist(),
            "T_camera_gripper": item["T_camera_gripper"].tolist(),
            # backward compatible field name
            "T_cam2gripper": item["T_gripper_camera"].tolist(),
            "t_gripper_camera_mm": item["t_gripper_camera"].tolist(),
            "validation": item["validation"],
            "quality": item["quality"],
            "score": item["score"],
        }

    output["selected"] = output["all_results"][selected_key]

    with open(AUTO_HANDEYE_FILE, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=4, ensure_ascii=False)

    np.savez(
        AUTO_HANDEYE_NPZ,
        selected_key=selected_key,
        selected_method=selected_result["method"],
        selected_euler_order=selected_result["euler_order"],
        T_gripper_camera=selected_result["T_gripper_camera"],
        T_camera_gripper=selected_result["T_camera_gripper"],
        # backward compatible names
        T_cam2gripper=selected_result["T_gripper_camera"],
        R_cam2gripper=selected_result["R_gripper_camera"],
        t_cam2gripper=selected_result["t_gripper_camera"],
        valid_indices=np.array(selected_result["valid_indices"]),
    )

    print("\n=== Eye-in-hand Hand-eye Calibration 완료 ===")
    print("selected method:", selected_result["method"])
    print("selected euler order:", selected_result["euler_order"])
    print("결과 JSON:", AUTO_HANDEYE_FILE)
    print("결과 NPZ :", AUTO_HANDEYE_NPZ)

    return output


# =========================================================
# 9. 저장 함수
# =========================================================

def save_auto_outputs(samples):
    json_samples = []

    valid_indices = []
    robot_coords_list = []
    robot_angles_list = []
    sampled_angles_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    for sample in samples:
        clean = {}

        for k, v in sample.items():
            if k.startswith("_"):
                continue
            clean[k] = v

        json_samples.append(clean)

        charuco = sample.get("charuco", {})

        if charuco.get("pose_ok"):
            rvec = np.array(
                charuco["rvec_target2cam"],
                dtype=np.float64
            ).reshape(3, 1)

            tvec = np.array(
                charuco["tvec_target2cam_mm"],
                dtype=np.float64
            ).reshape(3, 1)

            R_target2cam, _ = cv2.Rodrigues(rvec)

            valid_indices.append(sample["index"])
            robot_coords_list.append(
                np.array(sample["coords_after_move"], dtype=np.float64)
            )
            robot_angles_list.append(
                np.array(sample["angles_after_move"], dtype=np.float64)
            )
            sampled_angles_list.append(
                np.array(sample["sampled_angles_command"], dtype=np.float64)
            )
            R_target2cam_list.append(R_target2cam)
            t_target2cam_list.append(tvec)

    with open(AUTO_OUTPUT_JSON, "w", encoding="utf-8") as f:
        json.dump(json_samples, f, indent=4, ensure_ascii=False)

    print(f"\n자동 샘플 JSON 저장 완료: {AUTO_OUTPUT_JSON}")

    if len(valid_indices) > 0:
        np.savez(
            AUTO_OUTPUT_NPZ,
            valid_indices=np.array(valid_indices),
            robot_coords=np.array(robot_coords_list),
            robot_angles=np.array(robot_angles_list),
            sampled_angles_command=np.array(sampled_angles_list),
            R_target2cam=np.array(R_target2cam_list),
            t_target2cam=np.array(t_target2cam_list)
        )

        print(f"자동 hand-eye용 NPZ 저장 완료: {AUTO_OUTPUT_NPZ}")
        print(f"pose 유효 샘플 수: {len(valid_indices)}")
    else:
        print("[경고] pose 유효 샘플이 없어 NPZ를 저장하지 않았습니다.")


# =========================================================
# 10. 메인
# =========================================================

def main():
    np.random.seed()

    angle_samples = load_previous_success_angles(SOURCE_JSON)
    K, dist = load_intrinsic()

    board, aruco_dict = create_charuco_board()

    cap = cv2.VideoCapture(CAMERA_ID)

    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다. CAMERA_ID를 확인하세요.")

    mc = MyCobot280(PORT, BAUD)
    mc.thread_lock = True
    print("\n로봇이 연결되었습니다.")

    try:
        mc.focus_all_servos()
        print("Servo ON")
    except Exception as e:
        print("[경고] focus_all_servos 실패:", e)

    window_name = f"Auto ChArUco Capture {RUN_ID}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    auto_samples = []
    image_size = None

    print("\n=== 자동 샘플링 시작 ===")
    print("q를 누르면 종료합니다.")
    print("저장 폴더:", AUTO_IMAGE_DIR)
    print("샘플 JSON:", AUTO_OUTPUT_JSON)
    print("샘플 NPZ :", AUTO_OUTPUT_NPZ)
    print("K 결과:", AUTO_INTRINSIC_FILE)
    print("Hand-eye 결과:", AUTO_HANDEYE_FILE)

    try:
        while True:
            if MAX_AUTO_SAMPLES is not None:
                if len(auto_samples) >= MAX_AUTO_SAMPLES:
                    print("\nMAX_AUTO_SAMPLES 도달")
                    break

            if check_quit_key(window_enabled=True):
                print("\nq 입력 감지. 종료합니다.")
                break

            index = len(auto_samples) + 1

            sampled_angles = sample_next_angles(angle_samples)
            sampled_angles = [round(float(a), 2) for a in sampled_angles]

            print(f"\n=== Auto sample #{index} ===")
            print("sampled angles command:", sampled_angles)

            mc.send_angles(sampled_angles, SPEED)

            if WAIT_UNTIL_ROBOT_STOPS:
                stopped = wait_until_robot_stops(mc)
                if stopped:
                    print("로봇 정지 확인 완료.")
                else:
                    print("[경고] 로봇 정지 확인 실패 또는 timeout. 추가 대기 후 촬영합니다.")

            print(f"send_angles 완료. {WAIT_AFTER_MOVE_SEC}초 추가 안정화 대기 후 촬영합니다.")

            should_quit, _, _, _ = wait_with_preview_and_quit_check(
                seconds=WAIT_AFTER_MOVE_SEC,
                cap=cap,
                board=board,
                aruco_dict=aruco_dict,
                K=K,
                dist=dist,
                window_name=window_name
            )

            if should_quit:
                print("\nq 입력 감지. 촬영 전 종료합니다.")
                break

            ret, frame = cap.read()

            if not ret or frame is None:
                print("[실패] 카메라 프레임을 읽지 못했습니다.")
                continue

            h, w = frame.shape[:2]
            image_size = (w, h)

            result, annotated = detect_charuco(
                frame,
                board,
                aruco_dict,
                K=K,
                dist=dist
            )

            cv2.putText(
                annotated,
                f"Captured auto sample #{index}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )
            cv2.imshow(window_name, annotated)
            cv2.waitKey(1)

            coords_after = mc.get_coords()
            angles_after = mc.get_angles()

            if not isinstance(coords_after, list) or len(coords_after) != 6:
                print("[경고] coords_after 읽기 실패:", coords_after)
                coords_after = None

            if not isinstance(angles_after, list) or len(angles_after) != 6:
                print("[경고] angles_after 읽기 실패:", angles_after)
                angles_after = None

            image_file = AUTO_IMAGE_DIR / f"auto_sample_{index:03d}.png"
            annotated_file = AUTO_IMAGE_DIR / f"auto_sample_{index:03d}_charuco.png"

            cv2.imwrite(str(image_file), frame)
            cv2.imwrite(str(annotated_file), annotated)

            charuco_ids_np = None
            charuco_corners_np = None

            if result["charuco_ids"] is not None and result["charuco_corners_px"] is not None:
                charuco_ids_np = np.array(result["charuco_ids"], dtype=np.int32).reshape(-1, 1)
                charuco_corners_np = np.array(result["charuco_corners_px"], dtype=np.float32).reshape(-1, 1, 2)

            sample = {
                "index": index,
                "time": time.time(),
                "sampled_angles_command": sampled_angles,
                "speed": SPEED,
                "wait_after_move_sec": WAIT_AFTER_MOVE_SEC,
                "coords_after_move": coords_after,
                "angles_after_move": angles_after,
                "image_file": str(image_file),
                "annotated_file": str(annotated_file),
                "charuco": {
                    "marker_count": result["marker_count"],
                    "marker_ids": result["marker_ids"],
                    "charuco_corner_count": result["charuco_corner_count"],
                    "charuco_ids": result["charuco_ids"],
                    "charuco_corners_px": result["charuco_corners_px"],
                    "pose_ok": result["pose_ok"],
                    "rvec_target2cam": result["rvec_target2cam"],
                    "tvec_target2cam_mm": result["tvec_target2cam_mm"],
                    "reprojection_error_px": result["reprojection_error_px"],
                },
                "_object_points": result["object_points"],
                "_image_points": result["image_points"],
                "_charuco_ids_np": charuco_ids_np,
                "_charuco_corners_np": charuco_corners_np,
            }

            auto_samples.append(sample)

            print("coords_after_move:", coords_after)
            print("angles_after_move:", angles_after)
            print("detected markers:", result["marker_count"])
            print("detected ChArUco corners:", result["charuco_corner_count"])
            print("pose_ok:", result["pose_ok"])

            if result["pose_ok"]:
                print("tvec_target2cam_mm:", result["tvec_target2cam_mm"])
            elif K is None or dist is None:
                print("pose_ok=False: K, dist가 아직 없으면 정상입니다. 종료 후 K를 계산하고 pose를 재계산합니다.")

            print("저장:", image_file)
            print("저장:", annotated_file)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt 감지. 종료합니다.")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        print("\n=== 후처리 시작 ===")
        print(f"총 자동 샘플 수: {len(auto_samples)}")

        if len(auto_samples) == 0:
            print("[경고] 저장된 샘플이 없습니다.")
            return

        # 1. K, dist가 없거나 강제 재계산 옵션이면 ChArUco 샘플로 intrinsic 계산
        if K is None or dist is None:
            if image_size is None:
                print("[경고] image_size가 없어 K, dist를 계산할 수 없습니다.")
            else:
                K_new, dist_new, rms = calibrate_intrinsic_from_samples(
                    auto_samples,
                    image_size,
                    board
                )

                if K_new is not None and dist_new is not None:
                    K, dist = K_new, dist_new

        # 2. K, dist가 확보되었으면 모든 샘플의 target2cam pose 재계산
        if K is not None and dist is not None:
            recompute_pose_for_samples(auto_samples, K, dist)
        else:
            print("[경고] K, dist가 없어 pose 재계산과 hand-eye calibration을 건너뜁니다.")

        # 3. 샘플 저장
        save_auto_outputs(auto_samples)

        # 4. hand-eye calibration 수행
        if K is not None and dist is not None:
            run_handeye_calibration(auto_samples)

        print("\n자동 촬영 + K + hand-eye calibration 종료")


if __name__ == "__main__":
    main()