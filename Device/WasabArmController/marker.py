import os

# cv2 import 전에 설정해야 함
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import sys
import json
import time
import select
from pathlib import Path

import cv2
import numpy as np
from pymycobot.mycobot280 import MyCobot280


# =========================================================
# 1. 사용자 설정
# =========================================================

PORT = "/dev/ttyJETCOBOT"
BAUD = 1000000

CAMERA_ID = 0

# =========================================================
# ChArUco 보드 설정
# =========================================================
# 업로드한 calib.io PDF 기준:
# 8x11 | Checker Size: 15 mm | Marker Size: 11 mm | Dictionary: ArUco DICT_4X4
#
# 만약 ArUco marker는 많이 잡히는데 ChArUco corner가 0개에 가깝다면,
# SQUARES_X = 11
# SQUARES_Y = 8
# 로 바꿔서 다시 테스트하세요.
# =========================================================

SQUARES_X = 11
SQUARES_Y = 8

SQUARE_LENGTH_MM = 15.0
MARKER_LENGTH_MM = 11.0

ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50

MIN_CHARUCO_CORNERS = 6
MIN_CALIBRATION_SAMPLES = 10

INTRINSIC_FILE = "camera_intrinsic_charuco.npz"
OUTPUT_JSON = "saved_robot_charuco_points.json"
OUTPUT_NPZ = "handeye_charuco_samples.npz"

IMAGE_DIR = Path("charuco_captures")
IMAGE_DIR.mkdir(exist_ok=True)

SHOW_WINDOW = True


# =========================================================
# 2. OpenCV / ChArUco 유틸 함수
# =========================================================

def get_aruco_dictionary():
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)

    return cv2.aruco.Dictionary_get(ARUCO_DICT_ID)


def create_charuco_board():
    aruco_dict = get_aruco_dictionary()

    try:
        board = cv2.aruco.CharucoBoard(
            (SQUARES_X, SQUARES_Y),
            SQUARE_LENGTH_MM,
            MARKER_LENGTH_MM,
            aruco_dict
        )
    except Exception:
        board = cv2.aruco.CharucoBoard_create(
            SQUARES_X,
            SQUARES_Y,
            SQUARE_LENGTH_MM,
            MARKER_LENGTH_MM,
            aruco_dict
        )

    # calib.io / OpenCV 버전 차이 때문에 필요한 경우가 있음
    # ChArUco corner가 안 잡히면 True/False를 바꿔보세요.
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(True)

    return board, aruco_dict


def get_charuco_object_points(board, charuco_ids):
    """
    검출된 ChArUco corner id에 해당하는 3D board 좌표를 반환.
    단위: mm
    """

    ids = charuco_ids.flatten().astype(int)

    if hasattr(board, "getChessboardCorners"):
        chessboard_corners = np.asarray(
            board.getChessboardCorners(),
            dtype=np.float32
        ).reshape(-1, 3)

        return chessboard_corners[ids].astype(np.float32)

    # fallback
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


def detect_charuco(frame, board, aruco_dict, K=None, dist=None):
    """
    카메라 프레임에서 ChArUco 보드 검출.
    K, dist가 있으면 target2cam pose까지 계산.
    """

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    annotated = frame.copy()

    result = {
        "marker_count": 0,
        "marker_ids": None,
        "charuco_corner_count": 0,
        "charuco_ids": None,
        "pose_ok": False,
        "rvec_target2cam": None,
        "tvec_target2cam_mm": None,

        # 내부 계산용 numpy array
        "object_points": None,
        "image_points": None,
        "charuco_corners_array": None,
        "charuco_ids_array": None,
    }

    charuco_corners = None
    charuco_ids = None
    marker_corners = None
    marker_ids = None

    # 최신 OpenCV 방식
    if hasattr(cv2.aruco, "CharucoDetector"):
        try:
            charuco_detector = cv2.aruco.CharucoDetector(board)
            charuco_corners, charuco_ids, marker_corners, marker_ids = (
                charuco_detector.detectBoard(gray)
            )
        except Exception as e:
            print("[경고] CharucoDetector.detectBoard 실패:", e)

    # 구버전 fallback
    if charuco_corners is None and hasattr(cv2.aruco, "interpolateCornersCharuco"):
        try:
            if hasattr(cv2.aruco, "DetectorParameters"):
                detector_params = cv2.aruco.DetectorParameters()
            else:
                detector_params = cv2.aruco.DetectorParameters_create()

            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                gray,
                aruco_dict,
                parameters=detector_params
            )

            if marker_ids is not None and len(marker_ids) > 0:
                if K is not None and dist is not None:
                    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        marker_corners,
                        marker_ids,
                        gray,
                        board,
                        cameraMatrix=K,
                        distCoeffs=dist
                    )
                else:
                    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        marker_corners,
                        marker_ids,
                        gray,
                        board
                    )
        except Exception as e:
            print("[경고] interpolateCornersCharuco fallback 실패:", e)

    marker_count = 0 if marker_ids is None else len(marker_ids)
    result["marker_count"] = int(marker_count)

    if marker_ids is not None:
        ids_list = marker_ids.flatten().astype(int).tolist()
        result["marker_ids"] = ids_list

    if marker_corners is not None and marker_ids is not None:
        cv2.aruco.drawDetectedMarkers(
            annotated,
            marker_corners,
            marker_ids
        )

    if charuco_corners is None or charuco_ids is None:
        return result, annotated

    charuco_count = len(charuco_ids)

    result["charuco_corner_count"] = int(charuco_count)
    result["charuco_ids"] = charuco_ids.flatten().astype(int).tolist()
    result["charuco_corners_array"] = charuco_corners.astype(np.float32)
    result["charuco_ids_array"] = charuco_ids.astype(np.int32)

    cv2.aruco.drawDetectedCornersCharuco(
        annotated,
        charuco_corners,
        charuco_ids,
        (0, 255, 0)
    )

    if charuco_count < MIN_CHARUCO_CORNERS:
        return result, annotated

    obj_points = get_charuco_object_points(board, charuco_ids)
    img_points = charuco_corners.reshape(-1, 2).astype(np.float32)

    result["object_points"] = obj_points
    result["image_points"] = img_points

    # K, dist가 있을 때만 pose 계산 가능
    if K is not None and dist is not None:
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            K,
            dist,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if ok:
            result["pose_ok"] = True
            result["rvec_target2cam"] = rvec.reshape(3).astype(float).tolist()
            result["tvec_target2cam_mm"] = tvec.reshape(3).astype(float).tolist()

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
# 3. K, dist 계산 함수
# =========================================================

def load_intrinsic_if_exists():
    if not os.path.exists(INTRINSIC_FILE):
        print(f"[정보] {INTRINSIC_FILE} 없음")
        print("[정보] 저장된 ChArUco 샘플로 종료 시 K, dist를 계산합니다.")
        return None, None

    data = np.load(INTRINSIC_FILE)
    K = data["K"]
    dist = data["dist"]

    print(f"[정보] 기존 intrinsic 로드 완료: {INTRINSIC_FILE}")
    print("K:")
    print(K)
    print("dist:")
    print(dist)

    return K, dist


def calibrate_intrinsic_from_charuco_samples(samples, image_size, board):
    """
    ChArUco corner 샘플들로 K, dist 계산.
    """

    all_charuco_corners = []
    all_charuco_ids = []

    for sample in samples:
        corners = sample.get("_charuco_corners")
        ids = sample.get("_charuco_ids_array")

        if corners is None or ids is None:
            continue

        if len(ids) < MIN_CHARUCO_CORNERS:
            continue

        all_charuco_corners.append(corners.astype(np.float32))
        all_charuco_ids.append(ids.astype(np.int32))

    print("\n=== ChArUco intrinsic calibration 준비 ===")
    print("사용 가능한 샘플 수:", len(all_charuco_corners))
    print("image_size:", image_size)

    if len(all_charuco_corners) < MIN_CALIBRATION_SAMPLES:
        print("[경고] K, dist 계산용 샘플이 부족합니다.")
        print(f"최소 {MIN_CALIBRATION_SAMPLES}장 이상, 권장 15~30장 저장하세요.")
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
        print("[정보] calibrateCameraCharuco() 없음")
        print("[정보] cv2.calibrateCamera() fallback 사용")

        object_points = []
        image_points = []

        for sample in samples:
            obj = sample.get("_object_points")
            img = sample.get("_image_points")

            if obj is None or img is None:
                continue

            if len(obj) < MIN_CHARUCO_CORNERS:
                continue

            object_points.append(
                obj.reshape(-1, 1, 3).astype(np.float32)
            )
            image_points.append(
                img.reshape(-1, 1, 2).astype(np.float32)
            )

        if len(object_points) < MIN_CALIBRATION_SAMPLES:
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
    print(f"저장 완료: {INTRINSIC_FILE}")

    return K, dist, rms


def recompute_pose_for_samples(samples, K, dist):
    """
    K, dist를 구한 뒤 기존 샘플들의 target2cam pose를 다시 계산.
    """

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
            sample["charuco"]["pose_ok"] = True
            sample["charuco"]["rvec_target2cam"] = (
                rvec.reshape(3).astype(float).tolist()
            )
            sample["charuco"]["tvec_target2cam_mm"] = (
                tvec.reshape(3).astype(float).tolist()
            )
            pose_count += 1

    print(f"[정보] pose 재계산 완료: {pose_count}개 샘플")


# =========================================================
# 4. 저장 함수
# =========================================================

def save_outputs(samples):
    """
    JSON 저장 + hand-eye용 NPZ 저장.
    """

    json_samples = []

    valid_indices = []
    robot_coords_list = []
    robot_angles_list = []
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
                np.array(sample["coords"], dtype=np.float64)
            )
            robot_angles_list.append(
                np.array(sample["angles"], dtype=np.float64)
            )
            R_target2cam_list.append(R_target2cam)
            t_target2cam_list.append(tvec)

    with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
        json.dump(json_samples, f, indent=4, ensure_ascii=False)

    print(f"\nJSON 저장 완료: {OUTPUT_JSON}")

    if len(valid_indices) > 0:
        np.savez(
            OUTPUT_NPZ,
            valid_indices=np.array(valid_indices),
            robot_coords=np.array(robot_coords_list),
            robot_angles=np.array(robot_angles_list),
            R_target2cam=np.array(R_target2cam_list),
            t_target2cam=np.array(t_target2cam_list)
        )

        print(f"hand-eye용 NPZ 저장 완료: {OUTPUT_NPZ}")
        print(f"pose 유효 샘플 수: {len(valid_indices)}")
    else:
        print("[경고] pose 유효 샘플이 없어 NPZ를 저장하지 않았습니다.")


# =========================================================
# 5. 터미널 입력 / 샘플 저장 함수
# =========================================================

def read_terminal_command_nonblocking():
    """
    터미널 입력을 non-blocking으로 읽음.
    Enter만 누르면 "" 반환.
    q 입력 후 Enter면 "q" 반환.
    입력 없으면 None 반환.
    Linux 기준.
    """

    try:
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip()
    except Exception:
        return None

    return None


def save_current_sample(saved_samples, mc, frame, annotated, result):
    coords = mc.get_coords()
    angles = mc.get_angles()

    if not isinstance(coords, list) or len(coords) != 6:
        print("[실패] 좌표 읽기 실패:", coords)
        return False

    if not isinstance(angles, list) or len(angles) != 6:
        print("[실패] 각도 읽기 실패:", angles)
        return False

    index = len(saved_samples) + 1

    image_file = IMAGE_DIR / f"sample_{index:03d}.png"
    annotated_file = IMAGE_DIR / f"sample_{index:03d}_charuco.png"

    cv2.imwrite(str(image_file), frame)
    cv2.imwrite(str(annotated_file), annotated)

    sample = {
        "index": index,
        "time": time.time(),
        "coords": coords,
        "angles": angles,
        "image_file": str(image_file),
        "annotated_file": str(annotated_file),
        "charuco": {
            "marker_count": result["marker_count"],
            "marker_ids": result["marker_ids"],
            "charuco_corner_count": result["charuco_corner_count"],
            "charuco_ids": result["charuco_ids"],
            "pose_ok": result["pose_ok"],
            "rvec_target2cam": result["rvec_target2cam"],
            "tvec_target2cam_mm": result["tvec_target2cam_mm"],
        },

        # 내부 계산용. JSON 저장 시 제외됨.
        "_object_points": result["object_points"],
        "_image_points": result["image_points"],
        "_charuco_corners": result["charuco_corners_array"],
        "_charuco_ids_array": result["charuco_ids_array"],
    }

    saved_samples.append(sample)

    print(f"\n=== {index}번 위치 저장 완료 ===")
    print("coords:", coords)
    print("angles:", angles)

    print("\n--- ChArUco 처리 결과 ---")
    print("detected ArUco markers:", result["marker_count"])
    print("detected ChArUco corners:", result["charuco_corner_count"])

    if result["marker_ids"] is not None:
        ids = result["marker_ids"]
        print("marker id min/max:", min(ids), max(ids))

    if result["charuco_corner_count"] < MIN_CHARUCO_CORNERS:
        print("[경고] ChArUco corner가 너무 적습니다.")
        print("보드를 더 크게 보이게 하거나, 조명/초점/보드 설정을 확인하세요.")

    if result["pose_ok"]:
        print("pose estimation: OK")
        print("rvec_target2cam:", result["rvec_target2cam"])
        print("tvec_target2cam_mm:", result["tvec_target2cam_mm"])
    else:
        print("pose estimation: 아직 불가 또는 실패")
        print("K, dist가 없으면 첫 실행에서는 정상입니다.")

    print("원본 이미지:", image_file)
    print("검출 결과 이미지:", annotated_file)

    return True


# =========================================================
# 6. 메인 실행부
# =========================================================

def main():
    print("OpenCV version:", cv2.__version__)
    print("has cv2.aruco:", hasattr(cv2, "aruco"))
    print("has CharucoDetector:", hasattr(cv2.aruco, "CharucoDetector"))
    print("has calibrateCameraCharuco:", hasattr(cv2.aruco, "calibrateCameraCharuco"))

    board, aruco_dict = create_charuco_board()
    K, dist = load_intrinsic_if_exists()

    cap = cv2.VideoCapture(CAMERA_ID)

    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다. CAMERA_ID를 확인하세요.")

    # 필요하면 해상도 고정
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    mc = MyCobot280(PORT, BAUD)
    mc.thread_lock = True
    print("로봇이 연결되었습니다.")

    saved_samples = []
    image_size = None

    try:
        mc.release_all_servos()
        print("Free mode ON: release_all_servos()")
    except Exception as e:
        print("[경고] release_all_servos 실패:", e)

    print("\n카메라 화면이 열립니다.")
    print("저장:")
    print("  - 카메라 창 클릭 후 s 또는 Enter")
    print("  - 또는 터미널에서 Enter")
    print("종료:")
    print("  - 카메라 창 클릭 후 q")
    print("  - 또는 터미널에서 q 입력 후 Enter")
    print("\n첫 실행에서 K, dist가 없으면 Pose not available이 정상입니다.")
    print("여러 샘플을 저장한 뒤 q로 종료하면 K, dist를 계산합니다.")

    window_name = "Live ChArUco Preview"

    if SHOW_WINDOW:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    last_frame = None
    last_annotated = None
    last_result = None

    while True:
        ret, frame = cap.read()

        if not ret or frame is None:
            print("[실패] 카메라 프레임을 읽지 못했습니다.")
            time.sleep(0.1)
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

        marker_count = result["marker_count"]
        charuco_count = result["charuco_corner_count"]
        pose_ok = result["pose_ok"]

        cv2.putText(
            annotated,
            f"Markers: {marker_count}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        cv2.putText(
            annotated,
            f"ChArUco corners: {charuco_count}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        if K is None or dist is None:
            status_text = "K, dist: NOT loaded"
            status_color = (0, 0, 255)
        else:
            status_text = "K, dist: loaded"
            status_color = (0, 255, 0)

        cv2.putText(
            annotated,
            status_text,
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_color,
            2
        )

        if pose_ok:
            tvec = result["tvec_target2cam_mm"]
            pose_text = (
                f"Pose OK | "
                f"x={tvec[0]:.1f}, y={tvec[1]:.1f}, z={tvec[2]:.1f} mm"
            )
            pose_color = (0, 255, 0)
        else:
            pose_text = "Pose not available"
            pose_color = (0, 0, 255)

        cv2.putText(
            annotated,
            pose_text,
            (20, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            pose_color,
            2
        )

        cv2.putText(
            annotated,
            "Save: s / Enter / terminal Enter | Quit: q",
            (20, h - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        if SHOW_WINDOW:
            cv2.imshow(window_name, annotated)

        last_frame = frame.copy()
        last_annotated = annotated.copy()
        last_result = result

        key = -1

        if SHOW_WINDOW:
            key = cv2.waitKey(1) & 0xFF

        terminal_cmd = read_terminal_command_nonblocking()

        should_quit = False
        should_save = False

        if key == ord("q"):
            should_quit = True

        if terminal_cmd is not None and terminal_cmd.lower() == "q":
            should_quit = True

        if key == ord("s") or key == 13 or key == 10:
            should_save = True

        if terminal_cmd == "":
            should_save = True

        if should_quit:
            break

        if should_save:
            save_current_sample(
                saved_samples=saved_samples,
                mc=mc,
                frame=last_frame,
                annotated=last_annotated,
                result=last_result
            )

    cap.release()

    if SHOW_WINDOW:
        cv2.destroyAllWindows()

    try:
        mc.set_free_mode(0)
    except Exception:
        pass

    try:
        mc.focus_all_servos()
        print("\nFree mode OFF, Servo ON")
    except Exception as e:
        print("[경고] focus_all_servos 실패:", e)

    print(f"\n총 {len(saved_samples)}개 위치 저장")

    # K, dist가 없던 경우: 종료 시 ChArUco 샘플로 intrinsic calibration 수행
    if K is None or dist is None:
        if image_size is not None:
            K_new, dist_new, rms = calibrate_intrinsic_from_charuco_samples(
                saved_samples,
                image_size,
                board
            )

            if K_new is not None and dist_new is not None:
                K, dist = K_new, dist_new
                recompute_pose_for_samples(saved_samples, K, dist)
        else:
            print("[경고] image_size가 없어 intrinsic calibration을 수행하지 못했습니다.")

    save_outputs(saved_samples)

    print("\n프로그램 종료")


if __name__ == "__main__":
    main()