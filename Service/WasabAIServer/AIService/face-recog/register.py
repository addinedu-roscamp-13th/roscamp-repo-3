#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
register.py — 웹캠으로 얼굴을 캡처해 face_db/known/<name>/ 에 저장.

InsightFace 로 얼굴이 잡힐 때만 저장하고, 저장 후 encodings.pkl 캐시를
무효화한다(다음 perception_node.py 실행 시 자동 재임베딩).

실행:
    python register.py --name stephen
    SPACE: 캡처 | q: 종료
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import yaml

try:
    from insightface.app import FaceAnalysis
except ImportError:
    print("[ERROR] insightface 미설치 → pip install -r requirements.txt")
    sys.exit(1)

from udp_stream import UDPFrameReceiver


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def register(name: str, cap, known_dir: Path, app: FaceAnalysis,
             display_scale: float = 1.0) -> None:
    user_dir = known_dir / name
    user_dir.mkdir(parents=True, exist_ok=True)
    existing = list(user_dir.glob("*.jpg")) + list(user_dir.glob("*.png"))
    next_idx = len(existing) + 1
    captured = 0

    if not cap.isOpened():
        print("[ERROR] 카메라 열기 실패")
        return

    print(f"[register] '{name}' 등록 시작 | 기존 {len(existing)}장 | SPACE 캡처 / q 종료")
    fails = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            fails += 1
            if fails >= 100:   # ~5초 연속 실패 시 포기 (UDP 소스는 카메라 재기동 직후 잠깐 프레임이 없을 수 있음)
                print("[register] ❌ 카메라 프레임을 계속 못 받음 — 연결 확인 후 재시도하세요")
                break
            time.sleep(0.05)
            continue
        fails = 0
        frame = cv2.flip(frame, 1)
        clean = frame.copy()   # 오버레이 그리기 전 깨끗한 원본 보관(저장용)
        h, w = frame.shape[:2]
        has_face = len(app.get(frame)) > 0
        color = (0, 255, 100) if has_face else (0, 80, 200)
        status = "face OK - SPACE to capture" if has_face else "no face in view"

        cv2.putText(frame, f"User: {name}  saved: {len(existing) + captured}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(frame, status, (20, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        display = frame
        if display_scale != 1.0:
            display = cv2.resize(
                frame, None, fx=display_scale, fy=display_scale,
                interpolation=cv2.INTER_LINEAR)
        cv2.imshow("register", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            if not has_face:
                print("  ⚠️  얼굴 미감지 — 다시")
                continue
            save_path = user_dir / f"{next_idx + captured:03d}.jpg"
            cv2.imwrite(str(save_path), clean)   # 텍스트 오버레이 없는 깨끗한 프레임 저장
            captured += 1
            print(f"  ✅ {save_path}")
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    if captured:
        print(f"[register] '{name}' 총 {len(existing) + captured}장")
        cache = known_dir.parent / "encodings.pkl"
        if cache.exists():
            cache.unlink()
            print("[register] encodings.pkl 초기화 → 다음 실행 시 재임베딩")
    else:
        print("[register] 캡처 없이 종료")


def main() -> None:
    parser = argparse.ArgumentParser(description="face-recog 얼굴 등록")
    parser.add_argument("--name", help="등록자 이름 (미지정 시 프롬프트)")
    parser.add_argument("--camera", type=int, help="카메라 인덱스 (config 기본값 override)")
    parser.add_argument("--source", help="로컬 카메라 인덱스(정수). --local 모드에서 사용")
    parser.add_argument("--udp-port", type=int,
                        help="JetCobot cam_server.py UDP 수신 포트(지정 시 로컬 카메라보다 우선)")
    parser.add_argument("--config", default="config.yaml", help="설정 파일 경로")
    parser.add_argument("--display-scale", type=float, default=1.0,
                        help="등록 화면 표시 배율(저장 프레임은 원본 유지)")
    try:
        import argcomplete
        argcomplete.autocomplete(parser)
    except ImportError:
        pass
    args = parser.parse_args()

    cfg = _load_config(args.config)
    known_dir = Path(cfg["face_db"]["dir"]) / "known"

    name = args.name or input("등록할 이름 입력 (예: stephen): ").strip()
    if not name:
        print("[ERROR] 이름 필요")
        sys.exit(1)

    print(f"[register] InsightFace 로드: {cfg['model']['name']} ...")
    app = FaceAnalysis(name=cfg["model"]["name"], providers=cfg["model"]["providers"])
    app.prepare(ctx_id=0, det_size=(cfg["model"]["det_size"], cfg["model"]["det_size"]))

    if args.udp_port is not None:
        cap = UDPFrameReceiver(args.udp_port)
    else:
        src = args.source if args.source is not None else (
            args.camera if args.camera is not None else cfg["camera"]["index"])
        cam_idx = int(src) if isinstance(src, str) and src.isdigit() else src
        cap = cv2.VideoCapture(cam_idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    register(name, cap, known_dir, app, args.display_scale)


if __name__ == "__main__":
    import os
    os.chdir(Path(__file__).parent)
    main()
