#!/usr/bin/env python3
"""Run the wasab_통합 fire detector against the selected arm MJPEG stream.

This adapter preserves the dual-arm AdminGUI contract while using the latest
JetCobot fire detector/FSM from fire_detect_fsm.py.
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import rclpy

import fire_detect_fsm as fdn


def main() -> None:
    parser = argparse.ArgumentParser(description="왼팔 카메라용 최신 화재 감지 어댑터")
    parser.add_argument("--source", required=True)
    parser.add_argument("--console-domain", type=int, default=50)
    parser.add_argument("--response-file")
    parser.add_argument("--fire-topic", default="/wasab/k3/fire")
    parser.add_argument("--arm-still-topic", default="/wasab/k3/arm_still")
    parser.add_argument("--robot-id", default="WaSaBArm-left")
    parser.add_argument("--zone", default="D")
    parser.add_argument("--rate", type=float, default=8.0)
    parser.add_argument("--flip", action="store_true")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    source = int(args.source) if args.source.isdigit() else args.source
    capture = cv2.VideoCapture(source)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"카메라 스트림을 열 수 없습니다: {args.source}")

    response_file = Path(args.response_file) if args.response_file else None
    rclpy.init()
    node = fdn.FireDetectNode(
        args.fire_topic,
        args.console_domain,
        args.arm_still_topic,
        robot_id=args.robot_id,
        zone=args.zone,
    )
    print(
        f"[left_fire_detect] wasab_통합 최신 감지 사용 "
        f"S_MIN={fdn.S_MIN} H_LO_MAX={fdn.H_LO_MAX}",
        flush=True,
    )

    frame_buffer: list = []
    votes: list = []
    areas: list = []
    period = 1.0 / max(args.rate, 0.1)
    try:
        while rclpy.ok():
            started = time.monotonic()
            ok, frame = capture.read()
            if ok:
                if args.flip:
                    frame = cv2.flip(frame, 1)
                if response_file is not None and response_file.exists():
                    try:
                        response = response_file.read_text(encoding="utf-8").strip().lower()
                        response_file.unlink(missing_ok=True)
                        if response in {"yes", "no"}:
                            node._response = response
                    except OSError:
                        pass

                frame_buffer, votes, areas, bbox = fdn.process_frame(
                    node,
                    frame,
                    frame_buffer,
                    votes,
                    areas,
                    debug=args.debug,
                    ignore_arm_still=True,
                )
                if bbox is not None:
                    x, y, width, height = bbox
                    frame_height, frame_width = frame.shape[:2]
                    cx = (x + width / 2.0) / frame_width
                    cy = (y + height / 2.0) / frame_height
                    recent = votes[-5:]
                    confidence = sum(recent) / len(recent) if recent else 0.0
                    print(
                        f"VISION_TARGET cx={cx:.6f} cy={cy:.6f} conf={confidence:.6f}",
                        flush=True,
                    )

            rclpy.spin_once(node, timeout_sec=0.0)
            node.spin_console()
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        node.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
