#!/usr/bin/env python3
"""Generate the exact ChArUco board used by auto_marker.py."""
from __future__ import annotations

from pathlib import Path

import cv2


SQUARES_X = 11
SQUARES_Y = 8
SQUARE_LENGTH_MM = 15.0
MARKER_LENGTH_MM = 11.0
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50
PIXELS_PER_MM = 20


def main() -> None:
    output_dir = Path(__file__).resolve().parents[1] / "images"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "charuco_11x8_15mm_4x4_50.png"

    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_MM,
        MARKER_LENGTH_MM,
        dictionary,
    )
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(True)

    width_px = round(SQUARES_X * SQUARE_LENGTH_MM * PIXELS_PER_MM)
    height_px = round(SQUARES_Y * SQUARE_LENGTH_MM * PIXELS_PER_MM)
    image = board.generateImage((width_px, height_px), marginSize=0, borderBits=1)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"failed to write {output_path}")

    print(output_path)
    print(f"board: {SQUARES_X * SQUARE_LENGTH_MM:.1f} x {SQUARES_Y * SQUARE_LENGTH_MM:.1f} mm")
    print(f"raster: {width_px} x {height_px} px ({PIXELS_PER_MM * 25.4:.0f} dpi)")


if __name__ == "__main__":
    main()
