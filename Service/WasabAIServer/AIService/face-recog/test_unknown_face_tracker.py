import pytest

from face_recognizer import FaceResult
from unknown_face_tracker_node import (
    largest_unknown_result,
    unknown_confidence,
)


def test_largest_unknown_ignores_known_and_uses_face_area():
    known = FaceResult((0, 0, 200, 200), "teacher", 0.8, 0.99)
    small_unknown = FaceResult((0, 0, 50, 50), None, 0.1, 0.95)
    large_unknown = FaceResult((0, 0, 100, 80), None, 0.1, 0.95)

    assert largest_unknown_result(
        [known, small_unknown, large_unknown]
    ) is large_unknown


def test_largest_unknown_returns_none_when_only_known_faces():
    known = FaceResult((0, 0, 100, 100), "teacher", 0.8, 0.99)

    assert largest_unknown_result([known]) is None


def test_unknown_confidence_rejects_borderline_known_match():
    result = FaceResult(
        bbox=(0, 0, 100, 100),
        name=None,
        similarity=0.39,
        detection_score=0.9,
    )

    assert unknown_confidence(result, tolerance=0.4) == pytest.approx(0.0225)


def test_unknown_confidence_accepts_strong_unknown_face():
    result = FaceResult(
        bbox=(0, 0, 100, 100),
        name=None,
        similarity=0.1,
        detection_score=0.9,
    )

    assert unknown_confidence(result, tolerance=0.4) == pytest.approx(0.675)


def test_known_face_has_zero_unknown_confidence():
    result = FaceResult(
        bbox=(0, 0, 100, 100),
        name="teacher",
        similarity=0.8,
        detection_score=0.99,
    )

    assert unknown_confidence(result, tolerance=0.4) == 0.0
