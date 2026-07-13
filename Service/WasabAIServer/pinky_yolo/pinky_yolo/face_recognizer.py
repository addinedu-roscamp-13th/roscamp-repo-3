"""InsightFace SCRFD + ArcFace 얼굴 인식기."""

from __future__ import annotations

import pickle
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

try:
    from insightface.app import FaceAnalysis
    INSIGHTFACE_AVAILABLE = True
except ImportError:
    INSIGHTFACE_AVAILABLE = False
    FaceAnalysis = None  # type: ignore[assignment]


@dataclass(frozen=True)
class FaceResult:
    bbox: tuple[int, int, int, int]
    name: Optional[str]
    similarity: float
    embedding: Optional[np.ndarray] = None

    @property
    def is_known(self) -> bool:
        return self.name is not None


@dataclass
class _UnknownCandidate:
    embedding: np.ndarray
    sample_embeddings: list[np.ndarray]
    samples: list[np.ndarray]
    count: int
    last_seen: float
    last_frame_id: int | None = None


class FaceRecognizer:
    def __init__(
        self,
        face_db_dir: str = "face_db",
        tolerance: float = 0.45,
        min_face_size: int = 80,
        model_name: str = "buffalo_sc",
        det_size: int = 640,
        providers: Optional[list[str]] = None,
    ):
        self.face_db_dir = Path(face_db_dir)
        self.known_dir = self.face_db_dir / "known"
        self.cache_path = self.face_db_dir / "encodings.pkl"
        self.tolerance = tolerance
        self.min_face_size = min_face_size
        self.model_name = model_name
        self.det_size = det_size
        self.providers = providers or ["CPUExecutionProvider"]

        self.known_embeddings: list[np.ndarray] = []
        self.known_names: list[str] = []
        self._known_mat: np.ndarray | None = None
        self._unknown_candidates: list[_UnknownCandidate] = []
        self._app = None

        if INSIGHTFACE_AVAILABLE:
            self._init_model()
            self._load_known()

    def _init_model(self) -> None:
        print(f"[FaceRecognizer] InsightFace 로드: {self.model_name} ...")
        try:
            self._app = FaceAnalysis(name=self.model_name, providers=self.providers)
            self._app.prepare(ctx_id=0, det_size=(self.det_size, self.det_size))
            print("[FaceRecognizer] 모델 로드 완료")
        except Exception as exc:
            print(f"[FaceRecognizer] 모델 로드 실패: {exc}")
            self._app = None

    def _collect_image_files(self) -> list[Path]:
        files: list[Path] = []
        if not self.known_dir.exists():
            return files
        for user_dir in sorted(self.known_dir.iterdir()):
            if user_dir.is_dir():
                for ext in ("*.jpg", "*.jpeg", "*.png"):
                    files.extend(sorted(user_dir.glob(ext)))
        return files

    def _load_known(self) -> None:
        self.known_dir.mkdir(parents=True, exist_ok=True)
        image_files = self._collect_image_files()
        if not image_files:
            print("[FaceRecognizer] 등록된 얼굴이 없습니다. face_db/known/<id>/*.jpg")
            return

        if self.cache_path.exists() and self.cache_path.stat().st_mtime > max(
            file.stat().st_mtime for file in image_files
        ):
            try:
                with open(self.cache_path, "rb") as handle:
                    cache = pickle.load(handle)
                self.known_embeddings = cache.get("embeddings", [])
                self.known_names = cache.get("names", [])
                self._finalize_known("캐시")
                return
            except Exception:
                pass

        if self._app is None:
            print("[FaceRecognizer] 모델이 준비되지 않았습니다. 등록 얼굴 임베딩을 건너뜁니다.")
            return

        print("[FaceRecognizer] 등록 얼굴 임베딩을 생성합니다...")
        self.known_embeddings.clear()
        self.known_names.clear()
        for img_path in image_files:
            name = img_path.parent.name
            image = cv2.imread(str(img_path))
            if image is None:
                print(f"  ❌ {img_path}: 이미지 읽기 실패")
                continue
            faces = self._app.get(image)
            valid = [f for f in faces if int(f.bbox[3] - f.bbox[1]) >= self.min_face_size]
            if not valid:
                print(f"  ❌ {img_path.name}: 유효 얼굴이 없습니다")
                continue
            face = max(valid, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
            embedding = getattr(face, "normed_embedding", None)
            if embedding is None:
                embedding = getattr(face, "embedding", None)
            self.known_embeddings.append(self._normalize_embedding(embedding))
            self.known_names.append(name)
            print(f"  ✅ {name} ← {img_path.name}")

        self._save_cache()
        self._finalize_known("임베딩")

    def _save_cache(self) -> None:
        self.face_db_dir.mkdir(parents=True, exist_ok=True)
        with open(self.cache_path, "wb") as handle:
            pickle.dump({"embeddings": self.known_embeddings, "names": self.known_names}, handle)

    def _normalize_embedding(self, vector: object) -> np.ndarray:
        if vector is None:
            raise ValueError("임베딩을 생성할 수 없습니다")
        array = np.asarray(vector, dtype=np.float32).reshape(-1)
        norm = float(np.linalg.norm(array))
        if norm <= 0.0:
            raise ValueError("영벡터 임베딩")
        return array / norm

    def _finalize_known(self, source: str) -> None:
        if self.known_embeddings:
            self._known_mat = np.vstack(self.known_embeddings).astype(np.float32)
        else:
            self._known_mat = None
        unique_names = sorted(dict.fromkeys(self.known_names))
        print(f"[FaceRecognizer] {source} 완료 | 등록자: {unique_names} | 임베딩 개수: {len(self.known_embeddings)}")

    def reload(self) -> None:
        if self.cache_path.exists():
            self.cache_path.unlink()
        self._load_known()

    def identify(self, frame_bgr: np.ndarray) -> list[FaceResult]:
        if self._app is None:
            return []
        results: list[FaceResult] = []
        for face in self._app.get(frame_bgr):
            x1, y1, x2, y2 = map(int, face.bbox)
            if (y2 - y1) < self.min_face_size:
                continue
            similarity = 0.0
            name = None
            embedding = getattr(face, "normed_embedding", None)
            if embedding is None:
                embedding = getattr(face, "embedding", None)
            try:
                embedding = self._normalize_embedding(embedding)
            except Exception:
                continue
            if self._known_mat is not None:
                scores = self._known_mat @ embedding
                best_idx = int(np.argmax(scores))
                best_sim = float(scores[best_idx])
                if best_sim >= self.tolerance:
                    name = self.known_names[best_idx]
                similarity = best_sim
            results.append(FaceResult((x1, y1, x2, y2), name, similarity, embedding))
        return results

    def auto_enroll_unknowns(
        self,
        frame_bgr: np.ndarray,
        faces: list[FaceResult],
        frame_id: int,
        confirmations: int = 5,
        similarity_threshold: float = 0.65,
        prefix: str = "outsider",
        max_samples: int = 5,
        stale_seconds: float = 30.0,
    ) -> list[FaceResult]:
        now = time.monotonic()
        self._unknown_candidates = [
            candidate for candidate in self._unknown_candidates
            if now - candidate.last_seen <= stale_seconds
        ]

        updated_faces: list[FaceResult] = []
        for face in faces:
            if face.is_known or face.embedding is None:
                updated_faces.append(face)
                continue

            candidate = self._match_unknown_candidate(face.embedding, similarity_threshold)
            sample = self._crop_face(frame_bgr, face.bbox)
            if candidate is None:
                candidate = _UnknownCandidate(
                    embedding=face.embedding,
                    sample_embeddings=[face.embedding],
                    samples=[sample] if sample is not None else [],
                    count=1,
                    last_seen=now,
                    last_frame_id=frame_id,
                )
                self._unknown_candidates.append(candidate)
            else:
                candidate.last_seen = now
                if candidate.last_frame_id != frame_id:
                    candidate.count += 1
                    candidate.last_frame_id = frame_id
                    candidate.sample_embeddings.append(face.embedding)
                    candidate.embedding = self._average_embeddings(candidate.sample_embeddings)
                    if sample is not None and len(candidate.samples) < max_samples:
                        candidate.samples.append(sample)

            if candidate.count >= confirmations:
                name = self._enroll_unknown_candidate(candidate, prefix, max_samples)
                self._unknown_candidates.remove(candidate)
                updated_faces.append(FaceResult(face.bbox, name, 1.0, face.embedding))
            else:
                print(
                    f"[AUTO_ENROLL] unknown candidate {candidate.count}/{confirmations} "
                    f"bbox={face.bbox}",
                    flush=True,
                )
                updated_faces.append(face)

        return updated_faces

    def _match_unknown_candidate(
        self, embedding: np.ndarray, similarity_threshold: float
    ) -> _UnknownCandidate | None:
        best_candidate = None
        best_score = -1.0
        for candidate in self._unknown_candidates:
            score = float(candidate.embedding @ embedding)
            if score > best_score:
                best_candidate = candidate
                best_score = score
        if best_candidate is not None and best_score >= similarity_threshold:
            return best_candidate
        return None

    def _average_embeddings(self, embeddings: list[np.ndarray]) -> np.ndarray:
        stacked = np.vstack(embeddings).astype(np.float32)
        return self._normalize_embedding(stacked.mean(axis=0))

    def _crop_face(
        self, frame_bgr: np.ndarray, bbox: tuple[int, int, int, int], margin_ratio: float = 0.2
    ) -> np.ndarray | None:
        height, width = frame_bgr.shape[:2]
        x1, y1, x2, y2 = bbox
        margin = int(max(x2 - x1, y2 - y1) * margin_ratio)
        x1 = max(0, x1 - margin)
        y1 = max(0, y1 - margin)
        x2 = min(width, x2 + margin)
        y2 = min(height, y2 + margin)
        if x2 <= x1 or y2 <= y1:
            return None
        return frame_bgr[y1:y2, x1:x2].copy()

    def _next_auto_id(self, prefix: str) -> str:
        self.known_dir.mkdir(parents=True, exist_ok=True)
        max_number = 0
        marker = prefix + "_"
        for user_dir in self.known_dir.iterdir():
            if not user_dir.is_dir() or not user_dir.name.startswith(marker):
                continue
            suffix = user_dir.name[len(marker):]
            if suffix.isdigit():
                max_number = max(max_number, int(suffix))
        return f"{prefix}_{max_number + 1:03d}"

    def _enroll_unknown_candidate(
        self, candidate: _UnknownCandidate, prefix: str, max_samples: int
    ) -> str:
        name = self._next_auto_id(prefix)
        user_dir = self.known_dir / name
        user_dir.mkdir(parents=True, exist_ok=True)

        samples = candidate.samples[:max_samples]
        for index, sample in enumerate(samples, start=1):
            cv2.imwrite(str(user_dir / f"sample_{index:03d}.jpg"), sample)

        embeddings = candidate.sample_embeddings[:max_samples]
        self.known_embeddings.extend(embeddings)
        self.known_names.extend([name] * len(embeddings))
        self._finalize_known("자동 등록")
        self._save_cache()
        print(
            f"[AUTO_ENROLL] saved {name} to {user_dir} "
            f"samples={len(samples)} embeddings={len(embeddings)}",
            flush=True,
        )
        return name

    @property
    def is_ready(self) -> bool:
        return self._app is not None
