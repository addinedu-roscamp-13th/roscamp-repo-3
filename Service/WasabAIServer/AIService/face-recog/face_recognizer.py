"""
face_recognizer — InsightFace(SCRFD 검출) + ArcFace 512d 임베딩 기반 closed-set 얼굴 인식.

face_db 구조:
    face_db/
    ├── encodings.pkl        ← 임베딩 캐시 (자동 생성/갱신)
    └── known/
        ├── stephen/         ← 서브디렉토리명 = 등록자 이름
        │   ├── 001.jpg
        │   └── 002.jpg
        └── hong/
            └── 001.jpg

등록된 사람의 얼굴 임베딩을 미리 저장해두고, 입력 프레임의 각 얼굴을
등록 임베딩과 cosine 유사도로 1:N 비교한다. best_sim >= tolerance 이면
해당 등록자 이름, 미만이면 미등록(None)으로 판정한다.

SmartGate(~/smartgate) FaceAuthenticator 를 closed-set 신원 인식용으로 다듬은 것.
"""
from __future__ import annotations

import pickle
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
    print("[WARNING] insightface 미설치 → pip install insightface onnxruntime")


@dataclass
class FaceResult:
    """프레임에서 검출된 얼굴 1개의 인식 결과."""
    bbox: tuple[int, int, int, int]   # (x1, y1, x2, y2)
    name: Optional[str]               # 등록자 이름 (미등록이면 None)
    similarity: float                 # best cosine 유사도 (0~1)
    detection_score: float = 1.0      # 얼굴 검출기 자체 신뢰도 (0~1)

    @property
    def is_known(self) -> bool:
        return self.name is not None


class FaceRecognizer:
    """
    InsightFace 기반 closed-set 얼굴 인식기.

    - face_db_dir   : face_db 루트 경로
    - tolerance     : cosine 유사도 임계값 (높을수록 엄격, 권장 0.35~0.50)
    - min_face_size : 최소 얼굴 크기(px). 이보다 작은 얼굴은 무시
    - model_name    : "buffalo_sc"(경량) | "buffalo_l"(고정밀)
    - det_size      : 검출 입력 크기
    - providers     : onnxruntime 실행 provider
    """

    def __init__(
        self,
        face_db_dir: str = "face_db",
        tolerance: float = 0.40,
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

        self.known_embeddings: list[np.ndarray] = []   # 각 (512,)
        self.known_names: list[str] = []
        self._app: Optional["FaceAnalysis"] = None
        self._known_mat: Optional[np.ndarray] = None   # (N, 512) 매칭용 캐시

        if INSIGHTFACE_AVAILABLE:
            self._init_model()
            self._load_known()

    # ── 모델 ──────────────────────────────────────────────
    def _init_model(self) -> None:
        print(f"[FaceRecognizer] InsightFace 로드: {self.model_name} ...")
        try:
            self._app = FaceAnalysis(name=self.model_name, providers=self.providers)
            self._app.prepare(ctx_id=0, det_size=(self.det_size, self.det_size))
            print(f"[FaceRecognizer] ✅ 모델 로드 완료")
        except Exception as e:
            print(f"[FaceRecognizer] ❌ 모델 로드 실패: {e}")
            self._app = None

    # ── 등록 얼굴 로드 ────────────────────────────────────
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

        # 캐시 우선: 캐시가 있고 (등록 이미지가 없거나 캐시가 모든 이미지보다 최신)
        if self.cache_path.exists() and (
                not image_files
                or self.cache_path.stat().st_mtime
                > max(f.stat().st_mtime for f in image_files)):
            with open(self.cache_path, "rb") as f:
                cache = pickle.load(f)
            self.known_embeddings = cache.get("embeddings", [])
            self.known_names = cache.get("names", [])
            self._finalize_known("캐시")
            return

        if not image_files:
            print(f"[FaceRecognizer] ⚠️  등록 얼굴 없음 → {self.known_dir}/<이름>/001.jpg")
            return

        if self._app is None:
            print("[FaceRecognizer] ❌ 모델 미초기화 → 임베딩 불가")
            return

        print("[FaceRecognizer] 등록 얼굴 임베딩 중...")
        self.known_embeddings.clear()
        self.known_names.clear()
        for img_path in image_files:
            name = img_path.parent.name
            img = cv2.imread(str(img_path))
            if img is None:
                print(f"  ❌ {img_path}: 읽기 실패")
                continue
            faces = self._app.get(img)
            # 등록 이미지에 배경 인물 등 여러 얼굴이 있을 수 있음 → 최소 크기 필터 후
            # 가장 큰(가까운) 얼굴을 등록 대상으로 선택 (faces[0] 은 크기순 보장 안 됨).
            # cf. ~/smartgate FaceAuthenticator.authenticate 의 검증된 패턴.
            valid = [f for f in faces if (f.bbox[3] - f.bbox[1]) >= self.min_face_size]
            if valid:
                face = max(valid, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
                self.known_embeddings.append(face.normed_embedding)  # 정규화 512d
                self.known_names.append(name)
                print(f"  ✅ {name} ← {img_path.name}")
            else:
                print(f"  ❌ {img_path.name}: 유효 얼굴 없음(너무 작거나 미검출)")

        with open(self.cache_path, "wb") as f:
            pickle.dump({"embeddings": self.known_embeddings, "names": self.known_names}, f)
        self._finalize_known("임베딩")

    def _finalize_known(self, src: str) -> None:
        self._known_mat = np.array(self.known_embeddings) if self.known_embeddings else None
        unique = list(dict.fromkeys(self.known_names))
        print(f"[FaceRecognizer] {src} 완료 | 등록자: {unique} | {len(self.known_embeddings)}장")

    def reload(self) -> None:
        """캐시 삭제 후 재임베딩."""
        if self.cache_path.exists():
            self.cache_path.unlink()
        self._load_known()

    # ── 인식 ──────────────────────────────────────────────
    def identify(self, frame_bgr: np.ndarray) -> list[FaceResult]:
        """
        프레임의 모든 얼굴을 검출하고 각각 등록자와 매칭.

        Returns: FaceResult 리스트 (얼굴마다 1개). 검출 0개면 빈 리스트.
                 등록자 없거나 모델 미준비면 빈 리스트.
        """
        if self._app is None or self._known_mat is None:
            return []

        results: list[FaceResult] = []
        for face in self._app.get(frame_bgr):
            x1, y1, x2, y2 = map(int, face.bbox)
            if (y2 - y1) < self.min_face_size:
                continue
            sims = self._known_mat @ face.normed_embedding   # (N,) cosine (정규화 완료)
            best_idx = int(np.argmax(sims))
            best_sim = float(sims[best_idx])
            name = self.known_names[best_idx] if best_sim >= self.tolerance else None
            results.append(
                FaceResult(
                    (x1, y1, x2, y2),
                    name,
                    best_sim,
                    float(getattr(face, "det_score", 1.0)),
                )
            )
        return results

    @property
    def is_ready(self) -> bool:
        return self._app is not None and self._known_mat is not None
