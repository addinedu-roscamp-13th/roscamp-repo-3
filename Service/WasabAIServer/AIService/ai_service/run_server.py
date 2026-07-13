#!/usr/bin/env python3
"""노트북에서 YOLO + 3D 파지계획 HTTP 서비스를 실행한다.

라즈베리파이는 SSH 터널 없이 ``http://<노트북_LAN_IP>:8000``의
``/grasp-plan`` 또는 ``/v1/grasp-plan``으로 이미지를 전송한다.
"""
from __future__ import annotations

import uvicorn

from app.settings import settings


if __name__ == "__main__":
    print("[LAPTOP] YOLO + grasp-plan service")
    print(f"[LAPTOP] listening on http://{settings.host}:{settings.port}")
    print("[LAPTOP] Pi endpoint: /v1/grasp-plan (existing protocol preserved)")
    uvicorn.run("app.main:app", host=settings.host, port=settings.port, reload=False)
