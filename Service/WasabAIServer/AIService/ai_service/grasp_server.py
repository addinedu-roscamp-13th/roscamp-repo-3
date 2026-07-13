#!/usr/bin/env python3
# encoding: utf-8
"""기존 ``uvicorn grasp_server:app`` 실행을 위한 호환용 진입점.

원격 딥러닝 서버를 별도로 사용하지 않습니다. 실제 구현은 ``app.main``에 있고,
권장 실행 명령은 프로젝트 루트에서 ``python run_server.py``입니다.
"""
from app.main import app

__all__ = ["app"]
