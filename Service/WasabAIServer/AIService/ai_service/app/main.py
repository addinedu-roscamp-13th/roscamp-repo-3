#!/usr/bin/env python3
# encoding: utf-8
"""FastAPI entrypoint for the WaSaB laptop service.

The implementation lives under ``app.components`` so each runtime role can be
split without changing the uvicorn import path: ``app.main:app``.
"""
from __future__ import annotations

from app.components.wasab_web_service.service import app

__all__ = ["app"]
