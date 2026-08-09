# Wasab WebService

통합 사용자 GUI의 FastAPI 백엔드입니다. 프런트엔드는
`UI/mobile/user_gui/frontend`에서 정적으로 제공합니다.

## 실행

저장소 루트에서:

```bash
WASAB_WEBAPP_ADMIN=admin \
WASAB_WEBAPP_ADMIN_PW='8자 이상의 초기 비밀번호' \
./Service/WasabServer/scripts/run_webapp.sh --no-ros
```

기본 접속 주소는 `http://127.0.0.1:8100`입니다. 로봇팔 서버 주소는
`WASAB_ARM_API_URL`로 변경할 수 있으며 기본값은 `http://192.168.2.8:8000`입니다.
최초 실행에서만 관리자 환경변수가 필요하며 생성된 계정 파일은 Git에서 제외됩니다.

## 구조

- `wasab_web_service/`: 인증, fleet, 카메라, 로봇팔 API 프록시
- `config/robots.yaml`: 통합 GUI 로봇 구성
- `tests/`: WebService 테스트
