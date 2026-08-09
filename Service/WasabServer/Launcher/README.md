# WaSaB 통합 실행기

PyQt6 기반 `wasab_launcher.py`를 실행하면 AI 서버, 양팔 클라이언트, 통합 WebService를
체크박스로 선택한 항목만 순서대로 시작합니다. 통합 GUI를 선택하면 준비 후
브라우저를 열며, 최초 실행에서만 관리자 계정을 입력합니다.

실행 항목은 `서버·GUI`, `Robot Arm`, `Mobile Robot` 세 카테고리로 구분됩니다.
`전체 선택` 체크박스로 모든 실행 항목을 한 번에 선택하거나 해제할 수 있습니다.
AI Server, 좌·우 로봇팔, 각 Pinky의 IP와 ROS Domain을 실행기에서 직접 변경할
수 있으며 입력값은 실제 HTTP, SSH, WebService, Console, agent 실행에 반영됩니다.

통합 WebService는 ROS domain 50 브리지와 로봇팔 AI Server(`192.168.2.8:8000`)
HTTP 프록시를 함께 사용합니다.

`Pinky-50/87/44/31`을 선택하면 해당 로봇에 SSH로 접속해 문서
`ros2_ansible.md`의 bringup, localization(map11), navigation, agent를 실행합니다.
노트북의 `192.168.2.x` 주소는 실행 시 자동 감지해 `ROS_STATIC_PEERS`로 전달합니다.
`DDS 네트워크 준비`를 선택하면 CycloneDDS 환경을 설정하고, 존재하는 `ap0`를
권한 확인 후 비활성화합니다.

실행기는 최대화 화면으로 열립니다. `로봇 GUI 열기`는 AI Server Admin GUI를
입력한 `AI Server IP`의 8000 포트에서 열고, `Console 열기`는 최신 ROS Console
창을 domain 50으로 실행합니다. 같은 IP가 통합 GUI의 로봇팔 API 프록시와
ESTOP 전달에도 사용됩니다.

바탕화면의 **WaSaB 통합 실행기** 아이콘을 더블클릭하여 사용합니다. 실행 로그는
`/tmp/wasab-launcher/`에 저장됩니다.

Ubuntu 바탕화면에는 `wasab_launcher_entry.c`로 빌드한 네이티브 실행 파일
`WaSaB`를 배치합니다. 이 파일은 텍스트 편집기 연결을 거치지 않고 PyQt6
실행기를 바로 시작합니다.
