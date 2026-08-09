#!/usr/bin/env python3
"""PyQt6 WaSaB launcher for the AI server, arm clients, and integrated GUI."""
from __future__ import annotations

import os
import ipaddress
import signal
import shlex
import socket
import subprocess
import sys
import threading
import time
import traceback
import urllib.request
import webbrowser
from pathlib import Path

from PyQt6.QtCore import QObject, Qt, pyqtSignal
from PyQt6.QtGui import QCloseEvent, QFont
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
AI_ROOT = PROJECT_ROOT / "Service/WasabAIServer/AIService/ai_service"
WEB_ROOT = PROJECT_ROOT / "Service/WasabServer/WebService"
WEB_DATA = WEB_ROOT / "wasab_web_service/data/teachers.json"
WEB_SCRIPT = PROJECT_ROOT / "Service/WasabServer/scripts/run_webapp.sh"
SERVER_PYTHON = Path("/home/ane/dev_ws/.venv-server/bin/python")
LOG_ROOT = Path("/tmp/wasab-launcher")
GUI_URL = "http://127.0.0.1:8100"
DEFAULT_AI_SERVER_IP = "192.168.2.8"
CONSOLE_ROOT = Path("/home/ane/dev_ws/wasab")
CONSOLE_SCRIPT = CONSOLE_ROOT / "scripts/run_console.sh"

ARM_CLIENTS = {"left": "192.168.2.10", "right": "192.168.2.12"}
ARM_USER = "jetcobot"
ARM_PASSWORD = "x"
ARM_WORKDIR = "/home/jetcobot/wasab/roscamp-repo-3/Device/WasabBot/WasabArmController"
ARM_PYTHON = "/home/jetcobot/venv/wasabarm/bin/python"
PINKY_USER = "pinky"
PINKY_PASSWORD = "1"
PINKY_ROBOTS = {
    "pinky50": {"id": 50, "host": "192.168.2.9", "domain": 51},
    "pinky87": {"id": 87, "host": "192.168.2.13", "domain": 52},
    "pinky44": {"id": 44, "host": "192.168.2.11", "domain": 53},
    "pinky31": {"id": 31, "host": "192.168.2.15", "domain": 54},
}


class UiBridge(QObject):
    log = pyqtSignal(str)
    status = pyqtSignal(str, str)
    error = pyqtSignal(str)
    starting = pyqtSignal(bool)


class WasabLauncher(QMainWindow):
    def __init__(self):
        super().__init__()
        self.processes: dict[str, subprocess.Popen] = {}
        self.log_files = []
        self.is_starting = False
        self.started_pinkies: set[str] = set()
        self.bridge = UiBridge()
        self.status_labels: dict[str, QLabel] = {}
        self.component_checks: dict[str, QCheckBox] = {}
        self.arm_ip_inputs: dict[str, QLineEdit] = {}
        self.arm_domain_inputs: dict[str, QLineEdit] = {}
        self.pinky_ip_inputs: dict[str, QLineEdit] = {}
        self.pinky_domain_inputs: dict[str, QLineEdit] = {}
        self._build_ui()

        self.bridge.log.connect(self.append_log)
        self.bridge.status.connect(self.set_status)
        self.bridge.error.connect(self.show_error)
        self.bridge.starting.connect(self.set_starting)


    def _build_ui(self):
        self.setWindowTitle("WaSaB 통합 실행기")
        self.resize(760, 680)
        self.setMinimumSize(620, 460)

        root = QWidget()
        root.setObjectName("root")
        layout = QVBoxLayout(root)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(14)

        title = QLabel("WaSaB 통합 실행기")
        title.setFont(QFont("Sans", 22, QFont.Weight.Bold))
        subtitle = QLabel("필요한 항목을 선택한 뒤 시작 버튼을 누르세요.")
        subtitle.setObjectName("subtitle")
        layout.addWidget(title)
        layout.addWidget(subtitle)

        select_card = QFrame()
        select_card.setObjectName("card")
        select_card.setMinimumHeight(405)
        select_layout = QVBoxLayout(select_card)
        select_layout.setContentsMargins(18, 14, 18, 14)
        select_header = QHBoxLayout()
        self.select_all_check = QCheckBox("ALL SELECT")
        self.select_all_check.setObjectName("selectAll")
        select_header.addWidget(self.select_all_check)
        select_header.addStretch()
        select_layout.addLayout(select_header)
        categories = QHBoxLayout()
        categories.setSpacing(12)

        def category(title_text: str):
            frame = QFrame()
            frame.setObjectName("category")
            box = QVBoxLayout(frame)
            box.setContentsMargins(14, 12, 14, 12)
            box.setSpacing(10)
            box.setAlignment(Qt.AlignmentFlag.AlignTop)
            heading = QLabel(title_text)
            heading.setObjectName("categoryTitle")
            heading.setFixedHeight(24)
            box.addWidget(heading)
            categories.addWidget(frame, 1)
            return box

        def field_row(label_text: str, field: QLineEdit):
            row = QHBoxLayout()
            row.setSpacing(10)
            label = QLabel(label_text)
            label.setFixedWidth(82)
            label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            field.setFixedHeight(36)
            row.addWidget(label)
            row.addWidget(field, 1)
            return row

        def add_check(box, key: str, text: str, checked: bool):
            checkbox = QCheckBox(text)
            checkbox.setChecked(checked)
            self.component_checks[key] = checkbox
            box.addWidget(checkbox)
            return checkbox

        def config_block(box, key: str, title_text: str, host: str, domain: int, checked: bool):
            frame = QFrame()
            frame.setObjectName("robotConfig")
            frame.setMinimumHeight(126)
            block = QVBoxLayout(frame)
            block.setContentsMargins(10, 0, 10, 8)
            block.setSpacing(8)
            add_check(block, key, title_text, checked)
            ip_input = QLineEdit(host)
            domain_input = QLineEdit(str(domain))
            block.addLayout(field_row("IP", ip_input))
            block.addLayout(field_row("Domain", domain_input))
            if box is not None:
                box.addWidget(frame)
            return ip_input, domain_input, frame

        server_box = category("1. Server · GUI")
        add_check(server_box, "ai", "AI Server", True)
        self.ai_ip_input = QLineEdit(DEFAULT_AI_SERVER_IP)
        self.ai_ip_input.setPlaceholderText("예: 192.168.2.8")
        server_box.addLayout(field_row("AI Server IP", self.ai_ip_input))
        add_check(server_box, "console", "ROS Console", True)
        self.console_domain_input = QLineEdit("50")
        server_box.addLayout(field_row("ROS Domain", self.console_domain_input))
        add_check(server_box, "web", "Integrated GUI", True)
        server_box.addStretch()

        arm_box = category("2. Robot Arm")
        arm_grid = QGridLayout()
        arm_grid.setSpacing(10)
        for index, (arm_id, label) in enumerate((('left', 'Left Arm'), ('right', 'Right Arm'))):
            ip_input, domain_input, frame = config_block(
                None, arm_id, label, ARM_CLIENTS[arm_id], 69, True,
            )
            self.arm_ip_inputs[arm_id] = ip_input
            self.arm_domain_inputs[arm_id] = domain_input
            arm_grid.addWidget(frame, 0, index)
        arm_box.addLayout(arm_grid)
        arm_box.addStretch()

        mobile_box = category("3. Mobile Robot")
        add_check(mobile_box, "dds", "DDS Network Setup", False)
        pinky_grid = QGridLayout()
        pinky_grid.setHorizontalSpacing(10)
        pinky_grid.setVerticalSpacing(12)
        for index, (key, spec) in enumerate(PINKY_ROBOTS.items()):
            ip_input, domain_input, frame = config_block(
                None, key, f"Pinky-{spec['id']}", spec["host"], spec["domain"], False,
            )
            self.pinky_ip_inputs[key] = ip_input
            self.pinky_domain_inputs[key] = domain_input
            pinky_grid.addWidget(frame, index // 2, index % 2)
        mobile_box.addLayout(pinky_grid)
        mobile_box.addStretch()
        select_layout.addLayout(categories)
        self._syncing_select_all = False
        self.select_all_check.toggled.connect(self._toggle_all_components)
        for checkbox in self.component_checks.values():
            checkbox.stateChanged.connect(self._sync_select_all)
        self._sync_select_all()
        layout.addWidget(select_card)

        status_card = QFrame()
        status_card.setObjectName("card")
        status_card.setMaximumHeight(230)
        status_layout = QVBoxLayout(status_card)
        status_layout.setContentsMargins(18, 16, 18, 16)
        status_layout.setSpacing(10)
        status_title = QLabel("SYSTEM STATUS")
        status_title.setObjectName("statusTitle")
        status_title.setFont(QFont("Sans", 13, QFont.Weight.Bold))
        status_layout.addWidget(status_title)

        status_grid = QGridLayout()
        status_grid.setHorizontalSpacing(18)
        status_grid.setVerticalSpacing(10)
        status_grid.setColumnStretch(0, 1)
        status_grid.setColumnStretch(2, 1)
        status_separator = QFrame()
        status_separator.setObjectName("statusSeparator")
        status_separator.setFrameShape(QFrame.Shape.VLine)
        status_separator.setFrameShadow(QFrame.Shadow.Plain)
        left_status_title = QLabel("SERVER · GUI · NETWORK")
        left_status_title.setObjectName("statusColumnTitle")
        right_status_title = QLabel("ROBOT ARM · MOBILE ROBOT")
        right_status_title.setObjectName("statusColumnTitle")
        status_grid.addWidget(left_status_title, 0, 0)
        status_grid.addWidget(right_status_title, 0, 2)
        status_grid.addWidget(status_separator, 0, 1, 7, 1)
        status_columns = (
            (("ai", "AI Server"), ("web", "Integrated GUI"),
             ("console", "ROS Console"), ("dds", "DDS Network Setup")),
            (("left", "Left Arm"), ("right", "Right Arm"),
             ("pinky50", "Pinky-50"), ("pinky87", "Pinky-87"),
             ("pinky44", "Pinky-44"), ("pinky31", "Pinky-31")),
        )
        for column_index, items in enumerate(status_columns):
            for row_index, (key, label) in enumerate(items):
                row = QHBoxLayout()
                name = QLabel(label)
                name.setObjectName("statusName")
                name.setFixedHeight(28)
                value = QLabel("●  Waiting")
                value.setObjectName("status")
                value.setProperty("kind", "idle")
                value.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
                value.setFixedHeight(28)
                value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                row.addWidget(name)
                row.addStretch()
                row.addWidget(value)
                status_grid.addLayout(row, row_index + 1, 0 if column_index == 0 else 2)
                self.status_labels[key] = value
        status_layout.addLayout(status_grid)
        layout.addWidget(status_card)

        buttons = QHBoxLayout()
        self.start_button = QPushButton("선택 항목 시작")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.start_system)
        open_button = QPushButton("통합 GUI 열기")
        open_button.clicked.connect(lambda: webbrowser.open(GUI_URL))
        robot_gui_button = QPushButton("로봇 GUI 열기")
        robot_gui_button.clicked.connect(self.open_robot_gui)
        console_button = QPushButton("Console 열기")
        console_button.clicked.connect(self.open_console)
        self.stop_button = QPushButton("전체 종료")
        self.stop_button.setObjectName("danger")
        self.stop_button.clicked.connect(self.stop_system)
        buttons.addWidget(self.start_button)
        buttons.addWidget(open_button)
        buttons.addWidget(robot_gui_button)
        buttons.addWidget(console_button)
        buttons.addStretch()
        buttons.addWidget(self.stop_button)
        layout.addLayout(buttons)

        layout.addWidget(QLabel("실행 로그"))
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setObjectName("log")
        layout.addWidget(self.log_view, 1)
        self.setCentralWidget(root)

        self.setStyleSheet("""
            #root { background: #f4f7fb; color: #182235; }
            #subtitle { color: #617087; }
            #card { background: white; border: 1px solid #dbe3ef; border-radius: 12px; }
            #category { background: #f8fafc; border: 1px solid #dbe3ef; border-radius: 9px; }
            #categoryTitle { color: #27405f; font-size: 15px; font-weight: 800; padding-bottom: 5px; }
            #selectAll { color: #173b70; font-size: 14px; font-weight: 900;
                         letter-spacing: 1px; padding: 2px 0; }
            #robotConfig { background: white; border: 1px solid #e1e7f0; border-radius: 7px; }
            #statusTitle { color: #172b49; letter-spacing: 1px; }
            #statusColumnTitle { color: #7a889c; font-size: 10px; font-weight: 700;
                                 letter-spacing: 1px; padding-bottom: 3px; }
            #statusName { color: #263a57; font-size: 12px; font-weight: 600;
                          letter-spacing: .3px; }
            #status { color: #59708f; background: #eef2f7; border-radius: 9px;
                      padding: 3px 9px; font-size: 11px; font-weight: 700; }
            #status[kind="ok"] { color: #147a50; background: #e6f7ef; }
            #status[kind="busy"] { color: #a76400; background: #fff2d8; }
            #status[kind="error"] { color: #b42335; background: #fdebed; }
            #status[kind="muted"] { color: #7b8798; background: #f1f3f6; }
            #statusSeparator { color: #dbe3ef; background: #dbe3ef; max-width: 1px; }
            QPushButton { min-height: 38px; padding: 0 18px; border: 1px solid #ccd7e5;
                          border-radius: 8px; background: white; font-weight: 600; }
            QPushButton:hover { background: #edf3fa; }
            QPushButton:disabled { color: #9aa7b8; background: #edf0f4; }
            QLineEdit { min-height: 34px; padding: 0 10px; border: 1px solid #ccd7e5;
                        border-radius: 7px; background: white; }
            QLineEdit:focus { border: 2px solid #4f7ff0; }
            QCheckBox { spacing: 7px; }
            #primary { color: white; background: #246bfd; border-color: #246bfd; }
            #primary:hover { background: #1659dd; }
            #danger { color: #c1364e; }
            #log { color: #d7e2f0; background: #101722; border: 0; border-radius: 10px;
                   padding: 8px; font-family: monospace; }
        """)

    def append_log(self, message: str):
        self.log_view.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def set_status(self, key: str, value: str):
        label = self.status_labels[key]
        translations = {
            "실행 중 (기존)": "Running · Existing",
            "실행 중": "Running",
            "준비 완료": "Ready",
            "시작 중": "Starting",
            "연결 중": "Connecting",
            "설정 중": "Configuring",
            "실행 실패": "Failed",
            "연결 실패": "Connection Failed",
            "상태 확인 실패": "Status Failed",
            "선택 안 함": "Not Selected",
            "정지": "Stopped",
            "대기": "Waiting",
        }
        display_value = translations.get(value, value)
        label.setText(f"●  {display_value}")
        if "실행 중" in value or "준비 완료" in value:
            kind = "ok"
        elif any(token in value for token in ("시작 중", "연결 중", "설정 중")):
            kind = "busy"
        elif any(token in value for token in ("실패", "오류")):
            kind = "error"
        elif any(token in value for token in ("선택 안 함", "정지")):
            kind = "muted"
        else:
            kind = "idle"
        label.setProperty("kind", kind)
        label.style().unpolish(label)
        label.style().polish(label)
        label.adjustSize()
        label.setFixedHeight(28)

    def show_error(self, message: str):
        QMessageBox.critical(self, "WaSaB 실행 실패", message)

    def _toggle_all_components(self, checked: bool):
        if self._syncing_select_all:
            return
        self._syncing_select_all = True
        for checkbox in self.component_checks.values():
            checkbox.setChecked(checked)
        self._syncing_select_all = False

    def _sync_select_all(self, *_):
        if self._syncing_select_all:
            return
        self._syncing_select_all = True
        self.select_all_check.setChecked(
            bool(self.component_checks)
            and all(checkbox.isChecked() for checkbox in self.component_checks.values())
        )
        self._syncing_select_all = False

    def set_starting(self, starting: bool):
        self.is_starting = starting
        self.start_button.setEnabled(not starting)
        for checkbox in self.component_checks.values():
            checkbox.setEnabled(not starting)
        self.select_all_check.setEnabled(not starting)
        self.ai_ip_input.setEnabled(not starting)
        self.console_domain_input.setEnabled(not starting)
        for field in (*self.arm_ip_inputs.values(), *self.arm_domain_inputs.values(),
                      *self.pinky_ip_inputs.values(), *self.pinky_domain_inputs.values()):
            field.setEnabled(not starting)

    @staticmethod
    def port_open(port: int, host: str = "127.0.0.1") -> bool:
        try:
            with socket.create_connection((host, port), timeout=0.4):
                return True
        except OSError:
            return False

    def wait_port(self, port: int, timeout: float, host: str = "127.0.0.1") -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.port_open(port, host):
                return True
            time.sleep(0.25)
        return False

    def bootstrap_credentials(self) -> dict[str, str] | None:
        if WEB_DATA.exists():
            return {}
        account, accepted = QInputDialog.getText(self, "최초 관리자 설정", "관리자 아이디를 입력하세요.")
        if not accepted or not account.strip():
            return None
        password, accepted = QInputDialog.getText(
            self, "최초 관리자 설정", "관리자 비밀번호를 입력하세요. (8자 이상)",
            QLineEdit.EchoMode.Password,
        )
        if not accepted or len(password) < 8:
            QMessageBox.warning(self, "설정 실패", "비밀번호는 8자 이상이어야 합니다.")
            return None
        return {"WASAB_WEBAPP_ADMIN": account.strip(), "WASAB_WEBAPP_ADMIN_PW": password}

    def open_log(self, name: str):
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        handle = (LOG_ROOT / f"{name}.log").open("ab", buffering=0)
        self.log_files.append(handle)
        return handle

    def spawn(self, name: str, command: list[str], cwd: Path, env=None):
        process = subprocess.Popen(
            command, cwd=cwd, env=env, stdout=self.open_log(name),
            stderr=subprocess.STDOUT, start_new_session=True,
        )
        self.processes[name] = process
        return process

    def start_system(self):
        if self.is_starting:
            return
        selected = {key: checkbox.isChecked() for key, checkbox in self.component_checks.items()}
        ai_host = self.ai_ip_input.text().strip()
        try:
            ipaddress.ip_address(ai_host)
            console_domain = self.valid_domain(self.console_domain_input.text())
            self.arm_domains = {}
            for arm_id in ARM_CLIENTS:
                host = self.arm_ip_inputs[arm_id].text().strip()
                ipaddress.ip_address(host)
                ARM_CLIENTS[arm_id] = host
                self.arm_domains[arm_id] = self.valid_domain(
                    self.arm_domain_inputs[arm_id].text()
                )
            for key, spec in PINKY_ROBOTS.items():
                host = self.pinky_ip_inputs[key].text().strip()
                ipaddress.ip_address(host)
                spec["host"] = host
                spec["domain"] = self.valid_domain(self.pinky_domain_inputs[key].text())
        except ValueError:
            QMessageBox.warning(
                self, "IP / ROS Domain", "IP 주소와 ROS Domain(0~232)을 올바르게 입력하세요."
            )
            return
        if not any(selected.values()):
            QMessageBox.information(self, "실행 항목 선택", "실행할 항목을 하나 이상 선택하세요.")
            return
        credentials = {}
        if selected["web"] and not self.port_open(8100):
            credentials = self.bootstrap_credentials()
        if credentials is None:
            self.append_log("관리자 설정이 취소되어 실행하지 않았습니다.")
            return
        for key, enabled in selected.items():
            if not enabled:
                self.set_status(key, "선택 안 함")
        self.set_starting(True)
        threading.Thread(
            target=self._start_worker,
            args=(credentials, selected, ai_host, console_domain), daemon=True,
        ).start()

    @staticmethod
    def valid_domain(value: str) -> int:
        domain = int(value.strip())
        if not 0 <= domain <= 232:
            raise ValueError("ROS Domain out of range")
        return domain

    def _start_worker(
        self, credentials: dict[str, str], selected: dict[str, bool], ai_host: str,
        console_domain: int,
    ):
        try:
            if selected["dds"]:
                self.bridge.status.emit("dds", "설정 중")
                os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
                link = subprocess.run(
                    ["ip", "link", "show", "ap0"], stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL, check=False,
                )
                if link.returncode == 0:
                    result = subprocess.run(
                        ["pkexec", "ip", "link", "set", "ap0", "down"], check=False,
                    )
                    if result.returncode != 0:
                        raise RuntimeError("ap0 비활성화 권한이 승인되지 않았습니다.")
                self.bridge.status.emit("dds", "준비 완료")
                self.bridge.log.emit("CycloneDDS 환경과 ap0 네트워크 준비를 완료했습니다.")

            if selected["ai"]:
                if self.port_open(8000, ai_host):
                    self.bridge.status.emit("ai", "실행 중 (기존)")
                    self.bridge.log.emit("AI 서버가 이미 실행 중입니다.")
                else:
                    self.bridge.status.emit("ai", "시작 중")
                    self.spawn("ai-server", [str(SERVER_PYTHON), "-u", "run_server.py"], AI_ROOT)
                    if not self.wait_port(8000, 40, ai_host):
                        raise RuntimeError("AI 서버 8000 포트가 열리지 않았습니다.")
                    self.bridge.status.emit("ai", "실행 중")
                    self.bridge.log.emit("AI/로봇팔 서버가 준비되었습니다.")

            for arm_id, host in ARM_CLIENTS.items():
                if not selected[arm_id]:
                    continue
                if self.remote_client_running(host):
                    self.bridge.status.emit(arm_id, "실행 중 (기존)")
                    self.bridge.log.emit(f"{arm_id} 팔 클라이언트가 이미 실행 중입니다.")
                    continue
                self.bridge.status.emit(arm_id, "연결 중")
                command = self.ssh_command(host) + [
                    f"export ROS_DOMAIN_ID={self.arm_domains[arm_id]}; "
                    f"cd {ARM_WORKDIR} && exec {ARM_PYTHON} -u run_client.py"
                ]
                process = self.spawn(f"arm-{arm_id}", command, PROJECT_ROOT)
                time.sleep(1)
                if process.poll() is None:
                    self.bridge.status.emit(arm_id, "실행 중")
                    self.bridge.log.emit(f"{arm_id} 팔 클라이언트를 시작했습니다.")
                else:
                    self.bridge.status.emit(arm_id, "연결 실패")
                    self.bridge.log.emit(f"{arm_id} 팔 연결에 실패했습니다. 로그를 확인하세요.")

            console_ip = self.local_console_ip()
            for key, spec in PINKY_ROBOTS.items():
                if not selected[key]:
                    continue
                self.bridge.status.emit(key, "연결 중")
                if self.remote_pinky_running(spec):
                    self.bridge.status.emit(key, "실행 중 (기존)")
                    self.bridge.log.emit(f"Pinky-{spec['id']} ROS2 스택이 이미 실행 중입니다.")
                    continue
                try:
                    self.start_pinky_stack(spec, console_ip, console_domain)
                    self.started_pinkies.add(key)
                    self.bridge.status.emit(key, "실행 중")
                    self.bridge.log.emit(
                        f"Pinky-{spec['id']} bringup · localization · navigation · agent를 시작했습니다."
                    )
                except (OSError, subprocess.TimeoutExpired, subprocess.CalledProcessError) as exc:
                    self.bridge.status.emit(key, "실행 실패")
                    self.bridge.log.emit(f"Pinky-{spec['id']} 시작 실패: {exc}")

            if selected["web"]:
                if self.port_open(8100):
                    self.bridge.status.emit("web", "실행 중 (기존)")
                    self.bridge.log.emit("통합 GUI가 이미 실행 중입니다.")
                else:
                    self.bridge.status.emit("web", "시작 중")
                    env = os.environ.copy()
                    env.update(credentials)
                    env["WASAB_ARM_API_URL"] = f"http://{ai_host}:8000"
                    self.spawn(
                        "web-service",
                        [str(WEB_SCRIPT), "--domain", str(console_domain), "--host", "127.0.0.1", "--port", "8100"],
                        PROJECT_ROOT, env=env,
                    )
                    if not self.wait_port(8100, 20):
                        raise RuntimeError("통합 GUI 8100 포트가 열리지 않았습니다.")
                    self.bridge.status.emit("web", "실행 중")
                    self.bridge.log.emit("통합 GUI가 준비되었습니다.")
                webbrowser.open(GUI_URL)
                self.bridge.log.emit("브라우저에서 통합 GUI를 열었습니다.")

            if selected["console"]:
                self.start_console(console_domain)
        except Exception as exc:
            self.bridge.log.emit(f"실행 실패: {exc}")
            self.bridge.error.emit(str(exc))
        finally:
            self.bridge.starting.emit(False)

    @staticmethod
    def ssh_command(host: str) -> list[str]:
        return [
            "sshpass", "-p", ARM_PASSWORD, "ssh", "-o", "StrictHostKeyChecking=no",
            "-o", "ConnectTimeout=5", f"{ARM_USER}@{host}",
        ]

    def remote_client_running(self, host: str) -> bool:
        try:
            result = subprocess.run(
                self.ssh_command(host) + ["pgrep -f '[r]un_client.py'"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=7, check=False,
            )
            return result.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False

    @staticmethod
    def local_console_ip() -> str:
        try:
            output = subprocess.check_output(
                ["ip", "-4", "-brief", "address"], text=True, timeout=3,
            )
            for token in output.replace("/", " ").split():
                if token.startswith("192.168.2."):
                    return token
        except (OSError, subprocess.SubprocessError):
            pass
        return "192.168.2.5"

    @staticmethod
    def pinky_ssh_command(host: str) -> list[str]:
        return [
            "sshpass", "-p", PINKY_PASSWORD, "ssh", "-o", "StrictHostKeyChecking=no",
            "-o", "ConnectTimeout=5", f"{PINKY_USER}@{host}",
        ]

    def remote_pinky_running(self, spec: dict) -> bool:
        try:
            result = subprocess.run(
                self.pinky_ssh_command(spec["host"]) + [
                    "pgrep -f '[b]ringup_robot.launch.xml|[l]ocalization_launch.xml|[n]avigation_launch.xml|[w]asab_robot_agent.agent_node'"
                ],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=7, check=False,
            )
            return result.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False

    def start_pinky_stack(self, spec: dict, console_ip: str, console_domain: int):
        robot_id, domain = spec["id"], spec["domain"]
        setup = (
            "source /opt/ros/jazzy/setup.bash; "
            "source ~/pinky_pro/install/setup.bash; source ~/wasab/install/setup.bash; "
            f"export ROS_DOMAIN_ID={domain} ROS_STATIC_PEERS={console_ip} "
            f"WASAB_CONSOLE_DOMAIN={console_domain} WASAB_ROBOT_ID={robot_id}; "
            "mkdir -p ~/wasab/logs; "
        )
        commands = [
            "nohup ros2 launch pinky_bringup bringup_robot.launch.xml >~/wasab/logs/bringup.log 2>&1 </dev/null &",
            "nohup ros2 launch pinky_navigation localization_launch.xml map:=/home/pinky/wasab/Device/WasabBot/WasabMoveController/wasab_navigation/map/wasab_map11.yaml params_file:=/home/pinky/wasab/Device/WasabBot/WasabMoveController/wasab_navigation/wasab_nav2/params/nav2_params_0709.yaml use_composition:=False >~/wasab/logs/localization.log 2>&1 </dev/null &",
            "nohup ros2 launch pinky_navigation navigation_launch.xml params_file:=/home/pinky/wasab/Device/WasabBot/WasabMoveController/wasab_navigation/wasab_nav2/params/nav2_params_0709.yaml >~/wasab/logs/navigation.log 2>&1 </dev/null &",
            "cd ~/wasab; ./Service/WasabServer/scripts/start_agent.sh --robot-domain " + str(domain)
            + " --console-domain " + str(console_domain),
        ]
        remote = "bash -lc " + shlex.quote(setup + " ".join(commands))
        subprocess.run(
            self.pinky_ssh_command(spec["host"]) + [remote], timeout=25, check=True,
            stdout=self.open_log(f"pinky-{robot_id}"), stderr=subprocess.STDOUT,
        )

    def stop_pinky_stack(self, key: str):
        spec = PINKY_ROBOTS[key]
        remote = (
            "cd ~/wasab; ./Service/WasabServer/scripts/stop_agent.sh || true; "
            "pkill -TERM -f '[b]ringup_robot.launch.xml|[l]ocalization_launch.xml|[n]avigation_launch.xml' || true"
        )
        try:
            subprocess.run(
                self.pinky_ssh_command(spec["host"]) + [remote], timeout=12, check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass

    def send_arm_stop(self):
        ai_host = self.ai_ip_input.text().strip() or DEFAULT_AI_SERVER_IP
        for arm_id in ARM_CLIENTS:
            try:
                request = urllib.request.Request(
                    f"http://{ai_host}:8000/robot-command/stop?arm_id={arm_id}", method="POST"
                )
                urllib.request.urlopen(request, timeout=1.5).read()
            except Exception:
                pass

    def open_robot_gui(self):
        ai_host = self.ai_ip_input.text().strip()
        try:
            ipaddress.ip_address(ai_host)
        except ValueError:
            QMessageBox.warning(self, "AI Server IP", "올바른 AI Server IP 주소를 입력하세요.")
            return
        url = f"http://{ai_host}:8000/camera-view"
        webbrowser.open(url)
        self.append_log(f"브라우저에서 로봇 Admin GUI를 열었습니다: {url}")

    def console_running(self) -> bool:
        result = subprocess.run(
            ["pgrep", "-f", "[w]asab_gui.console_app"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )
        return result.returncode == 0

    def start_console(self, domain: int, peers: str | None = None):
        if self.console_running():
            self.bridge.status.emit("console", "실행 중 (기존)")
            self.bridge.log.emit("ROS Console이 이미 실행 중입니다.")
            return
        if not CONSOLE_SCRIPT.is_file():
            raise RuntimeError(f"Console 실행 스크립트를 찾을 수 없습니다: {CONSOLE_SCRIPT}")
        env = os.environ.copy()
        env["ROS_STATIC_PEERS"] = peers or ";".join(
            spec["host"] for spec in PINKY_ROBOTS.values()
        )
        self.bridge.status.emit("console", "시작 중")
        self.spawn(
            "console", ["bash", str(CONSOLE_SCRIPT), "--domain", str(domain)],
            CONSOLE_ROOT, env=env,
        )
        time.sleep(1)
        process = self.processes.get("console")
        if process is None or process.poll() is not None:
            self.bridge.status.emit("console", "실행 실패")
            raise RuntimeError("ROS Console 실행에 실패했습니다. console.log를 확인하세요.")
        self.bridge.status.emit("console", "실행 중")
        self.bridge.log.emit("ROS Console 창을 열었습니다.")

    def open_console(self):
        try:
            domain = self.valid_domain(self.console_domain_input.text())
            peer_values = [field.text().strip() for field in self.pinky_ip_inputs.values()]
            for host in peer_values:
                ipaddress.ip_address(host)
        except ValueError:
            QMessageBox.warning(self, "IP / ROS Domain", "Pinky IP와 ROS Domain을 확인하세요.")
            return
        threading.Thread(
            target=self._open_console_worker, args=(domain, ";".join(peer_values)), daemon=True,
        ).start()

    def _open_console_worker(self, domain: int, peers: str):
        try:
            self.start_console(domain, peers)
        except Exception as exc:
            self.bridge.log.emit(f"Console 실행 실패: {exc}")
            self.bridge.error.emit(str(exc))

    def stop_remote_client(self, arm_id: str):
        if f"arm-{arm_id}" not in self.processes:
            return
        try:
            subprocess.run(
                self.ssh_command(ARM_CLIENTS[arm_id]) + ["pkill -f '[r]un_client.py'"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=7, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass

    def stop_process(self, name: str):
        process = self.processes.pop(name, None)
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def stop_system(self):
        self.append_log("전체 종료를 시작합니다.")
        self.send_arm_stop()
        for arm_id in ARM_CLIENTS:
            self.stop_remote_client(arm_id)
        for key in tuple(self.started_pinkies):
            self.stop_pinky_stack(key)
            self.started_pinkies.discard(key)
        for name in ("arm-left", "arm-right", "console", "web-service", "ai-server"):
            self.stop_process(name)
        for key in self.status_labels:
            self.set_status(key, "정지")
        self.append_log("실행기가 시작한 프로세스를 종료했습니다.")

    def closeEvent(self, event: QCloseEvent):
        if self.processes or self.started_pinkies:
            answer = QMessageBox.question(
                self, "종료", "실행기가 시작한 서버와 로봇팔 연결도 함께 종료할까요?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.Yes,
            )
            if answer == QMessageBox.StandardButton.Yes:
                self.stop_system()
        event.accept()


def main():
    try:
        app = QApplication(sys.argv)
        app.setApplicationName("WaSaB 통합 실행기")
        window = WasabLauncher()
        window.showMaximized()
        return app.exec()
    except Exception:
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        (LOG_ROOT / "launcher-error.log").write_text(traceback.format_exc(), encoding="utf-8")
        raise


if __name__ == "__main__":
    raise SystemExit(main())
