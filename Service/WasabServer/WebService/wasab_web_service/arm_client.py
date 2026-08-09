"""통합 GUI에서 기존 로봇팔 HTTP 서버를 호출하는 얇은 클라이언트."""
import json
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode
from urllib.request import Request, urlopen


class ArmClientError(RuntimeError):
    def __init__(self, status_code, detail):
        super().__init__(detail)
        self.status_code = status_code
        self.detail = detail


class ArmClient:
    def __init__(self, base_url, timeout=3.0):
        self._base_url = base_url.rstrip("/")
        self._timeout = float(timeout)

    def _request(self, method, path, query=None, binary=False, body=None):
        url = self._base_url + path
        if query:
            url += "?" + urlencode(query)
        payload = None if body is None else json.dumps(body).encode("utf-8")
        headers = {} if payload is None else {"Content-Type": "application/json"}
        request = Request(url, data=payload, headers=headers, method=method)
        try:
            with urlopen(request, timeout=self._timeout) as response:
                body = response.read()
                if binary:
                    return body, response.headers.get_content_type()
                return json.loads(body or b"{}")
        except HTTPError as exc:
            try:
                detail = json.loads(exc.read()).get("detail", str(exc))
            except (ValueError, AttributeError):
                detail = str(exc)
            raise ArmClientError(exc.code, detail) from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise ArmClientError(503, f"로봇팔 서버 연결 실패: {exc}") from exc

    def status(self, arm_id):
        if arm_id == "dual":
            return self._request("GET", "/dual-arm/status")
        return self._request("GET", f"/arm-features/{arm_id}/status")

    def command(self, arm_id, command):
        path = f"/robot-command/{quote(command, safe='')}"
        return self._request("POST", path, {"arm_id": arm_id})

    def toggle_feature(self, arm_id, feature):
        return self._request(
            "POST", f"/arm-features/{arm_id}/{feature}/toggle")

    def fire_prompt(self, arm_id):
        return self._request("GET", f"/arm-features/{arm_id}/fire-prompt")

    def fire_response(self, arm_id, response):
        return self._request(
            "POST", f"/arm-features/{arm_id}/fire-response",
            body={"response": response},
        )

    def face_prompt(self, arm_id):
        return self._request("GET", f"/arm-features/{arm_id}/face-prompt")

    def acknowledge_face_prompt(self, arm_id):
        return self._request("POST", f"/arm-features/{arm_id}/face-prompt/ack")

    def dual_command(self, command):
        path = "/dual-arm/gift-giving" if command == "gift-giving" else f"/dual-arm/{command}"
        return self._request("POST", path)

    def logs(self, after_id=0):
        return self._request("GET", "/robot-logs", {"after_id": int(after_id)})

    def camera(self, arm_id):
        return self._request(
            "GET", "/camera-frame/latest.jpg", {"arm_id": arm_id}, binary=True)
