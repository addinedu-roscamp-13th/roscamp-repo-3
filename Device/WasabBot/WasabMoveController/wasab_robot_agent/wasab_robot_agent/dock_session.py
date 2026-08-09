"""dock 세션 순수 상태(rclpy 무관, pytest). started 후 done/failed 정확히 1회 종결."""


def parse_dock_cmd(data):
    """'start:<N>' | 'stop' → ('start', int) | ('stop', None) | None."""
    if data == "stop":
        return ("stop", None)
    if data.startswith("start:"):
        try:
            return ("start", int(data[len("start:"):]))
        except ValueError:
            return None
    return None


class DockSession:
    def __init__(self):
        self._active = False
        self._tag_id = None

    @property
    def active(self):
        return self._active

    @property
    def tag_id(self):
        return self._tag_id

    def start(self, tag_id):
        self._active = True
        self._tag_id = int(tag_id)

    def _terminate(self, event, reason):
        if not self._active:
            return None
        tag = self._tag_id
        self._active = False
        self._tag_id = None
        return {"event": event, "reason": reason, "tag_id": tag}

    def on_docking_state(self, state):
        if not self._active:
            return None
        s = state.get("state")
        if s == "DONE":
            return self._terminate("done", None)
        if s == "FAILED":
            return self._terminate("failed", state.get("fail_reason") or "failed")
        return None

    def on_process_exited(self):
        return self._terminate("failed", "process_exited")

    def on_timeout(self):
        return self._terminate("failed", "timeout")

    def on_stop(self):
        return self._terminate("failed", "cancelled")
