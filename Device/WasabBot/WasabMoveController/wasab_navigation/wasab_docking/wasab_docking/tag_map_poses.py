"""태그 map pose config(tag_id -> (x,y,yaw) map frame) 로드/저장. 순수(rclpy 무관)."""
import yaml


def load(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def save(cfg, path):
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=True, allow_unicode=True)


def get(cfg, tag_id):
    tags = cfg.get("tags") or {}
    t = tags.get(int(tag_id))
    if t is None:
        t = tags.get(str(tag_id))     # 손편집 yaml 문자열 키 방어(registered_ids와 정합)
    if t is None:
        return None
    return (float(t["x"]), float(t["y"]), float(t["yaw"]))


def upsert(cfg, tag_id, pose):
    cfg.setdefault("tags", {})[int(tag_id)] = {
        "x": float(pose[0]), "y": float(pose[1]), "yaw": float(pose[2])}
    return cfg


def registered_ids(cfg):
    return {int(k) for k in (cfg.get("tags") or {}).keys()}
