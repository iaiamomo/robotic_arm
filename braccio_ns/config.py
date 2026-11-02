"""Configuration loading. Relative paths in the config are resolved from the repo root."""
import copy
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "config" / "default.json"


def _merge(base, override):
    out = copy.deepcopy(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path=None, overrides=None):
    cfg = json.loads(DEFAULT_CONFIG.read_text())
    if path:
        cfg = _merge(cfg, json.loads(Path(path).read_text()))
    if overrides:
        cfg = _merge(cfg, overrides)
    return cfg


def resolve(path):
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p
