"""Configuration loading. Relative paths in the config are resolved from the repo root."""

import copy
import json
import os
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


def load_dotenv(path=None):
    """Load KEY=VALUE lines from <repo>/.env into os.environ (existing variables win).

    Minimal parser: blank lines and # comments are skipped, an optional `export ` prefix
    and surrounding quotes are stripped. Returns the names that were set.
    """
    p = Path(path) if path else REPO_ROOT / ".env"
    if not p.is_file():
        return []
    loaded = []
    for raw in p.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, val = line.removeprefix("export ").split("=", 1)
        key, val = key.strip(), val.strip()
        if len(val) >= 2 and val[0] == val[-1] and val[0] in "\"'":
            val = val[1:-1]
        if key and key not in os.environ:
            os.environ[key] = val
            loaded.append(key)
    return loaded
