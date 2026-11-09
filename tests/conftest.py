import importlib.util
import os
import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from braccio_ns.config import load_config  # noqa: E402


def ff_path():
    for p in (os.environ.get("FF_PATH"), shutil.which("ff"), str(Path(__file__).parent.parent / "ff")):
        if p and Path(p).is_file() and os.access(p, os.X_OK):
            return p
    return None


def has_fast_downward():
    return importlib.util.find_spec("up_fast_downward") is not None


@pytest.fixture
def cfg(tmp_path):
    over = {"planner": {"problem_file": str(tmp_path / "problem.pddl")}}
    if os.environ.get("FF_PATH") and ff_path():  # opt in to the thesis planner
        over["planner"].update(backend="ff", command=[ff_path()])
    return load_config(overrides=over)


needs_planner = pytest.mark.skipif(
    not has_fast_downward() and ff_path() is None, reason="no planner (pip install up-fast-downward, or set FF_PATH)"
)
