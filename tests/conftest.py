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


@pytest.fixture
def cfg(tmp_path):
    over = {"planner": {"problem_file": str(tmp_path / "problem.pddl")}}
    if ff_path():
        over["planner"]["command"] = [ff_path()]
    return load_config(overrides=over)


needs_ff = pytest.mark.skipif(ff_path() is None, reason="FF binary not found (set FF_PATH)")
