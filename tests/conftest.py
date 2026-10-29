import sys
from pathlib import Path

# Ensure repository root is on sys.path so tests run zero-config without PYTHONPATH=.
_repo_root = Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

import lang2sim  # noqa: E402, F401
import lang2sim.core  # noqa: E402, F401
import lang2sim.core.config  # noqa: E402, F401
import lang2sim.core.models  # noqa: E402, F401

sys.modules["sim2lang.core"] = sys.modules["lang2sim.core"]
sys.modules["sim2lang.core.models"] = sys.modules["lang2sim.core.models"]
sys.modules["sim2lang.core.config"] = sys.modules["lang2sim.core.config"]
sys.modules["sim2lang"] = sys.modules["lang2sim"]
