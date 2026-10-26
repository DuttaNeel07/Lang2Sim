from pathlib import Path
import sys

# Ensure repository root is on sys.path so tests run zero-config without PYTHONPATH=.
_repo_root = Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

import lang2sim
import lang2sim.core
import lang2sim.core.models
import lang2sim.core.config

sys.modules["sim2lang.core"] = sys.modules["lang2sim.core"]
sys.modules["sim2lang.core.models"] = sys.modules["lang2sim.core.models"]
sys.modules["sim2lang.core.config"] = sys.modules["lang2sim.core.config"]
sys.modules["sim2lang"] = sys.modules["lang2sim"]
