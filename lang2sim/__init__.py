"""Lang2Sim: Natural Language to Executable Robot Simulation."""

from __future__ import annotations

import sys

__version__ = "2.0.0"

# Backward-compatibility alias: allow legacy imports of sim2lang to resolve to lang2sim
if "sim2lang" not in sys.modules:
    sys.modules["sim2lang"] = sys.modules[__name__]
