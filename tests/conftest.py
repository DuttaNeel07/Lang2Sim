"""Pytest root configuration ensuring lang2sim and sim2lang alias are loaded."""

import sys
import lang2sim
import lang2sim.core
import lang2sim.core.models
import lang2sim.core.config

sys.modules["sim2lang.core"] = sys.modules["lang2sim.core"]
sys.modules["sim2lang.core.models"] = sys.modules["lang2sim.core.models"]
sys.modules["sim2lang.core.config"] = sys.modules["lang2sim.core.config"]
sys.modules["sim2lang"] = sys.modules["lang2sim"]
