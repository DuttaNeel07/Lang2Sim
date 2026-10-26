"""Lang2Sim: Natural Language to Executable Robot Simulation."""

from __future__ import annotations

import importlib
import sys
from importlib.abc import MetaPathFinder
from typing import Any

__version__ = "2.0.0"


class _Sim2LangRedirectFinder(MetaPathFinder):
    """Transparently redirects legacy 'sim2lang.*' imports to 'lang2sim.*'."""

    def find_spec(self, fullname: str, path: Any, target: Any = None) -> Any:
        if fullname == "sim2lang" or fullname.startswith("sim2lang."):
            target_name = "lang2sim" + fullname[len("sim2lang"):]
            try:
                target_mod = importlib.import_module(target_name)
                sys.modules[fullname] = target_mod
                return getattr(target_mod, "__spec__", None)
            except ModuleNotFoundError:
                return None
        return None


if not any(isinstance(f, _Sim2LangRedirectFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _Sim2LangRedirectFinder())

if "sim2lang" not in sys.modules:
    sys.modules["sim2lang"] = sys.modules[__name__]
