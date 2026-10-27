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


import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _never_touch_a_real_sim_container(request, monkeypatch):
    """Unit tests must not start, replace or remove the real `lang2sim-sim` container.

    A TUI test that types a scene ("Spawn 14x10 warehouse ...") reaches run_simulation, whose
    DockerRunner.start() replaces any running container: running the suite during a demo killed
    the live simulation. Integration tests (tests/integration, L2S_INTEGRATION=1) keep real Docker.
    """
    if "integration" in str(request.node.fspath):
        return
    import subprocess

    from lang2sim.runtime import docker_runner

    real_call = docker_runner.DockerRunner._call

    def guarded(self, args, check=True, timeout=60):
        if self._run is subprocess.run and args and args[0] in ("run", "rm", "stop", "kill", "pull", "build",
                                                                 "create", "start", "restart"):
            cp = subprocess.CompletedProcess([self.docker, *args], 1, "", "blocked: unit tests don't drive Docker")
            if check:
                raise docker_runner.DockerError(f"`docker {args[0]}` blocked in unit tests", cp.stderr)
            return cp
        return real_call(self, args, check=check, timeout=timeout)

    monkeypatch.setattr(docker_runner.DockerRunner, "_call", guarded)
