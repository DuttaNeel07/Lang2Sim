"""Part 3: container runtime, display/GPU selection, control API, Nav2 launch.

Host side:      platform, hardware_probe, display, docker_runner, doctor, control_api.client
Container side: control_api.server (FastAPI on :9090, needs the `sim` extra)

Keep this module import-light: the TUI imports it on startup.
"""
