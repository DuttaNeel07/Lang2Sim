"""
Async control API client. Runs on the HOST (TUI / agent tools / doctor).
File: sim2lang/runtime/control_api/client.py

    async with ControlClient() as api:
        await api.wait_until_ready(timeout=60)
        await api.sim_start(SimStartRequest(...))
        print(await api.sim_status())

Uses httpx; pass `transport=httpx.ASGITransport(app)` in tests to talk to the
server in-process.
"""

from __future__ import annotations

import asyncio
import importlib
import time
from typing import Any, Dict, List, Optional

import httpx

from lang2sim.core.models import SimStartRequest, SimStatusResponse, SimStopRequest
from lang2sim.runtime.control_api import API_PREFIX

# NB: `from lang2sim.core import config` yields the Config *instance* (core/__init__
# re-exports it under the module's name), so load the module explicitly.
cfg = importlib.import_module("lang2sim.core.config")


class ControlAPIError(RuntimeError):
    def __init__(self, message: str, status_code: Optional[int] = None):
        super().__init__(message)
        self.status_code = status_code


class ControlAPIUnavailable(ControlAPIError):
    """Container not up / API not listening yet."""


def default_base_url() -> str:
    return f"http://127.0.0.1:{cfg.DEFAULT_CONTROL_PORT}"


class ControlClient:
    def __init__(self, base_url: Optional[str] = None, timeout: float = 10.0,
                 transport: Optional[httpx.AsyncBaseTransport] = None):
        self.base_url = (base_url or default_base_url()).rstrip("/")
        self._http = httpx.AsyncClient(base_url=self.base_url + API_PREFIX,
                                       timeout=timeout, transport=transport)

    async def __aenter__(self) -> "ControlClient":
        return self

    async def __aexit__(self, *exc: Any) -> None:
        await self.aclose()

    async def aclose(self) -> None:
        await self._http.aclose()

    async def _request(self, method: str, path: str, **kw: Any) -> Any:
        try:
            r = await self._http.request(method, path, **kw)
        except (httpx.ConnectError, httpx.ReadError, httpx.RemoteProtocolError,
                httpx.ConnectTimeout) as e:
            raise ControlAPIUnavailable(f"control API not reachable at {self.base_url}: {e}") from e
        if r.status_code >= 400:
            try:
                detail = r.json().get("detail", r.text)
            except ValueError:
                detail = r.text
            raise ControlAPIError(f"{method} {path} -> {r.status_code}: {detail}", r.status_code)
        return r.json()

    # --- endpoints -------------------------------------------------------
    async def health(self) -> Dict[str, Any]:
        return await self._request("GET", "/health")

    async def info(self) -> Dict[str, Any]:
        return await self._request("GET", "/info")

    async def sim_status(self) -> SimStatusResponse:
        return SimStatusResponse.model_validate(await self._request("GET", "/sim/status"))

    async def sim_start(self, req: SimStartRequest) -> SimStatusResponse:
        data = await self._request("POST", "/sim/start", json=req.model_dump(mode="json"))
        return SimStatusResponse.model_validate(data)

    async def sim_stop(self, force: bool = False) -> SimStatusResponse:
        data = await self._request("POST", "/sim/stop",
                                   json=SimStopRequest(force=force).model_dump(mode="json"))
        return SimStatusResponse.model_validate(data)

    async def sim_logs(self, tail: int = 200) -> List[str]:
        return (await self._request("GET", "/sim/logs", params={"tail": tail}))["lines"]

    # --- helpers ---------------------------------------------------------
    async def wait_until_ready(self, timeout: float = 60.0, interval: float = 0.5) -> Dict[str, Any]:
        """Poll /health until it answers (container boot + uvicorn start)."""
        deadline = time.monotonic() + timeout
        last: Optional[Exception] = None
        while time.monotonic() < deadline:
            try:
                return await self.health()
            except ControlAPIUnavailable as e:
                last = e
                await asyncio.sleep(interval)
        raise ControlAPIUnavailable(f"control API not ready after {timeout:.0f}s ({last})")
