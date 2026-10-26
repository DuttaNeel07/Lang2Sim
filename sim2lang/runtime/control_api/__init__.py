"""Control API: FastAPI server inside the sim container, async client on the host.

Versioned under /v1. Import `client` on the host; `server` needs fastapi/uvicorn
(present in the sim image, or `pip install -e .[sim]`).
"""

API_PREFIX = "/v1"
