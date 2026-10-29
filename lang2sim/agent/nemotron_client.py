"""
NVIDIA Nemotron 3 Ultra 550B A55B API Client
File: lang2sim/agent/nemotron_client.py
"""

import os
from pathlib import Path
from typing import Any, AsyncGenerator, Dict, List, Optional

from openai import AsyncOpenAI

DEFAULT_NVIDIA_BASE_URL = "https://integrate.api.nvidia.com/v1"
DEFAULT_NEMOTRON_MODEL = "nvidia/nemotron-3-ultra-550b-a55b"


def _load_env_if_present() -> None:
    """Best-effort loader for .env without external dependencies."""
    candidates = [
        Path.cwd() / ".env",
        Path(__file__).resolve().parent.parent.parent / ".env",
        Path.home() / ".lang2sim" / ".env",
    ]
    for p in candidates:
        if p.is_file():
            try:
                for line in p.read_text().splitlines():
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        k, v = line.split("=", 1)
                        k, v = k.strip(), v.strip().strip("'\"")
                        if k not in os.environ:
                            os.environ[k] = v
            except Exception:
                pass


_load_env_if_present()


class NemotronClient:
    """Client for NVIDIA Nemotron 3 Ultra 550B via OpenAI-compatible endpoint."""

    def __init__(
        self,
        api_key: Optional[str] = None,
        base_url: Optional[str] = None,
        model: Optional[str] = None,
    ):
        _load_env_if_present()
        self.api_key = api_key or os.getenv("NVIDIA_API_KEY") or os.getenv("OPENAI_API_KEY")

        # Auto-detect base_url and model based on key type if not explicitly passed
        if base_url:
            self.base_url = base_url
        elif self.api_key and self.api_key.startswith("sk-"):
            self.base_url = os.getenv("OPENAI_BASE_URL", "https://api.openai.com/v1")
        elif os.getenv("NVIDIA_BASE_URL"):
            self.base_url = os.getenv("NVIDIA_BASE_URL")
        else:
            self.base_url = DEFAULT_NVIDIA_BASE_URL

        if model:
            self.model = model
        elif self.api_key and self.api_key.startswith("sk-"):
            self.model = os.getenv("OPENAI_MODEL", "gpt-4o-mini")
        elif os.getenv("NEMOTRON_MODEL"):
            self.model = os.getenv("NEMOTRON_MODEL")
        else:
            self.model = DEFAULT_NEMOTRON_MODEL

        self._client: Optional[AsyncOpenAI] = None

    @property
    def client(self) -> AsyncOpenAI:
        if self._client is None:
            if not self.api_key:
                raise ValueError(
                    "NVIDIA_API_KEY is not set. Please set the environment variable or configure .env."
                )
            self._client = AsyncOpenAI(api_key=self.api_key, base_url=self.base_url)
        return self._client

    def is_configured(self) -> bool:
        return bool(self.api_key)

    async def stream_chat(
        self,
        messages: List[Dict[str, Any]],
        tools: Optional[List[Dict[str, Any]]] = None,
        temperature: float = 0.2,
    ) -> AsyncGenerator[str, None]:
        """Streams response tokens from Nemotron 3 Ultra 550B with low latency."""
        kwargs: Dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature,
            "stream": True,
        }
        if tools:
            kwargs["tools"] = tools

        try:
            response = await self.client.chat.completions.create(**kwargs)
            async for chunk in response:
                delta = chunk.choices[0].delta if chunk.choices else None
                if delta and delta.content:
                    yield delta.content
        except Exception as e:
            yield f"\n\n[API Error: {str(e)}]"
