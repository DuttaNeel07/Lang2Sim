"""
NVIDIA Nemotron 3 Ultra 550B A55B API Client
File: sim2lang/agent/nemotron_client.py
"""

import os
from typing import List, Dict, Any, AsyncGenerator, Optional
from openai import AsyncOpenAI

DEFAULT_NVIDIA_BASE_URL = "https://integrate.api.nvidia.com/v1"
DEFAULT_NEMOTRON_MODEL = "nvidia/nemotron-3-ultra-550b"

class NemotronClient:
    """Client for NVIDIA Nemotron 3 Ultra 550B via OpenAI-compatible endpoint."""

    def __init__(self, api_key: Optional[str] = None, base_url: Optional[str] = None, model: Optional[str] = None):
        self.api_key = api_key or os.getenv("NVIDIA_API_KEY") or os.getenv("OPENAI_API_KEY")
        self.base_url = base_url or os.getenv("NVIDIA_BASE_URL", DEFAULT_NVIDIA_BASE_URL)
        self.model = model or DEFAULT_NEMOTRON_MODEL
        self._client: Optional[AsyncOpenAI] = None

    @property
    def client(self) -> AsyncOpenAI:
        if self._client is None:
            if not self.api_key:
                raise ValueError("NVIDIA_API_KEY is not set. Please set the environment variable or pass api_key.")
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
        """Streams response tokens from Nemotron 3 Ultra 550B."""
        kwargs: Dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature,
            "stream": True,
        }
        if tools:
            kwargs["tools"] = tools

        response = await self.client.chat.completions.create(**kwargs)
        async for chunk in response:
            delta = chunk.choices[0].delta if chunk.choices else None
            if delta and delta.content:
                yield delta.content
