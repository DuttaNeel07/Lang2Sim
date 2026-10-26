"""
Chat Session Memory & Persistence Manager
File: lang2sim/core/session_manager.py
"""

import json
import time
import uuid
from datetime import datetime, date, timedelta
from pathlib import Path
from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field
from .config import DEFAULT_LANG2SIM_ROOT

SESSIONS_DIR = DEFAULT_LANG2SIM_ROOT / "sessions"
SESSIONS_DIR.mkdir(parents=True, exist_ok=True)


class ChatMessage(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4())[:8])
    role: str  # "user", "assistant", "system"
    content: str
    timestamp: float = Field(default_factory=time.time)
    tokens: int = 0


class ChatSession(BaseModel):
    id: str = Field(
        default_factory=lambda: datetime.now().strftime("session_%Y%m%d_%H%M%S_") + str(uuid.uuid4())[:4]
    )
    title: str = "New Session"
    created_at: float = Field(default_factory=time.time)
    updated_at: float = Field(default_factory=time.time)
    pinned: bool = False
    messages: List[ChatMessage] = Field(default_factory=list)
    total_tokens: int = 0
    workspace_name: str = "active_project"


class SessionManager:
    """Manages creation, persistence, retrieval, and grouping of chat sessions."""

    def __init__(self, sessions_dir: Optional[Path] = None):
        self.sessions_dir = sessions_dir or SESSIONS_DIR
        self.sessions_dir.mkdir(parents=True, exist_ok=True)

    def _file_path(self, session_id: str) -> Path:
        return self.sessions_dir / f"{session_id}.json"

    def create_session(
        self,
        title: str = "New Session",
        workspace_name: str = "active_project",
        system_prompt: Optional[str] = None,
    ) -> ChatSession:
        session = ChatSession(title=title, workspace_name=workspace_name)
        if system_prompt:
            session.messages.append(ChatMessage(role="system", content=system_prompt))
        self.save_session(session)
        return session

    def get_session(self, session_id: str) -> Optional[ChatSession]:
        p = self._file_path(session_id)
        if not p.is_file():
            return None
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            return ChatSession.model_validate(data)
        except Exception:
            return None

    def save_session(self, session: ChatSession) -> None:
        session.updated_at = time.time()
        p = self._file_path(session.id)
        p.write_text(session.model_dump_json(indent=2), encoding="utf-8")

    def delete_session(self, session_id: str) -> bool:
        p = self._file_path(session_id)
        if p.is_file():
            try:
                p.unlink()
                return True
            except OSError:
                return False
        return False

    def pin_session(self, session_id: str, pinned: Optional[bool] = None) -> bool:
        session = self.get_session(session_id)
        if not session:
            return False
        if pinned is None:
            session.pinned = not session.pinned
        else:
            session.pinned = pinned
        self.save_session(session)
        return True

    def rename_session(self, session_id: str, new_title: str) -> bool:
        session = self.get_session(session_id)
        if not session:
            return False
        session.title = new_title.strip() or "Untitled Session"
        self.save_session(session)
        return True

    def list_sessions(self, workspace_name: Optional[str] = None) -> List[ChatSession]:
        """Lists all stored sessions sorted by pinned first, then by updated_at descending."""
        sessions: List[ChatSession] = []
        for p in self.sessions_dir.glob("*.json"):
            try:
                data = json.loads(p.read_text(encoding="utf-8"))
                s = ChatSession.model_validate(data)
                if workspace_name is None or s.workspace_name == workspace_name:
                    sessions.append(s)
            except Exception:
                continue

        # Sort: pinned first (True > False), then updated_at descending
        sessions.sort(key=lambda s: (not s.pinned, -s.updated_at))
        return sessions

    def group_sessions_by_date(
        self, sessions: List[ChatSession]
    ) -> Dict[str, List[ChatSession]]:
        """
        Groups sessions chronologically matching OpenCode:
        - "Today"
        - "Yesterday"
        - "%a %b %d %Y" (e.g. "Sun Oct 25 2026", "Wed Oct 07 2026")
        """
        today = date.today()
        yesterday = today - timedelta(days=1)
        grouped: Dict[str, List[ChatSession]] = {}

        for s in sessions:
            s_date = datetime.fromtimestamp(s.updated_at).date()
            if s.pinned:
                header = "Pinned"
            elif s_date == today:
                header = "Today"
            elif s_date == yesterday:
                header = "Yesterday"
            else:
                header = datetime.fromtimestamp(s.updated_at).strftime("%a %b %d %Y")

            if header not in grouped:
                grouped[header] = []
            grouped[header].append(s)

        return grouped


# Global instance
session_manager = SessionManager()
