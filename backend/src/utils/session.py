"""Session management for multi-turn conversations.

Implements in-memory session storage with LRU eviction.
"""

from typing import Dict, List, Optional
from datetime import datetime, timedelta
from collections import OrderedDict
import threading


class SessionManager:
    """Thread-safe session manager for conversation history."""

    def __init__(
        self,
        max_turns: int = 10,
        session_timeout_minutes: int = 30,
        max_sessions: int = 1000,
    ):
        """Initialize session manager.

        Args:
            max_turns: Maximum conversation turns to store per session
            session_timeout_minutes: Session idle timeout in minutes
            max_sessions: Maximum number of sessions to keep in memory
        """
        self.max_turns = max_turns
        self.session_timeout = timedelta(minutes=session_timeout_minutes)
        self.max_sessions = max_sessions

        # Session storage: {session_id: {"history": [...], "last_accessed": datetime}}
        self.sessions: OrderedDict[str, Dict] = OrderedDict()
        self.lock = threading.Lock()

    def get_history(self, session_id: str) -> List[Dict]:
        """Get conversation history for a session.

        Args:
            session_id: Session identifier

        Returns:
            List of conversation messages (empty if session not found or expired)
        """
        with self.lock:
            self._cleanup_expired()

            if session_id not in self.sessions:
                return []

            session = self.sessions[session_id]

            # Update last accessed time
            session["last_accessed"] = datetime.utcnow()

            # Move to end (most recently used)
            self.sessions.move_to_end(session_id)

            return session["history"].copy()

    def add_message(
        self, session_id: str, role: str, content: str, sources: Optional[List] = None
    ):
        """Add a message to conversation history.

        Args:
            session_id: Session identifier
            role: Message role ("user" or "assistant")
            content: Message content
            sources: Optional sources for assistant messages
        """
        with self.lock:
            self._cleanup_expired()

            # Create session if doesn't exist
            if session_id not in self.sessions:
                self.sessions[session_id] = {
                    "history": [],
                    "last_accessed": datetime.utcnow(),
                }

            session = self.sessions[session_id]

            # Create message
            message = {"role": role, "content": content, "timestamp": datetime.utcnow()}

            if sources:
                message["sources"] = sources

            # Add to history
            session["history"].append(message)

            # Trim to max turns (keep most recent)
            if len(session["history"]) > self.max_turns:
                session["history"] = session["history"][-self.max_turns :]

            # Update last accessed
            session["last_accessed"] = datetime.utcnow()

            # Move to end (LRU)
            self.sessions.move_to_end(session_id)

            # Evict oldest sessions if over limit
            while len(self.sessions) > self.max_sessions:
                self.sessions.popitem(last=False)  # Remove oldest

    def clear_session(self, session_id: str):
        """Clear a specific session.

        Args:
            session_id: Session to clear
        """
        with self.lock:
            if session_id in self.sessions:
                del self.sessions[session_id]

    def _cleanup_expired(self):
        """Remove expired sessions (called internally with lock held)."""
        now = datetime.utcnow()
        expired = [
            sid
            for sid, session in self.sessions.items()
            if now - session["last_accessed"] > self.session_timeout
        ]

        for sid in expired:
            del self.sessions[sid]

    def get_stats(self) -> Dict:
        """Get session statistics.

        Returns:
            Dictionary with session stats
        """
        with self.lock:
            self._cleanup_expired()

            return {
                "active_sessions": len(self.sessions),
                "max_sessions": self.max_sessions,
                "max_turns_per_session": self.max_turns,
                "session_timeout_minutes": self.session_timeout.total_seconds() / 60,
            }


# Global session manager instance
session_manager = SessionManager()
