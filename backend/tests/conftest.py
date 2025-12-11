"""Pytest configuration and fixtures for RAG chatbot tests."""

import os
import pytest
from typing import Generator
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

# Set test environment variables
os.environ["ENVIRONMENT"] = "test"
os.environ["OPENAI_API_KEY"] = "sk-test-key"
os.environ["QDRANT_URL"] = "http://localhost:6333"
os.environ["QDRANT_API_KEY"] = "test-key"
os.environ["DATABASE_URL"] = "sqlite:///:memory:"


@pytest.fixture(scope="session")
def test_db_engine():
    """Create in-memory SQLite database for testing."""
    engine = create_engine(
        "sqlite:///:memory:",
        connect_args={"check_same_thread": False},
        poolclass=StaticPool,
    )
    return engine


@pytest.fixture(scope="function")
def test_db_session(test_db_engine):
    """Create a new database session for each test."""
    TestingSessionLocal = sessionmaker(
        autocommit=False,
        autoflush=False,
        bind=test_db_engine,
    )
    session = TestingSessionLocal()
    try:
        yield session
    finally:
        session.close()


@pytest.fixture(scope="module")
def test_client() -> Generator:
    """Create a TestClient for API testing."""
    # Import here to avoid circular dependencies
    from src.main import app

    with TestClient(app) as client:
        yield client


@pytest.fixture
def mock_openai_embedding():
    """Mock OpenAI embedding response."""
    return [0.1] * 1536  # 1536-dimensional mock vector


@pytest.fixture
def mock_qdrant_search_results():
    """Mock Qdrant search results."""
    return [
        {
            "id": "chunk-001",
            "score": 0.85,
            "payload": {
                "chunk_id": "chunk-001",
                "file_path": "/docs/chapter1.md",
                "section_heading": "Introduction to Robotics",
                "content_text": "Robotics combines mechanical engineering, electrical engineering, and computer science.",
                "chunk_index": 0,
                "created_at": "2025-12-09T00:00:00Z",
                "content_hash": "abc123",
            },
        },
        {
            "id": "chunk-002",
            "score": 0.78,
            "payload": {
                "chunk_id": "chunk-002",
                "file_path": "/docs/chapter2.md",
                "section_heading": "Robot Sensors",
                "content_text": "Sensors allow robots to perceive their environment.",
                "chunk_index": 1,
                "created_at": "2025-12-09T00:00:00Z",
                "content_hash": "def456",
            },
        },
    ]


@pytest.fixture
def mock_agent_response():
    """Mock OpenAI Agent response."""
    return {
        "answer": "Robotics is an interdisciplinary field combining mechanical engineering, electrical engineering, and computer science.",
        "sources": [
            {
                "file": "/docs/chapter1.md",
                "section": "Introduction to Robotics",
                "chunk": "Robotics combines mechanical engineering, electrical engineering, and computer science.",
                "similarity": 0.85,
            }
        ],
    }


@pytest.fixture
def sample_question():
    """Sample test question."""
    return "What is robotics?"


@pytest.fixture
def sample_session_id():
    """Sample session ID."""
    return "test-session-123"
