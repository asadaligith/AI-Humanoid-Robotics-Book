"""API endpoint tests for RAG chatbot.

Tests for /ask and /ask-selected endpoints with various scenarios.
"""

import pytest
from unittest.mock import patch, MagicMock
from fastapi.testclient import TestClient


def test_ask_endpoint_basic(test_client):
    """Test basic /ask endpoint functionality."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        # Mock responses
        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = [
            {
                "id": "chunk-001",
                "score": 0.85,
                "payload": {
                    "content_text": "ROS 2 is a robot operating system.",
                    "section_heading": "Introduction to ROS 2",
                    "file_path": "/docs/chapter1.md"
                }
            }
        ]
        mock_agent.return_value = {
            "answer": "ROS 2 is a robot operating system used for robotics development.",
            "sources": [
                {
                    "file": "/docs/chapter1.md",
                    "section": "Introduction to ROS 2",
                    "chunk": "ROS 2 is a robot operating system."
                }
            ]
        }

        # Make request
        response = test_client.post(
            "/api/ask",
            json={"question": "What is ROS 2?"}
        )

        # Assertions
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert "session_id" in data
        assert len(data["answer"]) > 0


def test_ask_selected_endpoint_basic(test_client):
    """Test basic /ask-selected endpoint functionality."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        # Mock responses
        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = [
            {
                "id": "chunk-001",
                "score": 0.88,
                "payload": {
                    "content_text": "Inverse kinematics calculates joint parameters for desired end-effector positions.",
                    "section_heading": "Robot Kinematics",
                    "file_path": "/docs/chapter3.md"
                }
            }
        ]
        mock_agent.return_value = {
            "answer": "Inverse kinematics is the mathematical process of determining the joint angles needed to position a robot's end-effector at a specific location.",
            "sources": [
                {
                    "file": "/docs/chapter3.md",
                    "section": "Robot Kinematics",
                    "chunk": "Inverse kinematics calculates joint parameters for desired end-effector positions."
                }
            ]
        }

        # Make request with selected text
        response = test_client.post(
            "/api/ask-selected",
            json={
                "question": "Explain this in simpler terms",
                "selected_text": "Inverse kinematics is the process of determining the joint parameters that provide a desired position of the robot's end-effector."
            }
        )

        # Assertions
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert "session_id" in data
        assert len(data["answer"]) > 0

        # Verify hybrid scoring was used (search called with selected_text_embedding)
        mock_search.assert_called_once()
        call_kwargs = mock_search.call_args[1]
        assert "selected_text_embedding" in call_kwargs
        assert call_kwargs["selected_text_embedding"] is not None


def test_ask_selected_endpoint_validation(test_client):
    """Test /ask-selected endpoint input validation."""
    # Test: selected_text too short (< 10 chars)
    response = test_client.post(
        "/api/ask-selected",
        json={
            "question": "What is this?",
            "selected_text": "ROS 2"  # Only 5 chars
        }
    )
    assert response.status_code == 422  # Validation error

    # Test: selected_text too long (> 5000 chars)
    response = test_client.post(
        "/api/ask-selected",
        json={
            "question": "Summarize this",
            "selected_text": "A" * 5001  # Exceeds limit
        }
    )
    assert response.status_code == 422  # Validation error

    # Test: question too short (< 5 chars)
    response = test_client.post(
        "/api/ask-selected",
        json={
            "question": "Why",  # Only 3 chars
            "selected_text": "This is a valid selected text that is long enough."
        }
    )
    assert response.status_code == 422  # Validation error


def test_ask_selected_hybrid_scoring(test_client):
    """Test that /ask-selected uses hybrid scoring correctly."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        # Mock embeddings (different vectors for question and selected text)
        question_embedding = [0.1] * 1536
        selected_embedding = [0.2] * 1536
        mock_embed.side_effect = [question_embedding, selected_embedding]

        # Mock search with hybrid scores
        mock_search.return_value = [
            {
                "id": "chunk-001",
                "score": 0.82,  # Hybrid score
                "query_score": 0.75,
                "selected_score": 0.95,
                "payload": {
                    "content_text": "Navigation stack handles path planning.",
                    "section_heading": "ROS 2 Navigation",
                    "file_path": "/docs/chapter4.md"
                }
            }
        ]
        mock_agent.return_value = {
            "answer": "The navigation stack manages autonomous robot movement and path planning.",
            "sources": []
        }

        # Make request
        response = test_client.post(
            "/api/ask-selected",
            json={
                "question": "How does this work?",
                "selected_text": "The ROS 2 navigation stack provides autonomous navigation capabilities for mobile robots."
            }
        )

        # Assertions
        assert response.status_code == 200

        # Verify embed_query called twice (question + selected text)
        assert mock_embed.call_count == 2

        # Verify search called with both embeddings
        mock_search.assert_called_once()
        call_kwargs = mock_search.call_args[1]
        assert call_kwargs["query_embedding"] == question_embedding
        assert call_kwargs["selected_text_embedding"] == selected_embedding


def test_ask_selected_response_time(test_client):
    """Test that /ask-selected returns within acceptable time limits."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        # Mock responses
        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = []
        mock_agent.return_value = {
            "answer": "I don't have information about that.",
            "sources": []
        }

        # Make request and check latency
        response = test_client.post(
            "/api/ask-selected",
            json={
                "question": "Explain this concept",
                "selected_text": "A test passage about robotics that is long enough to be valid input."
            }
        )

        assert response.status_code == 200
        data = response.json()

        # Check that latency_ms is present and reasonable (< 5000ms for mocked calls)
        assert "latency_ms" in data
        assert data["latency_ms"] < 5000  # Should be fast with mocked services


def test_ask_endpoint_greeting_detection(test_client):
    """Test that greeting detection works correctly."""
    with patch('src.services.agent.generate_answer') as mock_agent:
        # Mock greeting response
        mock_agent.return_value = {
            "answer": "Hello! Welcome to the AI & Humanoid Robotics course!",
            "sources": [],
            "type": "greeting"
        }

        # Make request with greeting
        response = test_client.post(
            "/api/ask",
            json={"question": "hi"}
        )

        # Assertions
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "Hello" in data["answer"] or "hi" in data["answer"].lower()

        # Greeting should be fast (< 500ms)
        assert "latency_ms" in data
        # Note: With mocked services, latency will be minimal


def test_ask_endpoint_error_handling(test_client):
    """Test error handling for /ask endpoint."""
    # Test: Embedding service failure
    with patch('src.services.embeddings.embed_query') as mock_embed:
        mock_embed.side_effect = Exception("OpenAI API error")

        response = test_client.post(
            "/api/ask",
            json={"question": "What is ROS 2?"}
        )

        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "Embedding service unavailable" in data["detail"]

    # Test: Retrieval service failure
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search:

        mock_embed.return_value = [0.1] * 1536
        mock_search.side_effect = Exception("Qdrant connection error")

        response = test_client.post(
            "/api/ask",
            json={"question": "What is ROS 2?"}
        )

        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "Retrieval service unavailable" in data["detail"]


def test_ask_selected_endpoint_error_handling(test_client):
    """Test error handling for /ask-selected endpoint."""
    # Test: Agent generation failure
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = []
        mock_agent.side_effect = Exception("OpenAI API rate limit")

        response = test_client.post(
            "/api/ask-selected",
            json={
                "question": "Explain this",
                "selected_text": "A valid selected text passage for testing error handling."
            }
        )

        assert response.status_code == 503
        data = response.json()
        assert "detail" in data
        assert "Answer generation failed" in data["detail"]


def test_session_continuity(test_client):
    """Test that session_id is preserved across requests."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = []
        mock_agent.return_value = {
            "answer": "Test answer",
            "sources": []
        }

        # First request (no session_id)
        response1 = test_client.post(
            "/api/ask",
            json={"question": "What is ROS 2?"}
        )

        assert response1.status_code == 200
        session_id = response1.json()["session_id"]
        assert session_id is not None

        # Second request (with session_id)
        response2 = test_client.post(
            "/api/ask",
            json={
                "question": "Tell me more",
                "session_id": session_id
            }
        )

        assert response2.status_code == 200
        assert response2.json()["session_id"] == session_id


# ============================================================================
# Multi-Turn Conversation Tests (User Story 3)
# ============================================================================


def test_multi_turn_conversation_history(test_client):
    """Test that conversation history is maintained across multiple turns."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = [
            {
                "id": "chunk-001",
                "score": 0.85,
                "payload": {
                    "content_text": "ROS 2 is a robot operating system.",
                    "section_heading": "Introduction",
                    "file_path": "/docs/chapter1.md"
                }
            }
        ]

        # Mock agent to accept and use conversation history
        def mock_generate_with_history(question, retrieved_chunks, conversation_history=None):
            if conversation_history:
                return {
                    "answer": f"Based on our previous discussion about {conversation_history[0]['content'][:20]}... {question}",
                    "sources": []
                }
            return {"answer": "ROS 2 is a robot operating system.", "sources": []}

        mock_agent.side_effect = mock_generate_with_history

        # Turn 1: Initial question
        response1 = test_client.post(
            "/api/ask",
            json={"question": "What is ROS 2?"}
        )

        assert response1.status_code == 200
        session_id = response1.json()["session_id"]

        # Turn 2: Follow-up question (should have history)
        response2 = test_client.post(
            "/api/ask",
            json={
                "question": "How do I install it?",
                "session_id": session_id
            }
        )

        assert response2.status_code == 200

        # Verify agent was called with conversation_history on second turn
        assert mock_agent.call_count == 2
        second_call = mock_agent.call_args_list[1]
        assert second_call[1]["conversation_history"] is not None
        assert len(second_call[1]["conversation_history"]) > 0


def test_multi_turn_follow_up_questions(test_client):
    """Test that follow-up questions receive context from previous turns."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = []
        mock_agent.return_value = {"answer": "Test answer", "sources": []}

        # Turn 1
        response1 = test_client.post(
            "/api/ask",
            json={"question": "Explain inverse kinematics"}
        )
        session_id = response1.json()["session_id"]

        # Turn 2: Pronoun reference (requires history)
        response2 = test_client.post(
            "/api/ask",
            json={
                "question": "How is it used in robotics?",  # "it" refers to inverse kinematics
                "session_id": session_id
            }
        )

        # Turn 3: Further follow-up
        response3 = test_client.post(
            "/api/ask",
            json={
                "question": "Show me an example",
                "session_id": session_id
            }
        )

        # Verify all requests succeeded
        assert response1.status_code == 200
        assert response2.status_code == 200
        assert response3.status_code == 200

        # Verify conversation history grows
        call_args = mock_agent.call_args_list
        assert len(call_args[0][1].get("conversation_history", [])) == 0  # First turn: no history
        assert len(call_args[1][1].get("conversation_history", [])) >= 2  # Second turn: has Q1+A1
        assert len(call_args[2][1].get("conversation_history", [])) >= 4  # Third turn: has Q1+A1+Q2+A2


def test_multi_turn_max_history_limit(test_client):
    """Test that conversation history is limited to prevent token overflow."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = []
        mock_agent.return_value = {"answer": "Test answer", "sources": []}

        # Start conversation
        response = test_client.post(
            "/api/ask",
            json={"question": "Question 1"}
        )
        session_id = response.json()["session_id"]

        # Send many follow-up questions (more than max_turns limit)
        for i in range(12):  # Exceeds default max_turns=10
            response = test_client.post(
                "/api/ask",
                json={
                    "question": f"Question {i+2}",
                    "session_id": session_id
                }
            )
            assert response.status_code == 200

        # Verify last call has limited history (should be truncated to max_turns)
        last_call = mock_agent.call_args_list[-1]
        conversation_history = last_call[1].get("conversation_history", [])

        # History should be limited (session manager stores max 10 turns = 20 messages)
        assert len(conversation_history) <= 20  # 10 turns * 2 messages per turn


def test_multi_turn_conversation_with_selected_text(test_client):
    """Test multi-turn works with selected text endpoint."""
    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = []
        mock_agent.return_value = {"answer": "Test answer", "sources": []}

        # Turn 1: Regular question
        response1 = test_client.post(
            "/api/ask",
            json={"question": "What is ROS 2?"}
        )
        session_id = response1.json()["session_id"]

        # Turn 2: Selected text question (should preserve session)
        response2 = test_client.post(
            "/api/ask-selected",
            json={
                "question": "Explain this part in detail",
                "selected_text": "The ROS 2 navigation stack provides autonomous navigation capabilities for mobile robots.",
                "session_id": session_id
            }
        )

        # Verify session continuity
        assert response1.status_code == 200
        assert response2.status_code == 200
        assert response2.json()["session_id"] == session_id

        # Verify second call has history from first turn
        assert mock_agent.call_count == 2
        second_call = mock_agent.call_args_list[1]
        assert second_call[1].get("conversation_history") is not None


def test_multi_turn_session_expiry(test_client):
    """Test that expired sessions don't return stale history."""
    from src.utils.session import session_manager

    with patch('src.services.embeddings.embed_query') as mock_embed, \
         patch('src.services.retrieval.search') as mock_search, \
         patch('src.services.agent.generate_answer') as mock_agent:

        mock_embed.return_value = [0.1] * 1536
        mock_search.return_value = []
        mock_agent.return_value = {"answer": "Test answer", "sources": []}

        # Create initial conversation
        response1 = test_client.post(
            "/api/ask",
            json={"question": "What is ROS 2?"}
        )
        session_id = response1.json()["session_id"]

        # Manually clear the session (simulates expiry)
        session_manager.clear_session(session_id)

        # Try to continue conversation
        response2 = test_client.post(
            "/api/ask",
            json={
                "question": "Tell me more",
                "session_id": session_id
            }
        )

        # Should succeed but without history (treated as new conversation)
        assert response2.status_code == 200

        # Verify agent called without history
        second_call = mock_agent.call_args_list[1]
        conversation_history = second_call[1].get("conversation_history", [])
        assert len(conversation_history) == 0  # No history for expired session
