"""Pydantic request models for API endpoints."""

from pydantic import BaseModel, Field, field_validator
from typing import Optional
from uuid import UUID, uuid4


class AskRequest(BaseModel):
    """Request model for POST /ask endpoint."""

    question: str = Field(
        ...,
        min_length=5,
        max_length=2000,
        description="User's question about the book content",
        examples=["What are the key components of a humanoid robot?"],
    )

    session_id: str = Field(
        default_factory=lambda: str(uuid4()),
        description="Session ID for multi-turn conversation tracking",
        examples=["550e8400-e29b-41d4-a716-446655440000"],
    )


class AskSelectedRequest(BaseModel):
    """Request model for POST /ask-selected endpoint."""

    question: str = Field(
        ...,
        min_length=5,
        max_length=2000,
        description="User's question about the selected text",
        examples=["Explain this in simpler terms"],
    )

    selected_text: str = Field(
        ...,
        min_length=10,
        max_length=5000,
        description="Text selected by the user for context",
        examples=[
            "Inverse kinematics is the process of determining the joint parameters that provide a desired position of the robot's end-effector."
        ],
    )

    session_id: str = Field(
        default_factory=lambda: str(uuid4()),
        description="Session ID for multi-turn conversation tracking",
    )
