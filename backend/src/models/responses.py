"""Pydantic response models for API endpoints."""

from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime


class Source(BaseModel):
    """Citation source for an answer."""

    file: str = Field(
        ...,
        description="Source markdown file path",
        examples=["/docs/chapter1/introduction.md"],
    )

    section: Optional[str] = Field(
        default=None,
        description="Section or chapter heading",
        examples=["Introduction to Humanoid Robots"],
    )

    chunk: str = Field(
        ...,
        max_length=2000,
        description="Excerpt from the source chunk",
        examples=[
            "Humanoid robots are designed to mimic human appearance and behavior..."
        ],
    )

    similarity: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Relevance score (cosine similarity)",
        examples=[0.85],
    )


class ChatResponse(BaseModel):
    """Response model for /ask and /ask-selected endpoints."""

    answer: str = Field(
        ...,
        description="Generated answer from the RAG agent",
        examples=[
            "Humanoid robots consist of several key components including sensors, actuators, processors, and control systems."
        ],
    )

    sources: List[Source] = Field(
        ...,
        min_length=0,
        max_length=5,
        description="Citations for the answer",
    )

    session_id: str = Field(
        ...,
        description="Session ID for conversation tracking",
        examples=["550e8400-e29b-41d4-a716-446655440000"],
    )

    latency_ms: Optional[int] = Field(
        default=None,
        description="Response generation time in milliseconds",
        examples=[1200],
    )


class ServiceStatus(BaseModel):
    """Status of an external service."""

    name: str = Field(..., description="Service name", examples=["qdrant", "postgres"])
    healthy: bool = Field(..., description="Whether service is reachable")
    latency_ms: Optional[int] = Field(
        default=None, description="Health check latency in milliseconds"
    )
    error: Optional[str] = Field(
        default=None, description="Error message if unhealthy"
    )


class HealthResponse(BaseModel):
    """Response model for GET /health endpoint."""

    status: str = Field(
        ...,
        description="Overall health status",
        examples=["healthy", "degraded", "unhealthy"],
    )

    services: List[ServiceStatus] = Field(
        ..., description="Status of external dependencies"
    )

    version: str = Field(
        ..., description="API version", examples=["1.0.0"]
    )

    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Health check timestamp",
    )


class ErrorResponse(BaseModel):
    """Response model for error responses."""

    error: str = Field(
        ...,
        description="Error message",
        examples=["Invalid request", "Service unavailable"],
    )

    code: str = Field(
        ...,
        description="Error code",
        examples=["VALIDATION_ERROR", "SERVICE_UNAVAILABLE"],
    )

    details: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Additional error details",
    )

    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Error timestamp",
    )
