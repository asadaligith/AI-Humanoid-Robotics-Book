"""Comprehensive error messages for common failures.

Provides user-friendly error messages for different failure scenarios.
"""

from typing import Dict, Optional
from fastapi import HTTPException, status


class ErrorMessages:
    """Centralized error messages for the chatbot."""

    # Service unavailability
    QDRANT_UNAVAILABLE = (
        "The chatbot is temporarily unavailable due to a database issue. "
        "Please try again in a few moments."
    )

    OPENAI_UNAVAILABLE = (
        "The AI service is temporarily unavailable. "
        "Please try again in a few moments."
    )

    EMBEDDING_SERVICE_UNAVAILABLE = (
        "Unable to process your question right now due to a technical issue. "
        "Please try again in a few moments."
    )

    # Rate limiting
    RATE_LIMIT_EXCEEDED = (
        "You're asking questions too quickly. "
        "Please wait 60 seconds before trying again."
    )

    OPENAI_RATE_LIMIT = (
        "The AI service is currently busy. "
        "Please try again in about 60 seconds."
    )

    # No results
    NO_INFORMATION = (
        "I don't have information about that in the AI & Humanoid Robotics course materials. "
        "Could you try rephrasing your question or asking about a topic covered in the curriculum?"
    )

    NO_RESULTS_FOUND = (
        "I couldn't find relevant information to answer your question. "
        "Please try rephrasing or ask about a different topic from the course."
    )

    # Generation failures
    ANSWER_GENERATION_FAILED = (
        "I encountered an error while generating the answer. "
        "Please try asking your question again."
    )

    # Input validation
    QUESTION_TOO_LONG = "Your question is too long. Please keep it under 2000 characters."

    QUESTION_EMPTY = "Please enter a question."

    SELECTED_TEXT_TOO_LONG = (
        "The selected text is too long. Please select a shorter passage (max 5000 characters)."
    )

    SELECTED_TEXT_TOO_SHORT = (
        "The selected text is too short. Please select at least 10 characters."
    )

    INVALID_SESSION_ID = "Invalid session ID format."

    # Generic errors
    INTERNAL_ERROR = (
        "An unexpected error occurred. Please try again. "
        "If the problem persists, please contact support."
    )

    TIMEOUT_ERROR = (
        "The request took too long to process. "
        "Please try asking a simpler question or try again later."
    )


def create_error_response(
    error_type: str,
    detail: Optional[str] = None,
    status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR,
) -> HTTPException:
    """Create a standardized error response.

    Args:
        error_type: Type of error (e.g., "qdrant_unavailable", "rate_limit")
        detail: Optional custom detail message
        status_code: HTTP status code

    Returns:
        HTTPException with appropriate message

    Example:
        raise create_error_response("qdrant_unavailable", status_code=503)
    """
    error_messages = {
        "qdrant_unavailable": ErrorMessages.QDRANT_UNAVAILABLE,
        "openai_unavailable": ErrorMessages.OPENAI_UNAVAILABLE,
        "embedding_unavailable": ErrorMessages.EMBEDDING_SERVICE_UNAVAILABLE,
        "rate_limit": ErrorMessages.RATE_LIMIT_EXCEEDED,
        "openai_rate_limit": ErrorMessages.OPENAI_RATE_LIMIT,
        "no_information": ErrorMessages.NO_INFORMATION,
        "no_results": ErrorMessages.NO_RESULTS_FOUND,
        "generation_failed": ErrorMessages.ANSWER_GENERATION_FAILED,
        "timeout": ErrorMessages.TIMEOUT_ERROR,
        "internal_error": ErrorMessages.INTERNAL_ERROR,
    }

    message = error_messages.get(error_type, ErrorMessages.INTERNAL_ERROR)

    # Use custom detail if provided
    if detail:
        message = detail

    return HTTPException(status_code=status_code, detail=message)


def handle_service_error(exception: Exception, service_name: str) -> HTTPException:
    """Convert service exceptions to user-friendly errors.

    Args:
        exception: The caught exception
        service_name: Name of the service (e.g., "qdrant", "openai", "embedding")

    Returns:
        HTTPException with user-friendly message

    Example:
        try:
            result = search_qdrant(...)
        except Exception as e:
            raise handle_service_error(e, "qdrant")
    """
    error_str = str(exception).lower()

    # Rate limiting
    if "429" in error_str or "rate limit" in error_str or "too many requests" in error_str:
        if service_name == "openai":
            return create_error_response(
                "openai_rate_limit", status_code=status.HTTP_429_TOO_MANY_REQUESTS
            )
        else:
            return create_error_response(
                "rate_limit", status_code=status.HTTP_429_TOO_MANY_REQUESTS
            )

    # Connection/availability errors
    if "connection" in error_str or "timeout" in error_str or "unavailable" in error_str:
        if service_name == "qdrant":
            return create_error_response(
                "qdrant_unavailable", status_code=status.HTTP_503_SERVICE_UNAVAILABLE
            )
        elif service_name == "openai":
            return create_error_response(
                "openai_unavailable", status_code=status.HTTP_503_SERVICE_UNAVAILABLE
            )
        elif service_name == "embedding":
            return create_error_response(
                "embedding_unavailable", status_code=status.HTTP_503_SERVICE_UNAVAILABLE
            )

    # Default to internal error
    return create_error_response(
        "internal_error", status_code=status.HTTP_500_INTERNAL_SERVER_ERROR
    )
