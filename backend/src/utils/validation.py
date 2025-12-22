"""Input validation and sanitization utilities.

Prevents injection attacks and validates user input.
"""

import re
from typing import Optional
from fastapi import HTTPException, status


# Dangerous patterns to detect
SQL_INJECTION_PATTERNS = [
    r"(\b(SELECT|INSERT|UPDATE|DELETE|DROP|CREATE|ALTER|EXEC|EXECUTE)\b)",
    r"(--|\#|\/\*|\*\/)",  # SQL comments
    r"(\bOR\b.*=.*\bOR\b)",  # OR-based injection
    r"(\bAND\b.*=.*\bAND\b)",  # AND-based injection
    r"(;.*\b(SELECT|INSERT|UPDATE|DELETE|DROP)\b)",  # Stacked queries
]

XSS_PATTERNS = [
    r"<script[^>]*>.*?</script>",  # Script tags
    r"javascript:",  # JavaScript protocol
    r"on\w+\s*=",  # Event handlers (onclick, onload, etc.)
]

# HTML tag pattern
HTML_TAG_PATTERN = r"<[^>]+>"

# Maximum lengths
MAX_QUESTION_LENGTH = 2000
MAX_SELECTED_TEXT_LENGTH = 5000
MAX_SESSION_ID_LENGTH = 100


def sanitize_html(text: str) -> str:
    """Remove HTML tags from text.

    Args:
        text: Input text potentially containing HTML

    Returns:
        Text with HTML tags removed
    """
    if not text:
        return ""

    # Remove HTML tags
    cleaned = re.sub(HTML_TAG_PATTERN, "", text, flags=re.IGNORECASE | re.DOTALL)

    return cleaned.strip()


def detect_sql_injection(text: str) -> bool:
    """Detect potential SQL injection patterns.

    Args:
        text: Input text to check

    Returns:
        True if SQL injection pattern detected
    """
    if not text:
        return False

    text_upper = text.upper()

    for pattern in SQL_INJECTION_PATTERNS:
        if re.search(pattern, text_upper, re.IGNORECASE):
            return True

    return False


def detect_xss(text: str) -> bool:
    """Detect potential XSS patterns.

    Args:
        text: Input text to check

    Returns:
        True if XSS pattern detected
    """
    if not text:
        return False

    for pattern in XSS_PATTERNS:
        if re.search(pattern, text, re.IGNORECASE):
            return True

    return False


def validate_and_sanitize_question(
    question: str, max_length: int = MAX_QUESTION_LENGTH, allow_short: bool = True
) -> str:
    """Validate and sanitize user question.

    Args:
        question: User's question
        max_length: Maximum allowed length
        allow_short: Allow short questions like greetings (default: True)

    Returns:
        Sanitized question

    Raises:
        HTTPException: If validation fails
    """
    if not question or not question.strip():
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Question cannot be empty",
        )

    # Check length
    if len(question) > max_length:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=f"Question too long (max {max_length} characters)",
        )

    # Allow short greetings (e.g., "hi", "hello")
    # Don't require minimum length for greetings
    if not allow_short and len(question.strip()) < 3:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Question too short (minimum 3 characters)",
        )

    # Check for SQL injection
    if detect_sql_injection(question):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid input: potential SQL injection detected",
        )

    # Check for XSS
    if detect_xss(question):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid input: HTML/JavaScript not allowed",
        )

    # Sanitize HTML tags
    sanitized = sanitize_html(question)

    # Ensure still not empty after sanitization
    if not sanitized:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Question cannot be empty after sanitization",
        )

    return sanitized


def validate_and_sanitize_selected_text(
    selected_text: str, max_length: int = MAX_SELECTED_TEXT_LENGTH
) -> str:
    """Validate and sanitize selected text.

    Args:
        selected_text: User's selected text
        max_length: Maximum allowed length

    Returns:
        Sanitized selected text

    Raises:
        HTTPException: If validation fails
    """
    if not selected_text or not selected_text.strip():
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Selected text cannot be empty",
        )

    # Check length
    if len(selected_text) > max_length:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=f"Selected text too long (max {max_length} characters)",
        )

    # Check minimum length (at least 10 chars for meaningful selection)
    if len(selected_text) < 10:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Selected text too short (minimum 10 characters)",
        )

    # Check for SQL injection
    if detect_sql_injection(selected_text):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid input: potential SQL injection detected",
        )

    # Sanitize HTML tags
    sanitized = sanitize_html(selected_text)

    # Ensure still valid after sanitization
    if len(sanitized) < 10:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Selected text too short after sanitization (minimum 10 characters)",
        )

    return sanitized


def validate_session_id(
    session_id: Optional[str], max_length: int = MAX_SESSION_ID_LENGTH
) -> Optional[str]:
    """Validate session ID.

    Args:
        session_id: Session identifier
        max_length: Maximum allowed length

    Returns:
        Validated session ID or None

    Raises:
        HTTPException: If validation fails
    """
    if not session_id:
        return None

    # Check length
    if len(session_id) > max_length:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=f"Session ID too long (max {max_length} characters)",
        )

    # Session ID should be alphanumeric with dashes/underscores only
    if not re.match(r"^[a-zA-Z0-9_-]+$", session_id):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Session ID contains invalid characters (only alphanumeric, dash, underscore allowed)",
        )

    return session_id


def sanitize_user_input(text: str) -> str:
    """General-purpose sanitization for user input.

    Removes HTML tags, trims whitespace, normalizes line breaks.

    Args:
        text: Input text

    Returns:
        Sanitized text
    """
    if not text:
        return ""

    # Remove HTML tags
    cleaned = sanitize_html(text)

    # Normalize whitespace (replace multiple spaces with single space)
    cleaned = re.sub(r"\s+", " ", cleaned)

    # Trim
    cleaned = cleaned.strip()

    return cleaned
