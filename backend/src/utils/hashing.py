"""Content hashing utilities for deduplication.

Uses SHA256 for computing content hashes.
"""

import hashlib


def compute_hash(text: str) -> str:
    """Compute SHA256 hash of text content.

    Args:
        text: Input text to hash

    Returns:
        64-character hex string (SHA256 hash)

    Example:
        >>> compute_hash("Hello, world!")
        '315f5bdb76d078c43b8ac0064e4a0164612b1fce77c869345bfc94c75894edd3'
    """
    # Normalize text: strip whitespace and convert to UTF-8
    normalized_text = text.strip().encode("utf-8")

    # Compute SHA256 hash
    hash_object = hashlib.sha256(normalized_text)

    # Return hex digest (64 characters)
    return hash_object.hexdigest()


def verify_hash(text: str, expected_hash: str) -> bool:
    """Verify text matches expected hash.

    Args:
        text: Input text to verify
        expected_hash: Expected SHA256 hash (64 hex chars)

    Returns:
        True if hash matches, False otherwise

    Example:
        >>> verify_hash("Hello, world!", "315f5bdb76d078c43b8ac0064e4a0164612b1fce77c869345bfc94c75894edd3")
        True
    """
    computed_hash = compute_hash(text)
    return computed_hash == expected_hash


def content_changed(text: str, stored_hash: str) -> bool:
    """Check if content has changed compared to stored hash.

    Args:
        text: Current text content
        stored_hash: Previously stored SHA256 hash

    Returns:
        True if content has changed, False if unchanged

    Example:
        >>> content_changed("New content", "old_hash_value")
        True
    """
    return not verify_hash(text, stored_hash)
