"""Google Gemini embedding generation service.

Uses text-embedding-004 model for semantic search.
"""

import google.generativeai as genai
from typing import List
import time

from ..config import settings

# Configure Gemini
genai.configure(api_key=settings.gemini_api_key)

# Constants
EMBEDDING_MODEL = "models/text-embedding-004"
EMBEDDING_DIMENSIONS = 768  # Gemini text-embedding-004 dimensions


def embed_text(text: str, task_type: str = "RETRIEVAL_DOCUMENT") -> List[float]:
    """Generate embedding vector for a single text using Gemini.

    Args:
        text: Input text to embed
        task_type: Task type for Gemini ("RETRIEVAL_DOCUMENT" for indexing,
                   "RETRIEVAL_QUERY" for queries)

    Returns:
        List of 768 floats representing the embedding vector

    Raises:
        ValueError: If text is empty
        API errors: If Gemini API call fails
    """
    if not text or not text.strip():
        raise ValueError("Text cannot be empty")

    try:
        result = genai.embed_content(
            model=EMBEDDING_MODEL,
            content=text,
            task_type=task_type,
        )

        embedding = result["embedding"]

        # Validate dimensions
        if len(embedding) != EMBEDDING_DIMENSIONS:
            raise ValueError(
                f"Expected {EMBEDDING_DIMENSIONS} dimensions, got {len(embedding)}"
            )

        return embedding

    except Exception as e:
        print(f"Error generating embedding: {e}")
        raise


def embed_query(text: str) -> List[float]:
    """Generate embedding for a search query.

    Convenience function that uses RETRIEVAL_QUERY task type.

    Args:
        text: Query text

    Returns:
        List of 768 floats representing the embedding vector
    """
    return embed_text(text, task_type="RETRIEVAL_QUERY")


def embed_document(text: str) -> List[float]:
    """Generate embedding for a document chunk.

    Convenience function that uses RETRIEVAL_DOCUMENT task type.

    Args:
        text: Document text

    Returns:
        List of 768 floats representing the embedding vector
    """
    return embed_text(text, task_type="RETRIEVAL_DOCUMENT")


def embed_batch(
    texts: List[str], task_type: str = "RETRIEVAL_DOCUMENT", batch_size: int = 100
) -> List[List[float]]:
    """Generate embeddings for multiple texts in batches.

    Args:
        texts: List of texts to embed
        task_type: Task type for Gemini
        batch_size: Number of texts per API request

    Returns:
        List of embedding vectors (same order as input texts)

    Raises:
        ValueError: If texts list is empty
        API errors: If Gemini API call fails
    """
    if not texts:
        raise ValueError("Texts list cannot be empty")

    all_embeddings = []

    # Process in batches
    for i in range(0, len(texts), batch_size):
        batch = texts[i : i + batch_size]

        try:
            # Gemini supports batch embedding
            result = genai.embed_content(
                model=EMBEDDING_MODEL,
                content=batch,
                task_type=task_type,
            )

            # Extract embeddings
            batch_embeddings = result["embedding"]

            # Handle both single and batch responses
            if isinstance(batch_embeddings[0], list):
                all_embeddings.extend(batch_embeddings)
            else:
                all_embeddings.append(batch_embeddings)

            # Rate limiting: sleep briefly between batches
            if i + batch_size < len(texts):
                time.sleep(0.1)

        except Exception as e:
            print(f"Error generating batch embeddings (batch {i // batch_size}): {e}")
            raise

    return all_embeddings


def cosine_similarity(vec1: List[float], vec2: List[float]) -> float:
    """Calculate cosine similarity between two vectors.

    Args:
        vec1: First embedding vector
        vec2: Second embedding vector

    Returns:
        Cosine similarity score between -1 and 1 (higher = more similar)

    Raises:
        ValueError: If vectors have different dimensions
    """
    if len(vec1) != len(vec2):
        raise ValueError(
            f"Vectors must have same dimensions: {len(vec1)} != {len(vec2)}"
        )

    # Calculate dot product
    dot_product = sum(a * b for a, b in zip(vec1, vec2))

    # Calculate magnitudes
    mag1 = sum(a * a for a in vec1) ** 0.5
    mag2 = sum(b * b for b in vec2) ** 0.5

    # Avoid division by zero
    if mag1 == 0 or mag2 == 0:
        return 0.0

    return dot_product / (mag1 * mag2)
