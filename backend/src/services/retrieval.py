"""Qdrant retrieval service for semantic search.

Handles vector search and reranking of document chunks.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter
from typing import List, Dict, Optional
from uuid import uuid4

from ..config import settings
from .embeddings import EMBEDDING_DIMENSIONS

# Initialize Qdrant client
client = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)

# Constants
COLLECTION_NAME = "book_content"
DEFAULT_LIMIT = 10
RERANK_LIMIT = 5


def create_collection():
    """Create Qdrant collection for book content.

    Collection schema:
    - Vector size: 768 (Gemini text-embedding-004)
    - Distance: Cosine
    - Indexed payload fields: chunk_id, file_path, section_heading, content_hash
    """
    try:
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBEDDING_DIMENSIONS, distance=Distance.COSINE
            ),
        )

        # Create payload indexes for fast filtering
        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="chunk_id",
            field_schema="keyword",
        )

        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="file_path",
            field_schema="keyword",
        )

        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="section_heading",
            field_schema="text",
        )

        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="content_hash",
            field_schema="keyword",
        )

        print(f"Collection '{COLLECTION_NAME}' created successfully")

    except Exception as e:
        print(f"Error creating collection: {e}")
        raise


def upsert_chunk(chunk_id: str, vector: List[float], payload: Dict) -> bool:
    """Insert or update a chunk in Qdrant.

    Args:
        chunk_id: Unique chunk identifier (UUID)
        vector: Embedding vector (768 dimensions)
        payload: Chunk metadata (file_path, section_heading, content_text, etc.)

    Returns:
        True if successful

    Raises:
        Exception: If upsert fails
    """
    try:
        point = PointStruct(id=chunk_id, vector=vector, payload=payload)

        client.upsert(collection_name=COLLECTION_NAME, points=[point])

        return True

    except Exception as e:
        print(f"Error upserting chunk {chunk_id}: {e}")
        raise


def search(
    query_embedding: List[float],
    limit: int = DEFAULT_LIMIT,
    score_threshold: float = None,
    selected_text_embedding: Optional[List[float]] = None,
) -> List[Dict]:
    """Search for similar chunks in Qdrant.

    Args:
        query_embedding: Query vector (768 dimensions)
        limit: Maximum number of results to return
        score_threshold: Minimum similarity score (0-1)
        selected_text_embedding: Optional embedding for selected text (for hybrid scoring)

    Returns:
        List of search results with scores and payloads

    Example result:
        [
            {
                "id": "chunk-001",
                "score": 0.85,
                "payload": {
                    "chunk_id": "chunk-001",
                    "file_path": "/docs/chapter1.md",
                    "section_heading": "Introduction",
                    "content_text": "...",
                    "chunk_index": 0,
                    ...
                }
            },
            ...
        ]
    """
    if score_threshold is None:
        score_threshold = settings.similarity_threshold

    try:
        # Basic semantic search using query_points (qdrant-client v1.7+)
        search_results = client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_embedding,
            limit=limit,
            score_threshold=score_threshold,
        )

        # Convert to dict format
        results = []
        for hit in search_results.points:
            result = {
                "id": str(hit.id),
                "score": hit.score,
                "payload": hit.payload,
            }
            results.append(result)

        # If selected text provided, apply hybrid scoring
        if selected_text_embedding and results:
            results = apply_hybrid_scoring(
                results, query_embedding, selected_text_embedding
            )

        # Rerank and limit to top N
        results = rerank_results(results, limit=min(limit, RERANK_LIMIT))

        return results

    except Exception as e:
        print(f"Error searching Qdrant: {e}")
        raise


def apply_hybrid_scoring(
    results: List[Dict],
    query_embedding: List[float],
    selected_text_embedding: List[float],
    query_weight: float = 0.7,
    selected_weight: float = 0.3,
) -> List[Dict]:
    """Apply hybrid scoring: combine query similarity and selected text similarity.

    Args:
        results: Initial search results
        query_embedding: Query vector
        selected_text_embedding: Selected text vector
        query_weight: Weight for query similarity (default 0.7)
        selected_weight: Weight for selected text similarity (default 0.3)

    Returns:
        Results with updated hybrid scores
    """
    from .embeddings import cosine_similarity

    # Search again with selected text embedding to get selected text scores
    selected_results = client.query_points(
        collection_name=COLLECTION_NAME,
        query=selected_text_embedding,
        limit=len(results) * 2,  # Get more results for better coverage
    )

    # Create lookup for selected text scores
    selected_scores = {str(hit.id): hit.score for hit in selected_results.points}

    # Compute hybrid scores
    for result in results:
        chunk_id = result["id"]
        query_score = result["score"]
        selected_score = selected_scores.get(chunk_id, 0.0)

        # Hybrid score = weighted combination
        hybrid_score = (query_weight * query_score) + (selected_weight * selected_score)

        result["score"] = hybrid_score
        result["query_score"] = query_score
        result["selected_score"] = selected_score

    # Re-sort by hybrid score
    results.sort(key=lambda x: x["score"], reverse=True)

    return results


def rerank_results(results: List[Dict], limit: int = RERANK_LIMIT) -> List[Dict]:
    """Rerank results and return top N.

    Args:
        results: Search results
        limit: Number of top results to return

    Returns:
        Top N reranked results
    """
    # Already sorted by score from Qdrant
    return results[:limit]


def delete_chunk(chunk_id: str) -> bool:
    """Delete a chunk from Qdrant.

    Args:
        chunk_id: Chunk ID to delete

    Returns:
        True if successful
    """
    try:
        client.delete(collection_name=COLLECTION_NAME, points_selector=[chunk_id])

        return True

    except Exception as e:
        print(f"Error deleting chunk {chunk_id}: {e}")
        raise


def get_collection_info() -> Dict:
    """Get collection statistics.

    Returns:
        Dictionary with collection info (count, status, etc.)
    """
    try:
        info = client.get_collection(collection_name=COLLECTION_NAME)

        # Access nested properties (Qdrant client v1.7+)
        result = {
            "name": COLLECTION_NAME,
            "points_count": info.points_count if hasattr(info, 'points_count') else 0,
        }

        # Add vectors_count if available (older API versions)
        if hasattr(info, 'vectors_count'):
            result["vectors_count"] = info.vectors_count

        # Add status
        if hasattr(info, 'status'):
            result["status"] = info.status

        return result

    except Exception as e:
        print(f"Error getting collection info: {e}")
        return {"error": str(e)}
