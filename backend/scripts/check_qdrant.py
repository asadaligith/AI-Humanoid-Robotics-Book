"""Check Qdrant Cloud collection status and data."""

import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.services.retrieval import client, COLLECTION_NAME
from src.services.embeddings import embed_query

def check_collection():
    """Check Qdrant collection status."""
    print("=" * 60)
    print("Qdrant Cloud Diagnostic Check")
    print("=" * 60)

    try:
        # Get collection info
        info = client.get_collection(collection_name=COLLECTION_NAME)

        print(f"\n[OK] Collection exists: {COLLECTION_NAME}")
        print(f"  Points count: {info.points_count}")
        print(f"  Status: {info.status}")

        if hasattr(info, 'vectors_count'):
            print(f"  Vectors count: {info.vectors_count}")

        # Test search with a simple query
        print(f"\n[TEST] Testing search with query: 'robot'")
        test_embedding = embed_query("robot")

        results = client.query_points(
            collection_name=COLLECTION_NAME,
            query=test_embedding,
            limit=5,
            score_threshold=0.0,  # No threshold for diagnostic
        )

        print(f"\n[RESULTS] Search Results:")
        print(f"  Found {len(results.points)} points")

        if results.points:
            print(f"\n  Top 3 results:")
            for i, point in enumerate(results.points[:3], 1):
                print(f"    {i}. Score: {point.score:.3f}")
                if hasattr(point, 'payload') and point.payload:
                    print(f"       File: {point.payload.get('file_path', 'N/A')}")
                    print(f"       Section: {point.payload.get('section_heading', 'N/A')}")
                    content = point.payload.get('content_text', '')
                    preview = content[:100] + "..." if len(content) > 100 else content
                    print(f"       Content: {preview}")
        else:
            print("\n  [WARNING] No results found!")
            print("  This means the collection is empty or embeddings don't match.")

        # Try scrolling through points
        print(f"\n[SAMPLE] Sampling collection data...")
        scroll_result = client.scroll(
            collection_name=COLLECTION_NAME,
            limit=3,
        )

        points, _ = scroll_result
        print(f"  Retrieved {len(points)} sample points:")

        for i, point in enumerate(points, 1):
            print(f"\n  Point {i}:")
            print(f"    ID: {point.id}")
            if hasattr(point, 'payload') and point.payload:
                print(f"    File: {point.payload.get('file_path', 'N/A')}")
                print(f"    Section: {point.payload.get('section_heading', 'N/A')}")
            if hasattr(point, 'vector') and point.vector:
                print(f"    Vector dimensions: {len(point.vector) if isinstance(point.vector, list) else 'N/A'}")

        print("\n" + "=" * 60)
        print("[SUCCESS] Diagnostic complete!")
        print("=" * 60)

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_collection()
