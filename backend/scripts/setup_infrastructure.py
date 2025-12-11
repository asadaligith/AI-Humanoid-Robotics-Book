"""Setup script for initializing Qdrant collection and Postgres database.

Run this script once to set up the infrastructure before indexing.
"""

import sys
import os

# Add parent directory to path to import from src
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.services.retrieval import create_collection, get_collection_info, client
from src.services.database import create_tables, get_db
from src.config import settings

def setup_qdrant():
    """Create Qdrant collection with proper configuration."""
    print("=" * 60)
    print("Setting up Qdrant Collection")
    print("=" * 60)

    try:
        # Check if collection already exists
        try:
            info = get_collection_info()
            print(f"✅ Collection already exists:")
            print(f"   - Name: {info['name']}")
            print(f"   - Points: {info.get('points_count', 0)}")
            print(f"   - Vectors: {info.get('vectors_count', 0)}")
            print(f"   - Status: {info.get('status', 'unknown')}")
            return True
        except Exception:
            # Collection doesn't exist, create it
            pass

        print(f"📝 Creating collection 'book_content'...")
        print(f"   - Vector dimensions: 768 (Gemini text-embedding-004)")
        print(f"   - Distance metric: Cosine")
        print(f"   - Qdrant URL: {settings.qdrant_url}")

        create_collection()

        # Verify creation
        info = get_collection_info()
        print(f"✅ Collection created successfully!")
        print(f"   - Name: {info['name']}")
        print(f"   - Status: {info.get('status', 'unknown')}")

        return True

    except Exception as e:
        print(f"❌ Error setting up Qdrant: {e}")
        print(f"\nPlease verify:")
        print(f"  1. QDRANT_URL is correct in .env")
        print(f"  2. QDRANT_API_KEY is valid")
        print(f"  3. Qdrant cluster is accessible")
        return False


def setup_postgres():
    """Create Postgres tables with proper schema."""
    print("\n" + "=" * 60)
    print("Setting up Neon Postgres Database")
    print("=" * 60)

    try:
        print(f"📝 Creating database tables...")
        print(f"   - Database URL: {settings.database_url[:50]}...")

        create_tables()

        # Verify tables exist
        with get_db() as db:
            from src.services.database import ChunkMetadata
            count = db.query(ChunkMetadata).count()
            print(f"✅ Tables created successfully!")
            print(f"   - chunks_metadata table exists")
            print(f"   - Current row count: {count}")

        return True

    except Exception as e:
        print(f"❌ Error setting up Postgres: {e}")
        print(f"\nPlease verify:")
        print(f"  1. DATABASE_URL is correct in .env")
        print(f"  2. Neon database is accessible")
        print(f"  3. Database user has CREATE TABLE permissions")
        return False


def verify_gemini():
    """Verify Gemini API access."""
    print("\n" + "=" * 60)
    print("Verifying Gemini API Access")
    print("=" * 60)

    try:
        from src.services.embeddings import embed_text

        print(f"📝 Testing Gemini API...")
        print(f"   - Testing with sample text")

        test_embedding = embed_text("This is a test.")

        print(f"✅ Gemini API is working!")
        print(f"   - Generated {len(test_embedding)}D vector")
        print(f"   - API Key: {settings.gemini_api_key[:20]}...")

        return True

    except Exception as e:
        print(f"❌ Error accessing Gemini API: {e}")
        print(f"\nPlease verify:")
        print(f"  1. GEMINI_API_KEY is correct in .env")
        print(f"  2. API key is valid and has proper permissions")
        print(f"  3. Network connectivity to Gemini API")
        return False


def main():
    """Main setup function."""
    print("\n" + "=" * 60)
    print("🚀 RAG Chatbot Infrastructure Setup")
    print("=" * 60)
    print(f"\nEnvironment: {settings.environment}")
    print(f"Using Google Gemini for embeddings and generation\n")

    # Track success
    results = {
        "gemini": False,
        "qdrant": False,
        "postgres": False
    }

    # Step 1: Verify Gemini API
    results["gemini"] = verify_gemini()

    # Step 2: Setup Qdrant
    if results["gemini"]:
        results["qdrant"] = setup_qdrant()
    else:
        print("\n⚠️  Skipping Qdrant setup due to Gemini API issues")

    # Step 3: Setup Postgres
    results["postgres"] = setup_postgres()

    # Summary
    print("\n" + "=" * 60)
    print("Setup Summary")
    print("=" * 60)

    for service, success in results.items():
        status = "✅ Ready" if success else "❌ Failed"
        print(f"  {service.capitalize():12} {status}")

    if all(results.values()):
        print("\n🎉 All systems ready! You can now:")
        print("   1. Run: python scripts/reindex_book.py")
        print("   2. Start API: uvicorn src.main:app --reload")
    else:
        print("\n⚠️  Some services failed. Fix the issues above and try again.")
        sys.exit(1)


if __name__ == "__main__":
    main()
