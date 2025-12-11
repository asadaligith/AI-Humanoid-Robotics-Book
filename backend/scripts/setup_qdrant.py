"""Setup Qdrant collection for book content indexing.

Creates the 'book_content' collection with proper vector configuration:
- 768D vectors (Gemini text-embedding-004)
- Cosine distance metric
- HNSW indexing for fast similarity search
"""

import sys
import os
from pathlib import Path

# Fix Windows console encoding for emoji support
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, HnswConfigDiff
from dotenv import load_dotenv


def load_environment():
    """Load environment variables from .env file."""
    # Try to load from backend/.env
    env_path = Path(__file__).parent.parent / '.env'

    if env_path.exists():
        load_dotenv(env_path)
        print(f"✓ Loaded environment from: {env_path}")
    else:
        print(f"⚠️  No .env file found at {env_path}")
        print("   Using system environment variables...")

    # Validate required variables
    required_vars = ['QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"\n❌ Missing required environment variables: {', '.join(missing_vars)}")
        print("   Please set them in backend/.env or system environment")
        sys.exit(1)

    return {
        'url': os.getenv('QDRANT_URL'),
        'api_key': os.getenv('QDRANT_API_KEY')
    }


def create_collection(client: QdrantClient, collection_name: str = "book_content"):
    """Create Qdrant collection with proper configuration.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection to create

    Returns:
        bool: True if created successfully, False if already exists
    """
    print(f"\n📦 Creating collection: {collection_name}")
    print("=" * 60)

    # Check if collection already exists
    try:
        collections = client.get_collections().collections
        existing_names = [c.name for c in collections]

        if collection_name in existing_names:
            print(f"⚠️  Collection '{collection_name}' already exists!")

            # Get collection info
            info = client.get_collection(collection_name)
            print(f"\n📊 Existing collection details:")
            print(f"   Vector size:    {info.config.params.vectors.size}D")
            print(f"   Distance:       {info.config.params.vectors.distance}")
            print(f"   Points count:   {info.points_count}")
            print(f"   Vectors count:  {info.vectors_count}")

            # Validate configuration
            if info.config.params.vectors.size != 768:
                print(f"\n❌ ERROR: Vector size mismatch!")
                print(f"   Expected: 768D (Gemini text-embedding-004)")
                print(f"   Found:    {info.config.params.vectors.size}D")
                print(f"\n   To fix: Delete the collection and re-run this script")
                return False

            if info.config.params.vectors.distance != Distance.COSINE:
                print(f"\n❌ ERROR: Distance metric mismatch!")
                print(f"   Expected: {Distance.COSINE}")
                print(f"   Found:    {info.config.params.vectors.distance}")
                print(f"\n   To fix: Delete the collection and re-run this script")
                return False

            print(f"\n✅ Collection configuration is correct!")
            return True

    except Exception as e:
        print(f"⚠️  Could not check existing collections: {e}")

    # Create new collection
    try:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=768,  # Gemini text-embedding-004 dimension
                distance=Distance.COSINE
            ),
            hnsw_config=HnswConfigDiff(
                m=16,             # Number of edges per node
                ef_construct=100  # Construction time accuracy
            )
        )

        print(f"\n✅ Collection '{collection_name}' created successfully!")
        print(f"\n📊 Configuration:")
        print(f"   Vector size:    768D (Gemini text-embedding-004)")
        print(f"   Distance:       Cosine")
        print(f"   HNSW m:         16")
        print(f"   HNSW ef:        100")

        return True

    except Exception as e:
        print(f"\n❌ Failed to create collection: {e}")
        return False


def main():
    """Main setup function."""
    print("\n" + "=" * 60)
    print("🚀 Qdrant Collection Setup")
    print("=" * 60)

    try:
        # Step 1: Load environment
        print("\n[1/3] Loading environment variables...")
        config = load_environment()

        # Step 2: Connect to Qdrant
        print("\n[2/3] Connecting to Qdrant...")
        print(f"   URL: {config['url']}")

        client = QdrantClient(
            url=config['url'],
            api_key=config['api_key']
        )

        # Test connection
        try:
            collections = client.get_collections()
            print(f"✓ Connected successfully")
            print(f"   Existing collections: {len(collections.collections)}")
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            sys.exit(1)

        # Step 3: Create collection
        print("\n[3/3] Setting up collection...")
        success = create_collection(client, "book_content")

        if success:
            print("\n" + "=" * 60)
            print("✅ Setup complete!")
            print("=" * 60)
            print("\nNext steps:")
            print("1. Complete T008: Set up Postgres schema")
            print("2. Run: python scripts/reindex_book.py --source ../docs")
            print("=" * 60)
        else:
            print("\n" + "=" * 60)
            print("⚠️  Setup completed with warnings")
            print("=" * 60)
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\n⚠️  Setup interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Setup failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
