"""Setup Neon Postgres database schema for chunk metadata.

Creates the chunks_metadata table with proper indexes, constraints, and triggers.
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

from sqlalchemy import create_engine, text
from dotenv import load_dotenv


# SQL schema from data-model.md
SCHEMA_SQL = """
-- Drop existing table if needed (only for development)
-- DROP TABLE IF EXISTS chunks_metadata CASCADE;

-- Create chunks_metadata table
CREATE TABLE IF NOT EXISTS chunks_metadata (
  chunk_id UUID PRIMARY KEY,
  file_path VARCHAR(512) NOT NULL,
  section_heading VARCHAR(512),
  chunk_index INTEGER NOT NULL,
  content_hash CHAR(64) NOT NULL UNIQUE,
  token_count INTEGER NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  CONSTRAINT chunk_index_non_negative CHECK (chunk_index >= 0),
  CONSTRAINT token_count_range CHECK (token_count BETWEEN 100 AND 2000)
);

-- Create indexes
CREATE INDEX IF NOT EXISTS idx_file_path ON chunks_metadata(file_path);
CREATE INDEX IF NOT EXISTS idx_section_heading ON chunks_metadata(section_heading);
CREATE INDEX IF NOT EXISTS idx_content_hash ON chunks_metadata(content_hash);
CREATE INDEX IF NOT EXISTS idx_created_at ON chunks_metadata(created_at);

-- Create trigger function for updated_at
CREATE OR REPLACE FUNCTION update_updated_at()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = CURRENT_TIMESTAMP;
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Create trigger
DROP TRIGGER IF EXISTS trigger_update_updated_at ON chunks_metadata;
CREATE TRIGGER trigger_update_updated_at
BEFORE UPDATE ON chunks_metadata
FOR EACH ROW
EXECUTE FUNCTION update_updated_at();
"""


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
    database_url = os.getenv('DATABASE_URL')

    if not database_url:
        print(f"\n❌ Missing required environment variable: DATABASE_URL")
        print("   Please set it in backend/.env or system environment")
        sys.exit(1)

    # Validate DATABASE_URL format
    if 'postgresql://' not in database_url and 'postgres://' not in database_url:
        print(f"\n❌ Invalid DATABASE_URL format")
        print("   Expected: postgresql://user:password@host/database?sslmode=require")
        sys.exit(1)

    # Check if it's still the placeholder
    if 'ep-xxx' in database_url or 'user:password' in database_url:
        print(f"\n❌ DATABASE_URL appears to be a placeholder")
        print("   Please update it with your real Neon Postgres connection string")
        print("   Get it from: https://console.neon.tech/app/projects")
        sys.exit(1)

    return database_url


def create_schema(database_url: str):
    """Create database schema with tables, indexes, and triggers.

    Args:
        database_url: PostgreSQL connection string

    Returns:
        bool: True if successful
    """
    print(f"\n📦 Creating database schema")
    print("=" * 60)

    try:
        # Create engine with psycopg3 driver (not psycopg2)
        # Replace postgresql:// with postgresql+psycopg:// to use psycopg3
        if database_url.startswith('postgresql://'):
            database_url = database_url.replace('postgresql://', 'postgresql+psycopg://', 1)

        engine = create_engine(database_url)

        # Test connection
        with engine.connect() as conn:
            result = conn.execute(text("SELECT version();"))
            version = result.scalar()
            print(f"✓ Connected to PostgreSQL")
            print(f"   Version: {version[:50]}...")

        # Execute schema SQL
        print(f"\n📝 Executing schema creation...")

        with engine.begin() as conn:
            # Execute the entire SQL script
            # Note: text() with multiline SQL works better than splitting by ';'
            try:
                # Execute as raw SQL using connection's native method
                conn.connection.connection.execute(SCHEMA_SQL)
                print(f"✓ All SQL statements executed successfully")
            except Exception as e:
                # Check if error is just about objects already existing
                error_msg = str(e).lower()
                if "already exists" in error_msg or "duplicate" in error_msg:
                    print(f"⚠️  Some objects already exist (this is okay)")
                else:
                    raise

        print(f"✓ Schema created successfully")

        # Verify table exists
        with engine.connect() as conn:
            result = conn.execute(text("""
                SELECT table_name, column_name, data_type
                FROM information_schema.columns
                WHERE table_name = 'chunks_metadata'
                ORDER BY ordinal_position;
            """))

            columns = result.fetchall()

            if columns:
                print(f"\n📊 Table structure:")
                print(f"   Table: chunks_metadata")
                print(f"   Columns: {len(columns)}")
                for col in columns:
                    print(f"      - {col[1]}: {col[2]}")
            else:
                print(f"\n⚠️  Warning: chunks_metadata table not found")
                return False

            # Check indexes
            result = conn.execute(text("""
                SELECT indexname
                FROM pg_indexes
                WHERE tablename = 'chunks_metadata';
            """))

            indexes = result.fetchall()
            print(f"\n   Indexes: {len(indexes)}")
            for idx in indexes:
                print(f"      - {idx[0]}")

            # Check triggers
            result = conn.execute(text("""
                SELECT trigger_name
                FROM information_schema.triggers
                WHERE event_object_table = 'chunks_metadata';
            """))

            triggers = result.fetchall()
            print(f"\n   Triggers: {len(triggers)}")
            for trg in triggers:
                print(f"      - {trg[0]}")

        return True

    except Exception as e:
        print(f"\n❌ Failed to create schema: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main setup function."""
    print("\n" + "=" * 60)
    print("🚀 Neon Postgres Schema Setup")
    print("=" * 60)

    try:
        # Step 1: Load environment
        print("\n[1/2] Loading environment variables...")
        database_url = load_environment()

        # Mask password in URL for logging
        masked_url = database_url.split('@')[1] if '@' in database_url else database_url
        print(f"   Database: {masked_url}")

        # Step 2: Create schema
        print("\n[2/2] Creating database schema...")
        success = create_schema(database_url)

        if success:
            print("\n" + "=" * 60)
            print("✅ Setup complete!")
            print("=" * 60)
            print("\nDatabase ready for indexing.")
            print("\nNext step:")
            print("  Run: python scripts/reindex_book.py --source ../docs")
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
