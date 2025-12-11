"""Reindex book content into Qdrant and Postgres.

Reads markdown files from /docs/**, chunks, embeds with Gemini, and stores.
"""

import sys
import os
import argparse
from pathlib import Path
from typing import List, Dict
import time

# Fix Windows console encoding for emoji support
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.services.embeddings import embed_document, embed_batch
from src.services.retrieval import upsert_chunk, get_collection_info
from src.services.database import ChunkMetadata, get_db
from src.utils.chunking import chunk_text, count_tokens, validate_chunk
from src.utils.hashing import compute_hash
from uuid import uuid4


def read_markdown_files(source_dir: str) -> List[Dict]:
    """Read all markdown files from source directory.

    Args:
        source_dir: Path to /docs/ directory

    Returns:
        List of dicts with file_path and content
    """
    print(f"📂 Reading markdown files from: {source_dir}")

    source_path = Path(source_dir)
    if not source_path.exists():
        raise ValueError(f"Source directory not found: {source_dir}")

    markdown_files = []

    # Find all .md and .mdx files
    for pattern in ["**/*.md", "**/*.mdx"]:
        for file_path in source_path.glob(pattern):
            if file_path.is_file():
                try:
                    content = file_path.read_text(encoding="utf-8")

                    # Get relative path from source directory
                    relative_path = "/" + str(file_path.relative_to(source_path)).replace(
                        "\\", "/"
                    )

                    markdown_files.append(
                        {"file_path": relative_path, "content": content}
                    )

                    print(f"   ✓ {relative_path}")

                except Exception as e:
                    print(f"   ✗ Error reading {file_path}: {e}")

    print(f"📊 Found {len(markdown_files)} markdown files\n")
    return markdown_files


def process_files(
    markdown_files: List[Dict], dry_run: bool = False
) -> Dict[str, int]:
    """Process markdown files: chunk, hash, embed, and store.

    Args:
        markdown_files: List of file dicts
        dry_run: If True, don't actually insert into databases

    Returns:
        Statistics dict
    """
    stats = {
        "total_files": len(markdown_files),
        "total_chunks": 0,
        "new_chunks": 0,
        "updated_chunks": 0,
        "skipped_chunks": 0,
        "errors": 0,
    }

    print("🔄 Processing files...")

    with get_db() as db:
        for file_idx, file_data in enumerate(markdown_files, 1):
            file_path = file_data["file_path"]
            content = file_data["content"]

            print(
                f"\n[{file_idx}/{stats['total_files']}] Processing: {file_path}"
            )

            try:
                # Step 1: Chunk the file
                chunks = chunk_text(content, file_path)
                print(f"   📝 Generated {len(chunks)} chunks")

                if not chunks:
                    print(f"   ⚠️  No chunks generated (file too small?)")
                    continue

                stats["total_chunks"] += len(chunks)

                # Step 2: Process each chunk
                for chunk_idx, chunk_data in enumerate(chunks):
                    # Validate chunk
                    if not validate_chunk(chunk_data):
                        print(
                            f"   ✗ Chunk {chunk_idx} failed validation, skipping"
                        )
                        stats["errors"] += 1
                        continue

                    # Compute hash
                    text_content = chunk_data["chunk_text"]
                    content_hash = compute_hash(text_content)

                    # Check if chunk already exists
                    existing = (
                        db.query(ChunkMetadata)
                        .filter(ChunkMetadata.content_hash == content_hash)
                        .first()
                    )

                    if existing:
                        # Chunk exists, skip
                        stats["skipped_chunks"] += 1
                        continue

                    # Chunk is new or updated
                    chunk_id = str(uuid4())

                    if dry_run:
                        print(
                            f"   🔍 [DRY RUN] Would create chunk {chunk_idx}: {text_content[:50]}..."
                        )
                        stats["new_chunks"] += 1
                        continue

                    # Step 3: Generate embedding
                    try:
                        embedding = embed_document(text_content)
                    except Exception as e:
                        print(f"   ✗ Error embedding chunk {chunk_idx}: {e}")
                        stats["errors"] += 1
                        continue

                    # Step 4: Upsert to Qdrant
                    payload = {
                        "chunk_id": chunk_id,
                        "file_path": file_path,
                        "section_heading": chunk_data["section_heading"],
                        "chunk_index": chunk_data["chunk_index"],
                        "content_text": text_content,
                        "content_hash": content_hash,
                        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
                    }

                    try:
                        upsert_chunk(chunk_id, embedding, payload)
                    except Exception as e:
                        print(f"   ✗ Error upserting to Qdrant: {e}")
                        stats["errors"] += 1
                        continue

                    # Step 5: Insert to Postgres
                    try:
                        metadata = ChunkMetadata(
                            chunk_id=chunk_id,
                            file_path=file_path,
                            section_heading=chunk_data["section_heading"],
                            chunk_index=chunk_data["chunk_index"],
                            content_hash=content_hash,
                            token_count=chunk_data["token_count"],
                        )
                        db.add(metadata)
                        db.commit()

                        stats["new_chunks"] += 1
                        print(
                            f"   ✓ Indexed chunk {chunk_idx}: {text_content[:50]}..."
                        )

                    except Exception as e:
                        db.rollback()
                        print(f"   ✗ Error inserting to Postgres: {e}")
                        stats["errors"] += 1
                        continue

                    # Rate limiting: brief pause to avoid hitting API limits
                    time.sleep(0.05)  # 50ms between chunks

            except Exception as e:
                print(f"   ✗ Error processing file: {e}")
                import traceback
                traceback.print_exc()
                stats["errors"] += 1

    return stats


def print_statistics(stats: Dict[str, int]):
    """Print indexing statistics."""
    print("\n" + "=" * 60)
    print("📊 Indexing Statistics")
    print("=" * 60)

    print(f"Files processed:      {stats['total_files']}")
    print(f"Total chunks:         {stats['total_chunks']}")
    print(f"New chunks indexed:   {stats['new_chunks']}")
    print(f"Updated chunks:       {stats['updated_chunks']}")
    print(f"Skipped (unchanged):  {stats['skipped_chunks']}")
    print(f"Errors:               {stats['errors']}")

    # Get collection info
    try:
        info = get_collection_info()
        print(f"\nQdrant collection:")
        print(f"  Points count:       {info.get('points_count', 0)}")
        print(f"  Vectors count:      {info.get('vectors_count', 0)}")
    except Exception as e:
        print(f"\n⚠️  Could not get Qdrant stats: {e}")


def main():
    """Main indexing function."""
    parser = argparse.ArgumentParser(
        description="Reindex book content into Qdrant and Postgres"
    )

    parser.add_argument(
        "--source",
        type=str,
        default="../docs",
        help="Source directory containing markdown files (default: ../docs)",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Perform a dry run without actually indexing",
    )

    parser.add_argument(
        "--force",
        action="store_true",
        help="Force re-index all chunks (delete existing first)",
    )

    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("🚀 Book Content Indexing")
    print("=" * 60)
    print(f"Source directory: {args.source}")
    print(f"Dry run:          {args.dry_run}")
    print(f"Force re-index:   {args.force}\n")

    if args.force:
        print("⚠️  Force re-index not implemented yet.")
        print("    To re-index, manually delete Qdrant collection and Postgres rows.\n")

    try:
        # Step 1: Read files
        markdown_files = read_markdown_files(args.source)

        if not markdown_files:
            print("❌ No markdown files found!")
            sys.exit(1)

        # Step 2: Process and index
        stats = process_files(markdown_files, dry_run=args.dry_run)

        # Step 3: Print results
        print_statistics(stats)

        if args.dry_run:
            print("\n🔍 This was a dry run. No changes were made.")
        else:
            print("\n✅ Indexing complete!")

        if stats["errors"] > 0:
            print(
                f"\n⚠️  Completed with {stats['errors']} errors. Check logs above."
            )
            sys.exit(1)

    except Exception as e:
        print(f"\n❌ Indexing failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
