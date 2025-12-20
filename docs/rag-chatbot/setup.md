# RAG Chatbot Setup Guide

## Prerequisites

- Python 3.9+
- Node.js 18+ (for Docusaurus integration)
- Git
- API Keys:
  - Anthropic Claude API
  - OpenAI API (for embeddings)
  - Qdrant Cloud account
  - Neon PostgreSQL account

## Quick Start

```bash
# Clone repository
git clone https://github.com/asadaligith/AI-Humanoid-Robotics-Book.git
cd AI-Humanoid-Robotics-Book

# Create chatbot directory
mkdir -p chatbot
cd chatbot

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

## Requirements File

Create `chatbot/requirements.txt`:

```txt
# Core dependencies
fastapi==0.104.1
uvicorn[standard]==0.24.0
anthropic==0.7.7
openai==1.3.7
qdrant-client==1.7.0
psycopg2-binary==2.9.9

# Data processing
langchain==0.0.340
python-dotenv==1.0.0
pydantic==2.5.0

# Utilities
redis==5.0.1
tenacity==8.2.3
markdown==3.5.1
python-multipart==0.0.6

# Development
pytest==7.4.3
pytest-asyncio==0.21.1
black==23.11.0
```

## Environment Configuration

Create `.env` file:

```bash
# API Keys
ANTHROPIC_API_KEY=sk-ant-api03-...
OPENAI_API_KEY=sk-...

# Qdrant
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=course_docs

# PostgreSQL (Neon)
DATABASE_URL=postgresql://user:password@your-project.neon.tech/chatbot?sslmode=require
DB_POOL_SIZE=10
DB_MAX_OVERFLOW=20

# Redis (Optional)
REDIS_URL=redis://localhost:6379/0

# Application
APP_ENV=development
APP_PORT=8000
APP_HOST=0.0.0.0
LOG_LEVEL=INFO

# Security
API_RATE_LIMIT=10  # requests per minute
MAX_CONTEXT_LENGTH=8000
MAX_RESPONSE_TOKENS=2048
```

## Database Setup

### 1. Create Neon PostgreSQL Database

```bash
# Sign up at https://neon.tech
# Create new project: "chatbot"
# Copy connection string

# Initialize database
python scripts/init_database.py
```

`scripts/init_database.py`:

```python
#!/usr/bin/env python3
import psycopg2
import os
from dotenv import load_dotenv

load_dotenv()

# Connect to database
conn = psycopg2.connect(os.getenv('DATABASE_URL'))
cur = conn.cursor()

# Create tables
with open('schema.sql', 'r') as f:
    cur.execute(f.read())

conn.commit()
cur.close()
conn.close()

print("Database initialized successfully!")
```

`schema.sql`:

```sql
-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Documents table
CREATE TABLE IF NOT EXISTS documents (
    id SERIAL PRIMARY KEY,
    title VARCHAR(255) NOT NULL,
    url VARCHAR(512),
    module VARCHAR(50),
    chapter VARCHAR(50),
    content_hash VARCHAR(64) UNIQUE,
    chunk_count INTEGER DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Chat sessions
CREATE TABLE IF NOT EXISTS chat_sessions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id VARCHAR(100),
    started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_activity TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    message_count INTEGER DEFAULT 0,
    metadata JSONB
);

-- Chat messages
CREATE TABLE IF NOT EXISTS chat_messages (
    id SERIAL PRIMARY KEY,
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    tokens_used INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Retrieved documents
CREATE TABLE IF NOT EXISTS retrieved_documents (
    id SERIAL PRIMARY KEY,
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    document_id INTEGER REFERENCES documents(id),
    relevance_score FLOAT,
    query TEXT,
    rank INTEGER,
    retrieved_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User feedback
CREATE TABLE IF NOT EXISTS feedback (
    id SERIAL PRIMARY KEY,
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    message_id INTEGER,
    rating INTEGER CHECK (rating BETWEEN 1 AND 5),
    comment TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Query analytics
CREATE TABLE IF NOT EXISTS query_analytics (
    id SERIAL PRIMARY KEY,
    session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    query TEXT NOT NULL,
    response_time_ms INTEGER,
    contexts_retrieved INTEGER,
    model_used VARCHAR(50),
    tokens_used INTEGER,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes
CREATE INDEX IF NOT EXISTS idx_sessions_user ON chat_sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_sessions_activity ON chat_sessions(last_activity DESC);
CREATE INDEX IF NOT EXISTS idx_messages_session ON chat_messages(session_id);
CREATE INDEX IF NOT EXISTS idx_documents_module ON documents(module);
CREATE INDEX IF NOT EXISTS idx_documents_hash ON documents(content_hash);
CREATE INDEX IF NOT EXISTS idx_retrieved_query ON retrieved_documents(query);
CREATE INDEX IF NOT EXISTS idx_analytics_timestamp ON query_analytics(timestamp DESC);

-- Function to update last_activity
CREATE OR REPLACE FUNCTION update_session_activity()
RETURNS TRIGGER AS $$
BEGIN
    UPDATE chat_sessions
    SET last_activity = CURRENT_TIMESTAMP,
        message_count = message_count + 1
    WHERE id = NEW.session_id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Trigger to auto-update session activity
CREATE TRIGGER update_session_activity_trigger
AFTER INSERT ON chat_messages
FOR EACH ROW
EXECUTE FUNCTION update_session_activity();
```

### 2. Setup Qdrant Vector Database

```bash
# Sign up at https://cloud.qdrant.io
# Create cluster: "chatbot-vectors"
# Copy API key and URL

# Initialize collection
python scripts/init_qdrant.py
```

`scripts/init_qdrant.py`:

```python
#!/usr/bin/env python3
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import os
from dotenv import load_dotenv

load_dotenv()

# Connect to Qdrant
client = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

collection_name = os.getenv('QDRANT_COLLECTION', 'course_docs')

# Create collection
client.recreate_collection(
    collection_name=collection_name,
    vectors_config=VectorParams(
        size=1536,  # OpenAI text-embedding-3-small
        distance=Distance.COSINE
    )
)

# Create payload indexes
client.create_payload_index(
    collection_name=collection_name,
    field_name="module",
    field_schema="keyword"
)

client.create_payload_index(
    collection_name=collection_name,
    field_name="chapter",
    field_schema="keyword"
)

client.create_payload_index(
    collection_name=collection_name,
    field_name="title",
    field_schema="text"
)

print(f"Qdrant collection '{collection_name}' created successfully!")
```

## Ingesting Documentation

```bash
# Ingest all course documentation
python scripts/ingest_docs.py --docs-path ../docs

# Ingest specific module
python scripts/ingest_docs.py --docs-path ../docs/modules/module-01-ros2-fundamentals

# Re-index (clears and re-ingests)
python scripts/ingest_docs.py --docs-path ../docs --reindex
```

`scripts/ingest_docs.py`:

```python
#!/usr/bin/env python3
"""
Ingest course documentation into Qdrant and PostgreSQL
"""
import argparse
from pathlib import Path
import hashlib
from typing import List, Dict
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from openai import OpenAI
import psycopg2
import os
from dotenv import load_dotenv
import re

load_dotenv()

class DocumentIngester:
    def __init__(self):
        # Initialize clients
        self.qdrant = QdrantClient(
            url=os.getenv('QDRANT_URL'),
            api_key=os.getenv('QDRANT_API_KEY')
        )
        self.openai = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        self.db = psycopg2.connect(os.getenv('DATABASE_URL'))
        self.collection_name = os.getenv('QDRANT_COLLECTION', 'course_docs')

    def ingest_directory(self, docs_path: str, reindex: bool = False):
        """Ingest all markdown files."""
        docs_path = Path(docs_path)

        if reindex:
            print("Clearing existing data...")
            self.clear_data()

        print(f"Scanning {docs_path}...")
        md_files = list(docs_path.rglob("*.md"))
        print(f"Found {len(md_files)} markdown files")

        total_chunks = 0

        for md_file in md_files:
            print(f"\nProcessing: {md_file.name}")
            chunks = self.process_file(md_file)
            total_chunks += len(chunks)
            print(f"  Created {len(chunks)} chunks")

        print(f"\n✓ Ingestion complete! Total chunks: {total_chunks}")

    def process_file(self, file_path: Path) -> List[Dict]:
        """Process a single markdown file."""
        content = file_path.read_text(encoding='utf-8')

        # Extract metadata
        metadata = self.extract_metadata(file_path, content)

        # Check if already ingested
        content_hash = hashlib.sha256(content.encode()).hexdigest()
        if self.is_already_ingested(content_hash):
            print(f"  Skipping (already ingested)")
            return []

        # Chunk document
        chunks = self.chunk_document(content, metadata)

        # Generate embeddings
        texts = [chunk['text'] for chunk in chunks]
        embeddings = self.generate_embeddings(texts)

        # Store in Qdrant
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point_id = hash(f"{content_hash}_{i}") % (10 ** 10)
            points.append(
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=chunk
                )
            )

        self.qdrant.upsert(
            collection_name=self.collection_name,
            points=points
        )

        # Store metadata in PostgreSQL
        self.store_metadata(metadata, content_hash, len(chunks))

        return chunks

    def extract_metadata(self, file_path: Path, content: str) -> Dict:
        """Extract metadata from file path and content."""
        # Parse file path
        parts = file_path.parts
        module = None
        chapter = None

        for part in parts:
            if 'module-' in part:
                module = part
            if 'chapter-' in part:
                chapter = part

        # Extract title (first # heading)
        title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        title = title_match.group(1) if title_match else file_path.stem

        # Build URL
        rel_path = file_path.relative_to(file_path.parents[2])  # Relative to docs/
        url = f"/docs/{str(rel_path).replace('.md', '').replace(os.sep, '/')}"

        return {
            'title': title,
            'url': url,
            'module': module,
            'chapter': chapter,
            'file_path': str(file_path)
        }

    def chunk_document(self, content: str, metadata: Dict, chunk_size: int = 512, overlap: int = 50) -> List[Dict]:
        """Split document into chunks."""
        chunks = []

        # Split by sections (## headings)
        sections = re.split(r'\n##\s+', content)

        for section in sections:
            if not section.strip():
                continue

            # Split section into paragraphs
            paragraphs = section.split('\n\n')

            current_chunk = []
            current_tokens = 0

            for para in paragraphs:
                para_tokens = len(para.split())

                if current_tokens + para_tokens > chunk_size and current_chunk:
                    # Save chunk
                    chunks.append({
                        'text': '\n\n'.join(current_chunk),
                        **metadata,
                        'chunk_index': len(chunks)
                    })

                    # Keep overlap
                    if len(current_chunk) > 1:
                        current_chunk = current_chunk[-1:]
                        current_tokens = len(current_chunk[0].split())
                    else:
                        current_chunk = []
                        current_tokens = 0

                current_chunk.append(para)
                current_tokens += para_tokens

            # Add final chunk
            if current_chunk:
                chunks.append({
                    'text': '\n\n'.join(current_chunk),
                    **metadata,
                    'chunk_index': len(chunks)
                })

        return chunks

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings using OpenAI."""
        response = self.openai.embeddings.create(
            model="text-embedding-3-small",
            input=texts
        )
        return [item.embedding for item in response.data]

    def is_already_ingested(self, content_hash: str) -> bool:
        """Check if document already ingested."""
        cur = self.db.cursor()
        cur.execute(
            "SELECT COUNT(*) FROM documents WHERE content_hash = %s",
            (content_hash,)
        )
        count = cur.fetchone()[0]
        cur.close()
        return count > 0

    def store_metadata(self, metadata: Dict, content_hash: str, chunk_count: int):
        """Store document metadata in PostgreSQL."""
        cur = self.db.cursor()
        cur.execute(
            """
            INSERT INTO documents (title, url, module, chapter, content_hash, chunk_count)
            VALUES (%s, %s, %s, %s, %s, %s)
            ON CONFLICT (content_hash) DO UPDATE
            SET updated_at = CURRENT_TIMESTAMP
            """,
            (
                metadata['title'],
                metadata['url'],
                metadata['module'],
                metadata['chapter'],
                content_hash,
                chunk_count
            )
        )
        self.db.commit()
        cur.close()

    def clear_data(self):
        """Clear all data (for reindexing)."""
        # Clear Qdrant
        self.qdrant.delete_collection(self.collection_name)
        self.qdrant.create_collection(
            collection_name=self.collection_name,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
        )

        # Clear PostgreSQL
        cur = self.db.cursor()
        cur.execute("TRUNCATE TABLE documents RESTART IDENTITY CASCADE")
        self.db.commit()
        cur.close()

def main():
    parser = argparse.ArgumentParser(description='Ingest documentation')
    parser.add_argument('--docs-path', required=True, help='Path to docs directory')
    parser.add_argument('--reindex', action='store_true', help='Clear and reindex')

    args = parser.parse_args()

    ingester = DocumentIngester()
    ingester.ingest_directory(args.docs_path, args.reindex)

if __name__ == '__main__':
    main()
```

## Running the API

```bash
# Development mode (auto-reload)
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Production mode
uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
```

## Testing the API

```bash
# Health check
curl http://localhost:8000/health

# Chat request
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do I create a ROS 2 publisher?",
    "top_k": 5
  }'

# With session
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What about subscribers?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000"
  }'
```

## Docker Deployment

```dockerfile
# Dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY app/ ./app/
COPY .env .

# Expose port
EXPOSE 8000

# Run application
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

```yaml
# docker-compose.yml
version: '3.8'

services:
  chatbot-api:
    build: .
    ports:
      - "8000:8000"
    environment:
      - ANTHROPIC_API_KEY=${ANTHROPIC_API_KEY}
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - DATABASE_URL=${DATABASE_URL}
    restart: unless-stopped

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    restart: unless-stopped
```

## Next Steps

1. **Test locally**: Ingest documentation and test queries
2. **Deploy**: Follow [deployment.md](deployment.md) for production
3. **Integrate**: Add chat widget to Docusaurus (see [integration.md](integration.md))
4. **Monitor**: Set up analytics and logging

## Troubleshooting

### Issue: Database connection fails
```bash
# Test connection
python -c "import psycopg2; psycopg2.connect('$DATABASE_URL'); print('✓ Connected')"
```

### Issue: Qdrant connection timeout
```bash
# Verify API key and URL
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='$QDRANT_URL', api_key='$QDRANT_API_KEY'); print(client.get_collections())"
```

### Issue: Embeddings failing
```bash
# Test OpenAI API
python -c "from openai import OpenAI; client = OpenAI(); print(client.embeddings.create(model='text-embedding-3-small', input=['test']))"
```

## References

- [Qdrant Setup](https://qdrant.tech/documentation/quick-start/)
- [Neon PostgreSQL](https://neon.tech/docs/get-started-with-neon/signing-up)
- [FastAPI Deployment](https://fastapi.tiangolo.com/deployment/)
