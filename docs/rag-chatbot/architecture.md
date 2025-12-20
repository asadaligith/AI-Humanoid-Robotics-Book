# RAG Chatbot Architecture

## Overview

The Retrieval-Augmented Generation (RAG) chatbot provides intelligent, context-aware assistance for the AI & Humanoid Robotics course. It combines vector search, PostgreSQL metadata storage, and Claude's language understanding to answer student questions.

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     User Interface                           │
│              (Web Chat Widget + CLI)                         │
└─────────────────────┬────────────────────────────────────────┘
                      │
                      ↓
┌──────────────────────────────────────────────────────────────┐
│                   API Gateway (FastAPI)                      │
│  - Request validation                                        │
│  - Rate limiting                                             │
│  - Authentication                                            │
└─────────────────────┬────────────────────────────────────────┘
                      │
        ┌─────────────┴─────────────┐
        ↓                           ↓
┌───────────────────┐       ┌──────────────────┐
│  Query Processor  │       │ Session Manager  │
│  - Intent detect  │       │ - User history   │
│  - Query rewrite  │       │ - Context cache  │
└─────────┬─────────┘       └──────────────────┘
          │
          ↓
┌──────────────────────────────────────────────────────────────┐
│              Embedding & Retrieval Pipeline                  │
│                                                              │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │  Embeddings  │───→│   Qdrant     │───→│  Reranking   │  │
│  │  (OpenAI)    │    │ Vector Store │    │  (Optional)  │  │
│  └──────────────┘    └──────────────┘    └──────────────┘  │
└─────────────────────┬────────────────────────────────────────┘
                      │
                      ↓
┌──────────────────────────────────────────────────────────────┐
│                PostgreSQL (Neon)                             │
│  - Document metadata                                         │
│  - User sessions                                             │
│  - Feedback tracking                                         │
│  - Analytics                                                 │
└─────────────────────┬────────────────────────────────────────┘
                      │
                      ↓
┌──────────────────────────────────────────────────────────────┐
│              Claude API (Response Generation)                │
│  - Context: Retrieved docs                                   │
│  - System: Course assistant persona                          │
│  - Tools: Code execution, navigation                         │
└─────────────────────┬────────────────────────────────────────┘
                      │
                      ↓
┌──────────────────────────────────────────────────────────────┐
│                     Response                                 │
│  - Formatted answer                                          │
│  - Source citations                                          │
│  - Suggested next steps                                      │
└──────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. Document Ingestion Pipeline

```python
"""
Ingests course content into vector database
"""

class DocumentIngestionPipeline:
    def __init__(self, qdrant_client, postgres_conn, openai_api_key):
        self.qdrant = qdrant_client
        self.db = postgres_conn
        self.embeddings = OpenAIEmbeddings(api_key=openai_api_key)

    def ingest_directory(self, docs_path: str):
        """Ingest all markdown files from documentation."""
        documents = []

        for md_file in Path(docs_path).rglob("*.md"):
            # Parse markdown
            content = md_file.read_text()
            metadata = self.extract_metadata(md_file, content)

            # Chunk document
            chunks = self.chunk_document(content, metadata)
            documents.extend(chunks)

        # Generate embeddings
        embeddings = self.embeddings.embed_documents([d['text'] for d in documents])

        # Store in Qdrant
        self.qdrant.upsert(
            collection_name="course_docs",
            points=[
                {
                    "id": i,
                    "vector": emb,
                    "payload": doc
                }
                for i, (emb, doc) in enumerate(zip(embeddings, documents))
            ]
        )

        # Store metadata in PostgreSQL
        self.db.execute_many(
            "INSERT INTO documents (id, title, url, module, chapter) VALUES (%s, %s, %s, %s, %s)",
            [(d['id'], d['title'], d['url'], d['module'], d['chapter']) for d in documents]
        )

    def chunk_document(self, content: str, metadata: dict, chunk_size=512, overlap=50):
        """Split document into overlapping chunks."""
        chunks = []
        sentences = content.split('\n\n')  # Split by paragraphs

        current_chunk = []
        current_size = 0

        for sentence in sentences:
            tokens = len(sentence.split())

            if current_size + tokens > chunk_size and current_chunk:
                # Save chunk
                chunks.append({
                    'text': '\n\n'.join(current_chunk),
                    **metadata,
                    'chunk_index': len(chunks)
                })

                # Keep overlap
                overlap_size = 0
                overlap_chunk = []
                for s in reversed(current_chunk):
                    overlap_size += len(s.split())
                    overlap_chunk.insert(0, s)
                    if overlap_size >= overlap:
                        break

                current_chunk = overlap_chunk
                current_size = overlap_size

            current_chunk.append(sentence)
            current_size += tokens

        # Add final chunk
        if current_chunk:
            chunks.append({
                'text': '\n\n'.join(current_chunk),
                **metadata,
                'chunk_index': len(chunks)
            })

        return chunks
```

### 2. Query Processing

```python
"""
Processes user queries and retrieves relevant context
"""

class QueryProcessor:
    def __init__(self, qdrant_client, embeddings):
        self.qdrant = qdrant_client
        self.embeddings = embeddings

    def process_query(self, query: str, filters: dict = None, top_k: int = 5):
        """Process query and retrieve relevant documents."""

        # Generate query embedding
        query_embedding = self.embeddings.embed_query(query)

        # Search Qdrant
        search_results = self.qdrant.search(
            collection_name="course_docs",
            query_vector=query_embedding,
            limit=top_k,
            query_filter=self.build_filter(filters) if filters else None
        )

        # Extract contexts
        contexts = []
        for result in search_results:
            contexts.append({
                'text': result.payload['text'],
                'score': result.score,
                'metadata': {
                    'title': result.payload['title'],
                    'url': result.payload['url'],
                    'module': result.payload.get('module'),
                    'chapter': result.payload.get('chapter'),
                }
            })

        return contexts

    def build_filter(self, filters: dict):
        """Build Qdrant filter from dict."""
        conditions = []

        for key, value in filters.items():
            conditions.append({
                "key": key,
                "match": {"value": value}
            })

        return {"must": conditions} if conditions else None
```

### 3. Response Generation

```python
"""
Generates responses using Claude with retrieved context
"""

import anthropic

class ResponseGenerator:
    def __init__(self, api_key: str, model: str = "claude-3-5-sonnet-20241022"):
        self.client = anthropic.Anthropic(api_key=api_key)
        self.model = model

    def generate_response(self, query: str, contexts: list, conversation_history: list = None):
        """Generate response with retrieved context."""

        # Build context string
        context_str = self.format_contexts(contexts)

        # System prompt
        system_prompt = f"""You are an intelligent assistant for the AI & Humanoid Robotics course. Help students learn ROS 2, Gazebo, Isaac Sim, and autonomous robotics.

Retrieved Context:
{context_str}

Guidelines:
- Answer based on the retrieved context when possible
- Cite sources using [Module X, Chapter Y] notation
- Provide code examples when relevant
- Suggest related topics for deeper learning
- If the question is outside the course scope, politely redirect
- Be encouraging and educational"""

        # Build message history
        messages = []
        if conversation_history:
            messages.extend(conversation_history)

        messages.append({
            "role": "user",
            "content": query
        })

        # Generate response
        response = self.client.messages.create(
            model=self.model,
            max_tokens=2048,
            system=system_prompt,
            messages=messages
        )

        answer = response.content[0].text

        # Add source citations
        sources = self.extract_sources(contexts)

        return {
            'answer': answer,
            'sources': sources,
            'contexts_used': len(contexts)
        }

    def format_contexts(self, contexts: list) -> str:
        """Format retrieved contexts for prompt."""
        formatted = []

        for i, ctx in enumerate(contexts, 1):
            meta = ctx['metadata']
            formatted.append(
                f"[Source {i}] {meta.get('title', 'Unknown')}\n"
                f"Module: {meta.get('module', 'N/A')}, Chapter: {meta.get('chapter', 'N/A')}\n"
                f"Content: {ctx['text']}\n"
            )

        return "\n".join(formatted)

    def extract_sources(self, contexts: list) -> list:
        """Extract source references."""
        sources = []

        for ctx in contexts:
            meta = ctx['metadata']
            sources.append({
                'title': meta.get('title'),
                'url': meta.get('url'),
                'module': meta.get('module'),
                'chapter': meta.get('chapter'),
                'relevance_score': ctx['score']
            })

        return sources
```

### 4. FastAPI Application

```python
"""
API endpoints for chatbot
"""

from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
import uvicorn

app = FastAPI(title="RAG Chatbot API")

class ChatRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    filters: Optional[dict] = None
    top_k: int = 5

class ChatResponse(BaseModel):
    answer: str
    sources: List[dict]
    session_id: str

class Chatbot:
    def __init__(self):
        self.query_processor = QueryProcessor(qdrant_client, embeddings)
        self.response_generator = ResponseGenerator(claude_api_key)
        self.session_manager = SessionManager(db_connection)

    async def process_chat(self, request: ChatRequest) -> ChatResponse:
        """Process chat request."""

        # Get or create session
        session = self.session_manager.get_or_create(request.session_id)

        # Retrieve context
        contexts = self.query_processor.process_query(
            request.query,
            filters=request.filters,
            top_k=request.top_k
        )

        # Generate response
        result = self.response_generator.generate_response(
            request.query,
            contexts,
            conversation_history=session.history
        )

        # Update session
        session.add_exchange(request.query, result['answer'])

        # Log to database
        self.log_interaction(session.id, request.query, result)

        return ChatResponse(
            answer=result['answer'],
            sources=result['sources'],
            session_id=session.id
        )

chatbot = Chatbot()

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Chat endpoint."""
    try:
        return await chatbot.process_chat(request)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health():
    """Health check."""
    return {"status": "healthy"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

## Database Schema (PostgreSQL)

```sql
-- Documents metadata
CREATE TABLE documents (
    id SERIAL PRIMARY KEY,
    title VARCHAR(255) NOT NULL,
    url VARCHAR(512),
    module VARCHAR(50),
    chapter VARCHAR(50),
    content_hash VARCHAR(64) UNIQUE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Chat sessions
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(100),
    started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_activity TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    message_count INTEGER DEFAULT 0
);

-- Chat messages
CREATE TABLE chat_messages (
    id SERIAL PRIMARY KEY,
    session_id UUID REFERENCES chat_sessions(id),
    role VARCHAR(20) NOT NULL,  -- 'user' or 'assistant'
    content TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Retrieved documents (for analytics)
CREATE TABLE retrieved_documents (
    id SERIAL PRIMARY KEY,
    session_id UUID REFERENCES chat_sessions(id),
    document_id INTEGER REFERENCES documents(id),
    relevance_score FLOAT,
    query TEXT,
    retrieved_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User feedback
CREATE TABLE feedback (
    id SERIAL PRIMARY KEY,
    session_id UUID REFERENCES chat_sessions(id),
    message_id INTEGER,
    rating INTEGER CHECK (rating BETWEEN 1 AND 5),
    comment TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes
CREATE INDEX idx_sessions_user ON chat_sessions(user_id);
CREATE INDEX idx_messages_session ON chat_messages(session_id);
CREATE INDEX idx_documents_module ON documents(module);
CREATE INDEX idx_retrieved_query ON retrieved_documents(query);
```

## Qdrant Collection Schema

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

# Initialize client
client = QdrantClient(url="https://your-cluster.qdrant.io", api_key="your-key")

# Create collection
client.create_collection(
    collection_name="course_docs",
    vectors_config=VectorParams(
        size=1536,  # OpenAI text-embedding-3-small dimension
        distance=Distance.COSINE
    )
)

# Create payload indexes for filtering
client.create_payload_index(
    collection_name="course_docs",
    field_name="module",
    field_schema="keyword"
)

client.create_payload_index(
    collection_name="course_docs",
    field_name="chapter",
    field_schema="keyword"
)
```

## Performance Considerations

### 1. Caching Strategy
- Cache embeddings for frequently asked questions
- Cache Claude responses for identical queries
- Use Redis for session management

### 2. Rate Limiting
- 10 requests/minute per user
- 1000 requests/day per API key
- Exponential backoff on API failures

### 3. Cost Optimization
- Use text-embedding-3-small ($0.02/1M tokens) instead of ada-002
- Cache common query embeddings
- Batch document processing during ingestion
- Use Claude 3.5 Sonnet (not Opus) for most queries

### 4. Scalability
- Horizontal scaling with load balancer
- Database connection pooling
- Async I/O for API calls
- Qdrant cluster for large document collections

## Security

1. **API Key Management**: Store in environment variables, never in code
2. **Input Validation**: Sanitize user queries, prevent prompt injection
3. **Rate Limiting**: Prevent abuse and DoS attacks
4. **Authentication**: Optional user authentication for personalized experience
5. **Data Privacy**: Anonymize logs, respect user data preferences

## Monitoring & Analytics

```python
# Track key metrics
class Analytics:
    def track_query(self, session_id, query, response_time, contexts_count):
        """Track query metrics."""
        self.db.execute(
            """
            INSERT INTO query_analytics
            (session_id, query, response_time_ms, contexts_retrieved, timestamp)
            VALUES (%s, %s, %s, %s, NOW())
            """,
            (session_id, query, response_time * 1000, contexts_count)
        )

    def get_metrics(self, days=7):
        """Get analytics for last N days."""
        return self.db.query(
            """
            SELECT
                DATE(timestamp) as date,
                COUNT(*) as total_queries,
                AVG(response_time_ms) as avg_response_time,
                AVG(contexts_retrieved) as avg_contexts
            FROM query_analytics
            WHERE timestamp >= NOW() - INTERVAL '%s days'
            GROUP BY DATE(timestamp)
            ORDER BY date DESC
            """,
            (days,)
        )
```

## Deployment

See [deployment.md](deployment.md) for production deployment instructions.

## Next Steps

1. **Setup**: Follow [setup.md](setup.md) for local development
2. **Integration**: See [integration.md](integration.md) for embedding in Docusaurus
3. **Customization**: Review [customization.md](customization.md) for branding

## References

- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Neon PostgreSQL](https://neon.tech/docs)
- [Anthropic Claude API](https://docs.anthropic.com/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
