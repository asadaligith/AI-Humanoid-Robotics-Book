# Data Model: RAG Chatbot

**Feature**: 001-rag-chatbot
**Date**: 2025-12-09
**Status**: Complete

## Overview

This document defines the data entities, relationships, and schemas for the RAG chatbot system. The system uses two primary data stores: Qdrant (vector database) for embeddings and Neon Postgres (relational database) for structured metadata.

---

## Entity Relationship Diagram (Conceptual)

```
┌─────────────────┐
│  Markdown File  │
└────────┬────────┘
         │ 1:N
         ▼
┌─────────────────┐        ┌──────────────────┐
│  Document Chunk │◄──────►│ Chunks Metadata  │
│   (Qdrant)      │ 1:1    │   (Postgres)     │
└────────┬────────┘        └──────────────────┘
         │
         │ Uses
         ▼
┌─────────────────┐
│  Embedding      │
│  Vector (1536D) │
└─────────────────┘
         │
         │ Retrieved by
         ▼
┌─────────────────┐        ┌──────────────────┐
│  Search Query   │───────►│ Retrieval Result │
└─────────────────┘        └────────┬─────────┘
                                    │
                                    │ Generates
                                    ▼
                           ┌──────────────────┐
                           │   Chat Response  │
                           └──────────────────┘
```

---

## Entity Definitions

### 1. Document Chunk (Qdrant)

**Description**: A segment of book content with its embedding vector and metadata.

**Storage**: Qdrant collection `book_content`

**Attributes**:

| Field | Type | Description | Indexed | Required |
|-------|------|-------------|---------|----------|
| `id` | UUID (Qdrant Point ID) | Unique chunk identifier | Yes (Primary) | Yes |
| `vector` | Float[1536] | Embedding from text-embedding-3-large | Yes (HNSW) | Yes |
| `payload.chunk_id` | UUID | Same as id (for reference) | Yes | Yes |
| `payload.file_path` | String (max 512) | Relative path from /docs/ | Yes | Yes |
| `payload.section_heading` | String (max 512) | Section/chapter heading | Yes | No |
| `payload.chunk_index` | Integer | Position within document (0-indexed) | No | Yes |
| `payload.content_text` | String (max 8000) | Original chunk text | No | Yes |
| `payload.created_at` | ISO-8601 Timestamp | Indexing timestamp | No | Yes |
| `payload.content_hash` | String (64 chars) | SHA256 hash for deduplication | Yes | Yes |

**Indexes**:
- HNSW index on `vector` (M=16, ef_construct=100) for fast similarity search
- Payload indexes on: `chunk_id`, `file_path`, `section_heading`, `content_hash`

**Validation Rules**:
- `chunk_id` must be valid UUID v4
- `file_path` must start with `/` and end with `.md` or `.mdx`
- `section_heading` extracted from markdown headers (H1-H3)
- `chunk_index` must be >= 0
- `content_text` length: 100-2000 tokens (enforced at ingestion)
- `content_hash` must be unique (prevents duplicates)

**Relationships**:
- 1:1 with `chunks_metadata` table (Postgres) via `chunk_id`
- N:1 with source markdown file

---

### 2. Chunks Metadata (Postgres)

**Description**: Structured metadata for each chunk, enabling relational queries and auditing.

**Storage**: Neon Postgres table `chunks_metadata`

**Schema**:

```sql
CREATE TABLE chunks_metadata (
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

CREATE INDEX idx_file_path ON chunks_metadata(file_path);
CREATE INDEX idx_section_heading ON chunks_metadata(section_heading);
CREATE INDEX idx_content_hash ON chunks_metadata(content_hash);
CREATE INDEX idx_created_at ON chunks_metadata(created_at);
```

**Attributes**:

| Field | Type | Description | Constraints | Required |
|-------|------|-------------|-------------|----------|
| `chunk_id` | UUID | Primary key, matches Qdrant point ID | PRIMARY KEY | Yes |
| `file_path` | VARCHAR(512) | Relative markdown file path | NOT NULL | Yes |
| `section_heading` | VARCHAR(512) | Section/chapter heading | Nullable | No |
| `chunk_index` | INTEGER | Position within document | >=0, NOT NULL | Yes |
| `content_hash` | CHAR(64) | SHA256 hash (hex string) | UNIQUE, NOT NULL | Yes |
| `token_count` | INTEGER | Number of tokens in chunk | 100-2000, NOT NULL | Yes |
| `created_at` | TIMESTAMP | Creation timestamp | DEFAULT NOW() | Yes |
| `updated_at` | TIMESTAMP | Last update timestamp | DEFAULT NOW() | Yes |

**Validation Rules**:
- `chunk_id` must match Qdrant point ID (enforced at application level)
- `file_path` must be valid relative path
- `chunk_index` increments within same `file_path`
- `content_hash` computed as `SHA256(content_text)` (ensures uniqueness)
- `token_count` counted using tiktoken (cl100k_base encoding)

**Triggers**:

```sql
CREATE OR REPLACE FUNCTION update_updated_at()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = CURRENT_TIMESTAMP;
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trigger_update_updated_at
BEFORE UPDATE ON chunks_metadata
FOR EACH ROW
EXECUTE FUNCTION update_updated_at();
```

**Relationships**:
- 1:1 with `Document Chunk` (Qdrant) via `chunk_id`
- N:1 with source markdown file

---

### 3. Search Query (Application Layer)

**Description**: User's question submitted to the chatbot (ephemeral, not persisted).

**Storage**: In-memory during request lifecycle

**Attributes**:

| Field | Type | Description | Required |
|-------|------|-------------|----------|
| `question` | String (max 2000) | User's question | Yes |
| `selected_text` | String (max 5000) | Optional selected text context | No |
| `session_id` | UUID | Session identifier for multi-turn context | No |
| `timestamp` | ISO-8601 Timestamp | Query submission time | Yes |
| `query_embedding` | Float[1536] | Embedding of the question | Yes (computed) |

**Validation Rules**:
- `question` length: 5-2000 characters
- `selected_text` length: 0-5000 characters
- `session_id` must be valid UUID v4 (generated if not provided)
- `query_embedding` computed via OpenAI text-embedding-3-large

**Lifecycle**:
- Created on `/ask` or `/ask-selected` request
- Embedding generated
- Used for Qdrant retrieval
- Discarded after response (not stored)

---

### 4. Retrieval Result (Application Layer)

**Description**: Output from Qdrant semantic search (ephemeral).

**Storage**: In-memory during request lifecycle

**Attributes**:

| Field | Type | Description | Required |
|-------|------|-------------|----------|
| `chunk_id` | UUID | Reference to retrieved chunk | Yes |
| `file_path` | String | Source markdown file | Yes |
| `section_heading` | String | Section/chapter heading | No |
| `content_text` | String | Chunk text | Yes |
| `similarity_score` | Float (0-1) | Cosine similarity score | Yes |
| `chunk_index` | Integer | Position within document | Yes |

**Validation Rules**:
- `similarity_score` must be >= 0.7 (threshold for relevance)
- Results sorted by `similarity_score` descending
- Maximum 5 results passed to agent (after reranking)

**Lifecycle**:
- Created from Qdrant search response
- Filtered by similarity threshold
- Reranked by relevance
- Passed to OpenAI Agent as tool context
- Discarded after response generation

---

### 5. Chat Response (Application Layer)

**Description**: Final answer generated by the OpenAI Agent (ephemeral, optionally logged).

**Storage**: In-memory; optionally logged to file for debugging

**Attributes**:

| Field | Type | Description | Required |
|-------|------|-------------|----------|
| `answer` | String (max 4000) | Agent-generated answer | Yes |
| `sources` | Array[Source] | Citations for the answer | Yes |
| `session_id` | UUID | Session identifier | Yes |
| `confidence` | Float (0-1) | Agent's confidence score | No |
| `timestamp` | ISO-8601 Timestamp | Response generation time | Yes |
| `latency_ms` | Integer | Total response time (ms) | Yes |

**Source Object**:

| Field | Type | Description | Required |
|-------|------|-------------|----------|
| `file` | String | Source markdown file path | Yes |
| `section` | String | Section heading | No |
| `chunk` | String | Excerpt from chunk (max 200 chars) | Yes |
| `similarity` | Float (0-1) | Relevance score | Yes |

**Validation Rules**:
- `answer` must not be empty
- `sources` array must contain 1-5 elements
- Each source must reference a valid retrieved chunk
- `latency_ms` target: <2000ms (P95)

**Lifecycle**:
- Created by OpenAI Agent
- Validated for citations
- Returned to frontend
- Optionally logged for analytics

---

## Data Flow Diagram

```
User Question
     │
     ▼
[1. Embed Question]
     │
     ▼
Query Embedding
     │
     ▼
[2. Qdrant Search] ──────► Top-10 Chunks (by cosine)
     │
     ▼
[3. Rerank Results] ──────► Top-5 Chunks
     │
     ▼
[4. Fetch Metadata] ◄────── Postgres Join (chunk_id)
     │
     ▼
Retrieval Results
     │
     ▼
[5. OpenAI Agent] ◄──────── search_book Tool
     │
     ▼
Chat Response ◄──────────── Citations Added
     │
     ▼
Return to User
```

---

## Indexing Strategy

### Qdrant Indexes

1. **HNSW Vector Index**:
   - Parameters: `M=16` (neighbors per layer), `ef_construct=100` (construction accuracy)
   - Distance: Cosine
   - Tradeoff: Balanced speed vs. accuracy for 300 chunks

2. **Payload Indexes**:
   - `chunk_id`, `file_path`, `section_heading`, `content_hash`
   - Type: Keyword (exact match)
   - Purpose: Fast filtering and deduplication

### Postgres Indexes

1. **Primary Key**: `chunk_id` (B-tree, automatic)
2. **Unique Constraint**: `content_hash` (B-tree, automatic)
3. **Secondary Indexes**:
   - `file_path` (B-tree): For "all chunks from file X" queries
   - `section_heading` (B-tree): For "all chunks in section Y" queries
   - `created_at` (B-tree): For auditing and time-based queries

**Index Maintenance**:
- Qdrant: Auto-managed (HNSW updates on insert/delete)
- Postgres: Auto-vacuumed on Neon (no manual maintenance needed)

---

## Data Lifecycle

### Ingestion (Initial & Refresh)

1. **Read**: Parse markdown files from `/docs/**`
2. **Chunk**: Split into 800-token chunks with 200-token overlap
3. **Hash**: Compute SHA256 of chunk text
4. **Check**: Query Postgres for existing `content_hash`
5. **Insert (if new)**:
   - Generate `chunk_id` (UUID v4)
   - Embed chunk text → vector
   - Insert into Qdrant (point + payload)
   - Insert into Postgres (metadata row)
6. **Update (if changed)**:
   - Generate new `chunk_id`
   - Delete old point from Qdrant
   - Delete old row from Postgres
   - Insert new point and row
7. **Delete (if orphaned)**:
   - Identify chunks whose `file_path` no longer exists
   - Delete from Qdrant and Postgres

### Query (Runtime)

1. **Receive**: User question via `/ask` or `/ask-selected`
2. **Embed**: Question → query_embedding
3. **Search**: Qdrant for top-10 similar chunks
4. **Filter**: Remove chunks with score <0.7
5. **Rerank**: Sort by relevance
6. **Select**: Top-5 chunks
7. **Enrich**: Optionally join with Postgres for additional metadata (if needed)
8. **Generate**: Pass to OpenAI Agent → response
9. **Return**: Response with citations

### Retention

- **Qdrant**: Permanent storage (until manual deletion or refresh)
- **Postgres**: Permanent storage (audit trail)
- **Logs**: Response logs retained for 30 days (debugging)
- **No user data**: Session IDs are ephemeral; no personal information stored

---

## Scaling Considerations

### Current Scale (Free Tier)
- **Qdrant**: ~300 chunks × 1536 dimensions × 4 bytes = ~1.8MB (well under 1GB limit)
- **Postgres**: ~300 rows × ~500 bytes/row = ~150KB (well under 3GB limit)
- **API**: <1000 queries/day (free tier supports 10K/day)

### Future Scale (if book grows to 1000 chunks)
- **Qdrant**: Upgrade to Paid tier (1000 chunks = ~6MB, still fits in 1GB, but may need more RAM)
- **Postgres**: Still fits in Free tier
- **API**: May need Render Starter plan ($7/month) for more compute

### Optimization Strategies
- **Qdrant**: Enable `on_disk_payload=true` if payload size grows (trades speed for storage)
- **Postgres**: Add materialized view for common queries (e.g., "chunks per file")
- **Caching**: Add Redis layer for frequently asked questions (not needed initially)

---

## Security & Privacy

### Data Protection
- **Embeddings**: Not personally identifiable; no user data in vectors
- **Metadata**: File paths only; no user-generated content
- **API Keys**: Stored in environment variables (Render secrets)
- **HTTPS**: Enforced for all API communication

### Compliance
- **No PII**: System does not collect personal information
- **No cookies**: Session IDs are passed in request body (not stored)
- **No tracking**: No analytics or user behavior logging (optional: add Google Analytics later)

### Access Control
- **Qdrant API Key**: Shared secret (rotated quarterly)
- **Postgres**: Read-write access via DATABASE_URL (Neon-managed)
- **OpenAI API**: Restricted to backend service (not exposed to frontend)

---

## Testing Data Model

### Unit Tests
- Validate chunk_id UUID format
- Check content_hash uniqueness
- Test token_count constraints (100-2000)
- Verify timestamp auto-update trigger

### Integration Tests
- Insert chunk → verify in both Qdrant and Postgres
- Update chunk → verify old deleted, new inserted
- Query by file_path → verify correct chunks returned

### Performance Tests
- Bulk insert 300 chunks (<30s)
- Qdrant search latency (<100ms for top-10)
- Postgres metadata join (<50ms)

---

## Migration Strategy

### V1 → V2 (if schema changes)

**Example: Adding `author` field to chunks_metadata**

1. **Postgres**:
   ```sql
   ALTER TABLE chunks_metadata ADD COLUMN author VARCHAR(255);
   ```

2. **Qdrant**:
   - Update payload schema in application code
   - Re-index all chunks (or run migration script to update existing payloads)

3. **Backward Compatibility**:
   - Make new fields `NULLABLE` initially
   - Backfill data asynchronously
   - Make `NOT NULL` after backfill complete

**Rollback Plan**:
- Keep schema version in Postgres metadata table
- Store database snapshots before migrations (Neon supports point-in-time recovery)

---

## Summary

The data model is designed for:
- **Simplicity**: Minimal schema, only essential fields
- **Scalability**: Can grow to 1000+ chunks with minor adjustments
- **Reliability**: Deduplication via hashes, timestamps for auditing
- **Performance**: Indexed fields for fast queries, HNSW for vector search
- **Maintainability**: Clear entity boundaries, documented constraints

**Key Tradeoffs**:
- Dual storage (Qdrant + Postgres) for flexibility vs. single DB for simplicity
- No conversation history storage for statelessness vs. persistence for analytics
- Manual refresh pipeline for control vs. automated CI/CD for convenience

**Next Steps**: Create API contracts based on this data model.
