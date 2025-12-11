# Research & Technical Decisions: RAG Chatbot

**Feature**: 001-rag-chatbot
**Date**: 2025-12-09
**Status**: Complete

## Overview

This document captures research findings and technical decisions for implementing a Retrieval-Augmented Generation (RAG) chatbot for the AI Humanoid Robotics Book. Each decision includes rationale, alternatives considered, and supporting references.

---

## Decision 1: Chunk Size and Overlap Strategy

**Decision**: Use 800-token chunks with 200-token overlap (25% overlap ratio)

**Rationale**:
- 800 tokens (~600 words) provides sufficient context for semantic understanding while remaining within LLM context windows
- 200-token overlap ensures concepts spanning chunk boundaries are captured in multiple chunks
- This size balances retrieval precision (smaller chunks = more precise but fragmented) with context completeness (larger chunks = more context but less precise)
- OpenAI's text-embedding-3-large performs optimally with 512-8191 tokens; 800 is well within this range
- Estimated 300 chunks for the book content fits comfortably within Qdrant Free Tier (1GB storage)

**Alternatives Considered**:

| Option | Chunk Size | Overlap | Why Not Chosen |
|--------|-----------|---------|----------------|
| Small chunks | 400 tokens | 100 tokens | Too fragmented; loses context for complex robotics explanations |
| Large chunks | 1500 tokens | 300 tokens | May dilute semantic search precision; harder to cite specific sections |
| No overlap | 800 tokens | 0 tokens | Risks losing concepts at boundaries; lower retrieval recall |
| Semantic chunking | Variable | N/A | More complex to implement; requires section header parsing which may be inconsistent |

**Supporting References**:
- OpenAI Embeddings Guide: https://platform.openai.com/docs/guides/embeddings
- Pinecone RAG Best Practices: Recommends 512-1024 tokens for technical content
- LangChain Text Splitter Documentation: 20-25% overlap is standard for RAG systems

**Implementation Notes**:
- Use LangChain's `RecursiveCharacterTextSplitter` with markdown-aware splitting (preserves headers)
- Count tokens using `tiktoken` library (cl100k_base encoding for text-embedding-3-large)
- Store original section headings in metadata for citation purposes

---

## Decision 2: Embedding Model Selection

**Decision**: OpenAI `text-embedding-3-large` (1536 dimensions)

**Rationale**:
- Industry-leading performance on MTEB benchmark (average score 64.6%)
- Native integration with OpenAI Agent SDK (same API, unified billing)
- 1536-dimensional vectors balance accuracy with storage efficiency
- Cost-effective: $0.13 per 1M tokens; estimated cost for 300 chunks (~240K tokens) = $0.03 one-time
- Strong performance on technical/domain-specific content compared to general-purpose models

**Alternatives Considered**:

| Model | Dimensions | Cost per 1M tokens | Why Not Chosen |
|-------|-----------|-------------------|----------------|
| text-embedding-3-small | 512 | $0.02 | Lower accuracy (MTEB 62.3%); not worth the marginal cost savings |
| text-embedding-ada-002 | 1536 | $0.10 | Older model; 3-large outperforms on retrieval tasks |
| Cohere embed-english-v3.0 | 1024 | $0.10 | Requires separate API integration; no advantage over OpenAI |
| Open-source (e.g., BGE-large) | 1024 | Free | Requires self-hosting; adds infrastructure complexity; lower quality |

**Supporting References**:
- OpenAI Embeddings Pricing: https://openai.com/api/pricing/
- MTEB Leaderboard: https://huggingface.co/spaces/mteb/leaderboard
- OpenAI text-embedding-3 announcement (2024-01): Performance improvements over ada-002

**Implementation Notes**:
- Batch embedding generation (up to 2048 inputs per request) to minimize API calls
- Store embedding model version in Postgres metadata for future compatibility
- Use cosine similarity for retrieval (normalized vectors; standard for semantic search)

---

## Decision 3: Qdrant Collection Schema

**Decision**: Single collection with payload-based metadata filtering

**Schema Structure**:
```
Collection: "book_content"
Vector Config:
  - Size: 1536
  - Distance: Cosine
  - On-Disk: false (use in-memory for free tier performance)

Payload Fields:
  - chunk_id (string, indexed): Unique identifier (UUID)
  - file_path (string, indexed): Source markdown file path
  - section_heading (string, indexed): Section/chapter heading
  - chunk_index (integer): Position within document
  - content_text (string): Original chunk text (for citation display)
  - created_at (timestamp): Indexing timestamp
  - content_hash (string): SHA256 hash for deduplication
```

**Rationale**:
- Single collection simplifies retrieval logic and reduces management overhead
- Indexed payload fields enable hybrid search (vector + metadata filtering)
- Content_text in payload avoids separate database lookup for display
- Cosine distance is standard for normalized embeddings (OpenAI vectors are pre-normalized)
- In-memory storage maximizes search speed on free tier (1GB limit sufficient for ~300 chunks)

**Alternatives Considered**:

| Option | Why Not Chosen |
|--------|----------------|
| Multiple collections (per chapter) | Adds complexity; requires collection selection logic; overkill for 300 chunks |
| Euclidean distance | Less suitable for normalized vectors; cosine is industry standard |
| On-disk storage | Slower retrieval; unnecessary given small dataset size |
| Separate metadata storage only | Requires additional Postgres query per result; increases latency |

**Supporting References**:
- Qdrant Documentation: Collections and Indexing
- Qdrant Best Practices: Recommends single collection for <10K vectors
- OpenAI Embeddings are L2-normalized; cosine similarity is optimal

**Implementation Notes**:
- Create collection with `vectors_config` and `optimizers_config` for HNSW indexing
- Use `on_disk_payload=false` for free tier performance
- Implement idempotent indexing (check content_hash before insertion to prevent duplicates)

---

## Decision 4: Qdrant vs. Alternatives (Distance Metric)

**Decision**: Cosine similarity (not Euclidean)

**Rationale**:
- OpenAI embeddings are L2-normalized (unit vectors)
- For normalized vectors, cosine similarity is mathematically equivalent to dot product but more interpretable (range -1 to 1)
- Cosine similarity is direction-based, ignoring magnitude, which aligns with semantic similarity
- Industry standard for text embeddings (used by Pinecone, Weaviate, OpenAI examples)

**Alternatives Considered**:

| Distance Metric | Why Not Chosen |
|-----------------|----------------|
| Euclidean (L2) | Magnitude-sensitive; less suitable for normalized vectors; cosine is preferred |
| Dot Product | Equivalent to cosine for normalized vectors but less intuitive scoring |
| Manhattan (L1) | Rarely used for embeddings; no performance benefit |

**Supporting References**:
- Qdrant Distance Metrics Documentation
- OpenAI Best Practices: Recommends cosine for text-embedding models
- Academic: "Cosine similarity is the de-facto standard for semantic text retrieval"

---

## Decision 5: API Route Design

**Decision**: FastAPI with 3 RESTful endpoints

**Endpoints**:

1. **POST /ask**
   - Input: `{"question": str, "session_id": str (optional)}`
   - Output: `{"answer": str, "sources": [{"file": str, "section": str, "chunk": str}], "session_id": str}`
   - Purpose: General Q&A with conversation context tracking

2. **POST /ask-selected**
   - Input: `{"question": str, "selected_text": str, "session_id": str (optional)}`
   - Output: `{"answer": str, "sources": [...], "session_id": str}`
   - Purpose: Context-specific queries with selected text as additional context

3. **GET /health**
   - Output: `{"status": str, "services": {"qdrant": bool, "postgres": bool, "openai": bool}, "version": str}`
   - Purpose: Healthcheck for monitoring and deployment validation

**Rationale**:
- RESTful design aligns with web standards; easier to integrate with JavaScript frontend
- Session ID enables multi-turn conversation tracking without requiring authentication
- Separate `/ask-selected` endpoint makes selected-text feature explicit and easier to A/B test
- Health endpoint is deployment best practice (used by Render/Fly/Railway for zero-downtime)
- JSON request/response is universally supported and easy to debug

**Alternatives Considered**:

| Option | Why Not Chosen |
|--------|----------------|
| GraphQL | Overkill for 3 simple endpoints; adds client complexity |
| WebSocket (real-time) | Not needed; request-response pattern sufficient for chatbot UX |
| Single `/chat` endpoint | Less clear separation between general and selected-text modes |
| gRPC | Requires protobuf; adds complexity without performance benefit at this scale |

**Supporting References**:
- FastAPI Documentation: RESTful best practices
- Render Deployment Guide: Health checks via HTTP endpoints
- MDN Web Docs: Fetch API for JavaScript-backend communication

**Implementation Notes**:
- Use Pydantic models for request/response validation
- Implement request timeout (10s) to prevent hanging connections
- Add CORS middleware for Docusaurus domain (GitHub Pages)
- Include rate limiting (e.g., 10 requests/minute per session_id) to prevent abuse

---

## Decision 6: RAG Context Window Design

**Decision**: Top-5 retrieval with reranking, max 3000-token context window

**Retrieval Strategy**:
1. Query Qdrant for top-10 chunks by cosine similarity
2. Rerank using relevance scoring (query-chunk semantic alignment)
3. Select top-5 chunks (estimated ~4000 tokens)
4. Truncate to 3000 tokens if needed (preserve top-ranked chunks)
5. Pass to OpenAI Agent as context

**Rationale**:
- Top-10 initial retrieval ensures recall of diverse relevant chunks
- Reranking improves precision (reduces false positives from semantic search)
- Top-5 final selection balances context richness with token budget
- 3000-token limit leaves headroom for: question (200), instructions (500), response (1000) within 8K context
- Empirical: RAG systems perform best with 3-5 high-quality chunks vs. 10+ lower-quality chunks

**Alternatives Considered**:

| Option | Why Not Chosen |
|--------|----------------|
| Top-3 retrieval | May miss relevant context; recall too low for complex questions |
| Top-10 retrieval (no reranking) | Dilutes context with marginally relevant chunks; increases noise |
| Hybrid search (keyword + vector) | Adds complexity; markdown content is well-suited for pure semantic search |
| Larger context (5000 tokens) | Risks exceeding LLM context limits; diminishing returns beyond 3000 tokens |

**Supporting References**:
- Anthropic RAG Guide: Recommends 3-5 retrieved chunks for optimal accuracy
- OpenAI GPT-4 context limits: 8K for gpt-4, 16K for gpt-4-turbo
- LangChain Reranking: Cohere rerank-english-v3.0 or simple cosine-based reranking

**Implementation Notes**:
- Use Qdrant's `limit` and `score_threshold` parameters (threshold: 0.7 for cosine)
- Implement simple reranking: compute cosine(query_embedding, chunk_embedding) for top-10
- Sort by reranked score and select top-5
- Count tokens using `tiktoken` to enforce 3000-token limit

---

## Decision 7: OpenAI Agent SDK Tool Schema

**Decision**: Single `search_book` tool with structured output

**Tool Definition**:
```
Name: search_book
Description: Search the AI Humanoid Robotics Book content for information related to the user's question.
Parameters:
  - query (string, required): Semantic search query (rephrased user question for optimal retrieval)
  - max_results (integer, optional, default=5): Number of chunks to retrieve (1-10)
Returns:
  - results (array of objects):
    - file_path (string): Source markdown file
    - section_heading (string): Chapter/section heading
    - content (string): Chunk text
    - relevance_score (float): Similarity score (0-1)
```

**Rationale**:
- Single tool simplifies agent logic and reduces decision complexity
- Structured output (JSON) enables agent to parse and cite sources correctly
- Query rephrasing parameter allows agent to optimize retrieval query vs. user question
- Max_results provides flexibility for agent to request more/less context based on question complexity
- Relevance score enables agent to assess confidence and avoid low-quality chunks

**Alternatives Considered**:

| Option | Why Not Chosen |
|--------|----------------|
| Multiple tools (search_chapter, search_concept) | Overcomplicates tool selection; single tool is sufficient |
| No tool (direct RAG in endpoint) | Agent SDK provides better reasoning and citation formatting |
| Tool returns only text (no metadata) | Loses citation capability; can't reference sources |
| Separate `get_chunk_details` tool | Unnecessary; single tool can return all needed data |

**Supporting References**:
- OpenAI Function Calling Guide: Best practices for tool design
- OpenAI Agent SDK Documentation: Structured outputs for tools
- Anthropic Tool Use Guide: Recommends single, well-designed tools over many specialized tools

**Implementation Notes**:
- Register tool using OpenAI Agent SDK's `@agent.tool` decorator
- Validate `max_results` range (1-10) to prevent over-retrieval
- Format tool output as JSON array for easy agent parsing
- Include system prompt: "ONLY use information from search_book tool. If no results, say 'I don't have information about that in the book.'"

---

## Decision 8: Deployment Platform Selection

**Decision**: Render (Web Service on Free Tier)

**Rationale**:
- **Free Tier**: 750 hours/month (sufficient for 24/7 operation), 512MB RAM, shared CPU
- **Zero-config deployment**: Connects to GitHub repo; auto-deploys on push
- **Built-in health checks**: Uses `/health` endpoint for zero-downtime deployments
- **HTTPS included**: Automatic SSL certificates (required for GitHub Pages integration)
- **Cold start**: ~30s spin-up after inactivity (acceptable for chatbot use case)
- **Environment variables**: Secure storage for API keys (OPENAI_API_KEY, QDRANT_URL, etc.)

**Alternatives Considered**:

| Platform | Free Tier | Why Not Chosen |
|----------|-----------|----------------|
| Fly.io | 3 shared VMs, 256MB RAM each | More complex configuration; overkill for single service |
| Railway | 500 hours/month, $5 credit | Lower monthly uptime than Render; credit-based billing risk |
| Vercel | Serverless, 100GB bandwidth | Optimized for frontends; not ideal for stateful FastAPI backend |
| Heroku | Deprecated free tier | No longer free as of 2022 |
| AWS Lambda | 1M requests/month | Cold start >3s; requires API Gateway setup; more complexity |

**Supporting References**:
- Render Documentation: Free tier limits and deployment guides
- Render vs. Competitors: Community comparisons favor Render for simple backends
- FastAPI Deployment Guide: Recommends Render for hobby/free projects

**Implementation Notes**:
- Create `render.yaml` for infrastructure-as-code
- Set environment variables in Render dashboard (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL)
- Configure health check path: `/health` with 30s interval
- Use `gunicorn` with `uvicorn` workers (recommended for FastAPI on Render)
- Add Dockerfile for consistent deployment (Python 3.11 base image)

---

## Decision 9: Neon Postgres Schema Design

**Decision**: Single `chunks_metadata` table with indexing on file_path and section_heading

**Schema**:
```sql
CREATE TABLE chunks_metadata (
  chunk_id UUID PRIMARY KEY,
  file_path VARCHAR(512) NOT NULL,
  section_heading VARCHAR(512),
  chunk_index INTEGER NOT NULL,
  content_hash CHAR(64) NOT NULL UNIQUE,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_file_path ON chunks_metadata(file_path);
CREATE INDEX idx_section_heading ON chunks_metadata(section_heading);
CREATE INDEX idx_content_hash ON chunks_metadata(content_hash);
```

**Rationale**:
- Minimalist schema focuses on essential metadata (no premature optimization)
- `chunk_id` links to Qdrant payload for cross-reference
- `content_hash` enables idempotent indexing (prevents duplicates on re-runs)
- Indexes on `file_path` and `section_heading` optimize common queries (e.g., "get all chunks from Chapter 3")
- No session/conversation storage (stateless API; session_id managed in-memory or client-side)

**Alternatives Considered**:

| Option | Why Not Chosen |
|--------|----------------|
| Store full chunk text in Postgres | Duplicates Qdrant data; increases storage costs; no benefit |
| Add conversation history table | Out of scope (no user auth or analytics requirement) |
| Separate tables (files, sections, chunks) | Over-normalization for simple use case; adds join complexity |
| NoSQL (MongoDB) | Overkill; relational data fits SQL; Neon Postgres is free and managed |

**Supporting References**:
- Neon Postgres Free Tier: 0.5GB storage, 1 project (sufficient for metadata)
- PostgreSQL Indexing Best Practices: B-tree indexes for equality and range queries
- FastAPI + SQLAlchemy: Recommended ORM for Postgres integration

**Implementation Notes**:
- Use SQLAlchemy ORM for Python-Postgres interaction
- Implement `updated_at` trigger for automatic timestamp updates
- Add foreign key constraint: `chunk_id` references Qdrant (logical, not enforced)
- Connection pooling: Use `asyncpg` with `databases` library for async queries

---

## Decision 10: Data Refresh Pipeline (Optional)

**Decision**: Manual re-indexing script (not automated CI/CD)

**Approach**:
- Create standalone Python script: `scripts/reindex_book.py`
- Script workflow:
  1. Read all markdown files from `/docs/**`
  2. Compute content hashes for each chunk
  3. Query Postgres for existing hashes
  4. Insert only new/changed chunks (upsert logic)
  5. Delete chunks for removed files (orphan cleanup)
- Run manually when book content is updated (e.g., monthly or after major edits)

**Rationale**:
- Book content is relatively static (not real-time updates)
- Manual trigger gives control over indexing timing (avoids mid-update inconsistencies)
- Simpler than CI/CD integration (no GitHub Actions, webhooks, or automated testing required)
- Free tiers have API rate limits; manual execution prevents accidental quota exhaustion
- Incremental indexing (hash-based) minimizes redundant API calls and costs

**Alternatives Considered**:

| Option | Why Not Chosen |
|--------|----------------|
| Automated (GitHub Actions on push) | Overkill for infrequent updates; consumes free tier CI/CD minutes |
| Real-time sync (file watcher) | Unnecessary complexity; book isn't live-edited |
| Full re-index every time | Wasteful; 300 chunks × $0.13/1M tokens = $0.04 per run (manageable but avoidable) |
| No refresh mechanism | Stale content after updates; manual deletion/re-insertion is error-prone |

**Supporting References**:
- LangChain Document Loaders: Markdown loading and chunking utilities
- Qdrant Upsert API: Idempotent insertion based on point ID
- PostgreSQL ON CONFLICT: Upsert pattern for deduplication

**Implementation Notes**:
- Use `click` library for CLI interface (e.g., `python reindex_book.py --dry-run`)
- Log changes: "X new chunks, Y updated, Z deleted"
- Add `--force` flag for full re-index (deletes all, re-inserts all)
- Store last index timestamp in Postgres metadata table for auditing

---

## Decision 11: Frontend Chatbot Widget Framework

**Decision**: Vanilla JavaScript (no framework) with Web Components

**Rationale**:
- **Zero dependencies**: No React/Vue/Svelte build step; simpler integration with Docusaurus
- **Web Components**: Encapsulated, reusable `<chatbot-widget>` custom element
- **Performance**: Minimal bundle size (<10KB minified); instant load time
- **Compatibility**: Works with any Docusaurus version; no plugin conflicts
- **Maintainability**: Easier to debug and customize without framework abstractions

**Alternatives Considered**:

| Option | Why Not Chosen |
|--------|----------------|
| React component | Requires build step; potential version conflicts with Docusaurus React |
| Docusaurus plugin | More complex; ties implementation to Docusaurus lifecycle |
| jQuery | Outdated; unnecessary for modern DOM manipulation |
| Svelte | Requires build tooling; overkill for simple widget |
| iframe embed | Prevents styling customization; poor UX (scrolling, responsiveness) |

**Supporting References**:
- MDN Web Components Guide: Custom elements and shadow DOM
- Docusaurus Client Modules: How to add custom JavaScript
- Modern vanilla JS: `fetch`, `addEventListener`, `classList` are now standard

**Implementation Notes**:
- Create `static/js/chatbot-widget.js` in Docusaurus
- Register in `docusaurus.config.js` under `scripts: ['/js/chatbot-widget.js']`
- Use Shadow DOM for CSS isolation (prevents style conflicts with Docusaurus theme)
- Implement lazy loading (initialize widget only when button is clicked to save resources)

---

## Decision 12: Hallucination Prevention Strategy

**Decision**: Multi-layered approach with strict grounding + confidence scoring

**Layers**:
1. **System Prompt**: "You are a helpful assistant for the AI Humanoid Robotics Book. ONLY answer questions using information from the search_book tool. If the tool returns no results or low-confidence results, respond with: 'I don't have information about that in the book.'"
2. **Confidence Threshold**: Reject Qdrant results with cosine similarity <0.7 (indicates low relevance)
3. **Tool Enforcement**: Agent cannot generate answers without calling `search_book` tool first (enforced via OpenAI Agent SDK config)
4. **Citation Requirement**: Agent must include source citations (file path + section heading) in every answer
5. **Post-processing Validation**: Backend checks if agent's answer contains citations; if not, returns error

**Rationale**:
- System prompt sets explicit behavioral boundaries (most effective according to OpenAI research)
- Confidence threshold filters out irrelevant retrievals (reduces false positive answers)
- Tool enforcement prevents agent from relying on pre-trained knowledge (which may be outdated or incorrect)
- Citation requirement makes answers verifiable and increases user trust
- Post-processing acts as safety net for edge cases

**Alternatives Considered**:

| Option | Why Not Chosen |
|--------|----------------|
| Retrieval-only (no LLM) | Can't synthesize information or generate natural language answers |
| Fine-tuned model | Expensive; overkill for this use case; pre-trained models work well with good prompting |
| Human-in-the-loop verification | Not feasible for real-time chatbot |
| Semantic similarity check (question vs answer) | Complex to implement; system prompt + citations are sufficient |

**Supporting References**:
- OpenAI Prompt Engineering Guide: Grounding techniques for factual accuracy
- Anthropic Constitutional AI: Principle-based prompting reduces hallucinations
- RAG Research (Lewis et al., 2020): Retrieval-augmented models reduce hallucination by 50%+

**Implementation Notes**:
- Add `strict_grounding=true` flag in agent config (if available in OpenAI SDK)
- Log all responses with confidence scores for monitoring
- Implement A/B test framework to measure hallucination rate (manual review of 50+ responses)

---

## Summary of Key Decisions

| Decision | Choice | Confidence |
|----------|--------|-----------|
| Chunk Size | 800 tokens, 200 overlap | High |
| Embedding Model | text-embedding-3-large | High |
| Vector DB | Qdrant (cosine similarity) | High |
| Metadata DB | Neon Postgres | High |
| API Framework | FastAPI (3 RESTful endpoints) | High |
| Context Window | Top-5 retrieval, 3000 tokens | Medium |
| Agent Tool | Single `search_book` tool | High |
| Deployment | Render Free Tier | High |
| Frontend | Vanilla JS Web Components | Medium |
| Hallucination Prevention | Multi-layer (prompt + threshold + citations) | High |
| Refresh Pipeline | Manual script (hash-based incremental) | High |

**Next Steps**:
- Proceed to Phase 1: Data model and API contracts
- Begin implementation with highest-confidence decisions
- Monitor and iterate on medium-confidence decisions (context window, frontend framework)

**References**:
- OpenAI Documentation: https://platform.openai.com/docs
- Qdrant Documentation: https://qdrant.tech/documentation
- FastAPI Documentation: https://fastapi.tiangolo.com
- Docusaurus Documentation: https://docusaurus.io
- Neon Postgres: https://neon.tech/docs
