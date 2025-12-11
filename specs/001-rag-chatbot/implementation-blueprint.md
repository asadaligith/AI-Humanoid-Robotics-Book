# Implementation Blueprint: RAG Chatbot for AI Humanoid Robotics Book

**Feature**: 001-rag-chatbot
**Date**: 2025-12-09
**Purpose**: Conceptual implementation guide without executable code
**Audience**: Developers converting this blueprint into working code

---

## Overview

This blueprint provides step-by-step instructions for implementing the RAG Chatbot system. All instructions are **conceptual and architectural** - no executable code is included. Developers should use this as a guide to write actual implementation code.

**System Architecture Summary**:
- **Backend**: FastAPI REST API (Python 3.11+)
- **Vector Storage**: Qdrant Cloud (embeddings + metadata)
- **Metadata Storage**: Neon Serverless Postgres
- **AI Services**: OpenAI (text-embedding-3-large, Agent SDK)
- **Frontend**: Vanilla JavaScript widget in Docusaurus
- **Deployment**: Render (backend), GitHub Pages (frontend)

---

## Checklist Status

| Checklist | Total | Completed | Incomplete | Status |
|-----------|-------|-----------|------------|--------|
| requirements.md | 13 | 13 | 0 | ✓ PASS |

**Overall Status**: ✅ ALL CHECKLISTS COMPLETE - Ready for implementation

---

## Module 1: Data Ingestion Workflow

### Purpose
Extract markdown content from `/docs/**`, chunk into semantic segments, compute hashes for deduplication, and prepare for embedding generation.

### Input
- Markdown files from `/docs/` directory (all `.md` and `.mdx` files recursively)
- Chunking parameters: 800 tokens per chunk, 200-token overlap (25%)

### Output
- List of text chunks with metadata:
  - `chunk_text`: The actual text content
  - `file_path`: Relative path from repository root
  - `section_heading`: Extracted from markdown headers (H1-H3)
  - `chunk_index`: Position within document (0-indexed)
  - `content_hash`: SHA256 hash of chunk_text (hex string, 64 chars)
  - `token_count`: Number of tokens (counted via tiktoken library)

### Dependencies
- **LangChain RecursiveCharacterTextSplitter**: For markdown-aware chunking
- **tiktoken library**: For token counting (cl100k_base encoding)
- **hashlib (standard library)**: For SHA256 hashing
- **File system access**: Read permissions on `/docs/**`

### Implementation Steps

1. **File Discovery**:
   - Recursively scan `/docs/` directory for `.md` and `.mdx` files
   - Filter out excluded files (e.g., `_category_.json`, `.gitkeep`)
   - Build list of absolute file paths
   - **Failure mode**: If `/docs/` doesn't exist, log error and exit

2. **File Reading**:
   - For each markdown file, read full content as UTF-8 text
   - **Failure mode**: If file is unreadable (permissions, encoding), skip file and log warning

3. **Section Header Extraction**:
   - Parse markdown to extract H1, H2, H3 headings
   - Associate each chunk with the most recent heading above it
   - If no heading found, use file path as section_heading
   - **Failure mode**: If markdown is malformed, use basic regex to extract `# `, `## `, `### ` patterns

4. **Text Chunking**:
   - Initialize LangChain's RecursiveCharacterTextSplitter:
     - `chunk_size`: 800 tokens (measured, not characters)
     - `chunk_overlap`: 200 tokens
     - `separators`: `["\n\n", "\n", " ", ""]` (prioritize paragraph breaks)
     - `keep_separator`: True (preserves markdown structure)
   - Split each file's content into chunks
   - **Failure mode**: If splitting fails, fall back to fixed 3000-character chunks (approximate)

5. **Token Counting**:
   - For each chunk, count tokens using tiktoken.encoding_for_model("text-embedding-3-large")
   - Verify token_count is between 100 and 2000 (enforce constraints)
   - **Failure mode**: If chunk is too small (<100) or too large (>2000), skip with warning

6. **Content Hashing**:
   - For each chunk, compute SHA256(chunk_text.encode('utf-8')).hexdigest()
   - Store as `content_hash` for deduplication in next workflow
   - **Failure mode**: Hash collisions are extremely unlikely; no special handling needed

7. **Metadata Assembly**:
   - Create structured object for each chunk:
     ```
     {
       "chunk_text": <string>,
       "file_path": <string>,
       "section_heading": <string or null>,
       "chunk_index": <integer>,
       "content_hash": <string>,
       "token_count": <integer>
     }
     ```
   - Return list of all chunks across all files

### Integration Points
- **Output goes to**: Module 2 (Embedding and Indexing Workflow)
- **Called by**: Reindexing script (backend/scripts/reindex_book.py)

### Testing
- **Unit test**: Verify 800-token chunks with 200-token overlap
- **Integration test**: Process sample markdown file, verify output structure
- **Edge case test**: Empty files, files with no headers, very long files (>10K tokens)

---

## Module 2: Embedding and Qdrant Indexing Workflow

### Purpose
Generate embeddings for text chunks, store vectors in Qdrant, store metadata in Postgres, handle deduplication.

### Input
- List of chunks from Module 1 (each with chunk_text, file_path, section_heading, chunk_index, content_hash, token_count)
- OpenAI API key
- Qdrant URL and API key
- Postgres DATABASE_URL

### Output
- Qdrant collection `book_content` populated with vectors and payloads
- Postgres table `chunks_metadata` populated with metadata rows
- Deduplication report: X new chunks, Y updated, Z skipped (already exists)

### Dependencies
- **OpenAI SDK**: For text-embedding-3-large API calls
- **Qdrant Python client**: For vector insertion
- **SQLAlchemy**: For Postgres interaction
- **psycopg2**: PostgreSQL driver

### Implementation Steps

1. **Qdrant Collection Initialization** (one-time setup):
   - Check if collection `book_content` exists
   - If not, create collection with:
     - `vector_size`: 1536 (dimension for text-embedding-3-large)
     - `distance`: Cosine
     - `hnsw_config`: `{"m": 16, "ef_construct": 100}` (HNSW index parameters)
     - `on_disk_payload`: False (keep in memory for speed)
   - **Failure mode**: If Qdrant is unreachable, retry 3 times with exponential backoff, then fail

2. **Postgres Schema Initialization** (one-time setup):
   - Check if table `chunks_metadata` exists
   - If not, execute CREATE TABLE statement from data-model.md
   - Create indexes on file_path, section_heading, content_hash, created_at
   - **Failure mode**: If Postgres is unreachable, fail immediately (blocking issue)

3. **Deduplication Check**:
   - For each chunk, query Postgres: `SELECT chunk_id FROM chunks_metadata WHERE content_hash = ?`
   - If hash exists: Skip chunk (already indexed)
   - If hash doesn't exist: Proceed to embed and index
   - **Failure mode**: If query fails, assume chunk doesn't exist and proceed (better to duplicate than skip)

4. **Batch Embedding Generation**:
   - Group chunks into batches of up to 2048 (OpenAI batch limit)
   - For each batch, call OpenAI API:
     - Model: `text-embedding-3-large`
     - Input: List of chunk_text strings
     - Dimensions: 1536 (default for this model)
   - Extract embedding vectors from response (each is float[1536])
   - **Failure mode**: If API call fails (rate limit, timeout), retry with exponential backoff (max 3 retries)

5. **Vector Insertion into Qdrant**:
   - For each chunk, generate UUID (chunk_id)
   - Create Qdrant point:
     - `id`: chunk_id (UUID string)
     - `vector`: embedding vector (float[1536])
     - `payload`: {chunk_id, file_path, section_heading, chunk_index, content_text, created_at (ISO8601), content_hash}
   - Use `upsert` operation (idempotent - overwrites if exists)
   - **Failure mode**: If upsert fails, log error and continue with next chunk (partial indexing tolerated)

6. **Metadata Insertion into Postgres**:
   - For each chunk, insert row into chunks_metadata:
     - chunk_id, file_path, section_heading, chunk_index, content_hash, token_count, created_at, updated_at
   - Use `ON CONFLICT (content_hash) DO UPDATE` for idempotency
   - **Failure mode**: If insert fails, rollback Qdrant insertion for that chunk (maintain consistency)

7. **Progress Reporting**:
   - After each batch, log: "Processed X/Y chunks (Z% complete)"
   - At end, report: "Indexed N new chunks, updated M chunks, skipped P duplicates"

8. **Orphan Cleanup** (optional, during full re-index):
   - Query Postgres for all chunk_ids
   - Query file system for current file_paths
   - Identify chunks whose file_path no longer exists
   - Delete from Qdrant and Postgres
   - **Failure mode**: If cleanup fails, log warning but don't fail overall indexing

### Integration Points
- **Input from**: Module 1 (Data Ingestion Workflow)
- **Output to**: Qdrant and Postgres (consumed by Module 5: Retrieval Logic)
- **Called by**: Reindexing script (backend/scripts/reindex_book.py)

### Testing
- **Unit test**: Mock OpenAI API, verify batch embedding calls
- **Integration test**: Index 10 sample chunks, verify Qdrant and Postgres contain correct data
- **Deduplication test**: Index same chunk twice, verify only one entry exists
- **Stress test**: Index 300 chunks, measure total time (<5 minutes expected)

---

## Module 3: FastAPI Backend Components

### Purpose
Expose RESTful API endpoints for chatbot queries, handle request validation, orchestrate retrieval and agent services.

### Input
- HTTP requests from frontend (POST /ask, POST /ask-selected, GET /health)
- Environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL)

### Output
- HTTP responses (JSON): ChatResponse, HealthResponse, ErrorResponse
- Logs (structured JSON logs for monitoring)

### Dependencies
- **FastAPI framework**: For routing, request validation, async support
- **Pydantic**: For request/response models
- **Uvicorn**: ASGI server for running FastAPI
- **starlette middleware**: For CORS, error handling, rate limiting

### Component Structure

#### 3.1 Configuration Module (backend/src/config.py)

**Purpose**: Load and validate environment variables

**Implementation**:
- Use Pydantic Settings to define config class
- Required fields: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL
- Optional fields: LOG_LEVEL (default: INFO), RATE_LIMIT (default: 10/min)
- Validate on startup: If required field missing, raise error and exit
- **Failure mode**: Missing env var → immediate failure with clear error message

#### 3.2 Request Models (backend/src/models/requests.py)

**Purpose**: Define and validate incoming request schemas

**Pydantic Models**:

1. **AskRequest**:
   - `question`: str (min 5 chars, max 2000 chars)
   - `session_id`: Optional UUID (auto-generate if not provided)
   - **Validation**: Reject if question is empty, too long, or contains only whitespace

2. **AskSelectedRequest**:
   - `question`: str (min 5 chars, max 2000 chars)
   - `selected_text`: str (min 10 chars, max 5000 chars)
   - `session_id`: Optional UUID
   - **Validation**: Ensure selected_text is not just whitespace

**Failure modes**: Invalid requests return 422 Unprocessable Entity with details

#### 3.3 Response Models (backend/src/models/responses.py)

**Purpose**: Define outgoing response schemas

**Pydantic Models**:

1. **ChatResponse**:
   - `answer`: str
   - `sources`: List[Source] (0-5 elements)
   - `session_id`: UUID

2. **Source**:
   - `file`: str (file path)
   - `section`: Optional str (section heading)
   - `chunk`: str (truncated to 200 chars)
   - `similarity`: float (0-1)

3. **HealthResponse**:
   - `status`: "healthy" | "unhealthy"
   - `services`: {qdrant: bool, postgres: bool, openai: bool}
   - `version`: str (e.g., "1.0.0")
   - `timestamp`: ISO8601 timestamp

4. **ErrorResponse**:
   - `error`: str (human-readable message)
   - `code`: str (machine-readable code, e.g., "INVALID_QUESTION_LENGTH")
   - `details`: Optional dict (additional context)

#### 3.4 Routes: Chat Endpoints (backend/src/routes/chat.py)

**Purpose**: Handle /ask and /ask-selected endpoints

**POST /ask**:
- Accept AskRequest
- Validate request
- Embed question (Module 5)
- Retrieve context (Module 5)
- Generate answer (Module 4: Agent SDK)
- Return ChatResponse
- **Failure modes**:
  - Validation error → 422
  - Retrieval failure → 503 "Chatbot temporarily unavailable"
  - Rate limit exceeded → 429 "Please try again in 60 seconds"
  - Timeout (>10s) → 504 "Request timeout"

**POST /ask-selected**:
- Accept AskSelectedRequest
- Embed both question and selected_text
- Retrieve context with hybrid scoring (0.7 * query_sim + 0.3 * selected_sim)
- Generate answer (Module 4: Agent SDK)
- Return ChatResponse
- **Failure modes**: Same as /ask, plus validation for selected_text length

**Implementation Flow** (conceptual):
1. Parse and validate request body
2. Retrieve or generate session_id
3. Load conversation history (if exists) from session store
4. Call embedding service to embed question
5. Call retrieval service with query embedding (and selected_text embedding if applicable)
6. If no results above threshold (0.7), return "I don't have information about that in the book"
7. Call agent service with question, retrieved context, conversation history
8. Append Q&A to conversation history
9. Store updated history in session store
10. Return ChatResponse with answer and sources

#### 3.5 Routes: Health Endpoint (backend/src/routes/health.py)

**Purpose**: Monitor service health for deployment platform

**GET /health**:
- Check Qdrant connectivity (try to query collection info)
- Check Postgres connectivity (try to execute `SELECT 1`)
- Check OpenAI API connectivity (try to list models or lightweight call)
- Return HealthResponse with status of each service
- Overall status = "healthy" if all services UP, else "unhealthy"
- **Failure modes**: Service down → return 503 with details of which service(s) failed

**Implementation Flow** (conceptual):
1. Attempt Qdrant health check (timeout: 2s)
2. Attempt Postgres health check (timeout: 2s)
3. Attempt OpenAI health check (timeout: 2s)
4. Aggregate results
5. Return 200 if all healthy, 503 if any unhealthy

#### 3.6 Main Application (backend/src/main.py)

**Purpose**: Initialize FastAPI app, register routes, configure middleware

**Initialization Steps**:
1. Create FastAPI app instance
2. Add CORS middleware:
   - `allow_origins`: ["https://asadaligith.github.io"]
   - `allow_methods`: ["POST", "GET"]
   - `allow_headers`: ["Content-Type"]
3. Add rate limiting middleware (10 requests/minute per session_id)
4. Add request timeout middleware (10s per request)
5. Add structured logging middleware (log request_id, method, path, status, latency)
6. Register routes from chat.py and health.py
7. Add global exception handler (catch all errors, return ErrorResponse)
8. **Startup event**: Verify Qdrant and Postgres connectivity

**Middleware Order** (outer to inner):
1. CORS
2. Request logging (start timer)
3. Rate limiting
4. Timeout
5. Error handling
6. Route handler
7. Response logging (end timer, log latency)

**Failure modes**: Startup failure if Qdrant or Postgres unreachable

### Integration Points
- **Calls**: Module 4 (Agent SDK), Module 5 (Retrieval Logic)
- **Called by**: Frontend widget (HTTPS requests)
- **Storage**: Session store (in-memory dict, keyed by session_id)

### Testing
- **Unit tests**: Test each route handler with mocked services
- **Integration tests**: End-to-end test POST /ask with real Qdrant and Postgres (test database)
- **Performance tests**: Measure P95 latency (<2s requirement)
- **Error tests**: Simulate Qdrant down, verify 503 response

---

## Module 4: Agent SDK Configuration

### Purpose
Configure OpenAI Agent SDK to generate answers using retrieved context, enforce grounding to prevent hallucinations.

### Input
- User question
- Retrieved context (list of chunks with content and metadata)
- Conversation history (optional, for multi-turn)
- Session metadata

### Output
- Generated answer (string)
- Citations (list of sources used)
- Confidence score (optional)

### Dependencies
- **OpenAI Agent SDK**: For agent orchestration
- **OpenAI API**: For GPT-4 chat completions
- **Retrieval service**: For search_book tool implementation

### Agent Configuration

#### 4.1 System Prompt Design

**Purpose**: Set behavioral boundaries and enforce grounding

**System Prompt Template** (conceptual):
```
You are a helpful assistant for the AI Humanoid Robotics Book.

CRITICAL RULES:
1. ONLY answer questions using information retrieved from the search_book tool.
2. You MUST call the search_book tool before generating any answer.
3. If the search_book tool returns no results or results with low similarity (<0.7), respond with: "I don't have information about that in the book."
4. ALWAYS include citations in your answers. For each fact, reference the source file and section heading.
5. Do NOT use your general knowledge or training data. Stick strictly to the retrieved book content.
6. If asked about non-book topics (weather, current events, personal questions), respond: "I can only answer questions about the AI Humanoid Robotics Book."
7. Be concise. Aim for 2-4 sentence answers unless more detail is explicitly requested.

CITATION FORMAT:
For each fact or concept, include: (Source: [file_path], Section: [section_heading])

Example:
"Humanoid robots use several types of sensors including visual, tactile, and proprioceptive sensors (Source: /docs/module-02/chapter-05-sensor-types.md, Section: Sensor Integration)."
```

**Failure modes**: If agent ignores system prompt (rare), post-processing validation will catch missing citations

#### 4.2 Tool Definition: search_book

**Purpose**: Enable agent to query the book content

**Tool Schema** (conceptual JSON):
```json
{
  "name": "search_book",
  "description": "Search the AI Humanoid Robotics Book for information related to the user's question. This is the ONLY tool you should use to answer questions.",
  "parameters": {
    "type": "object",
    "properties": {
      "query": {
        "type": "string",
        "description": "The search query. Rephrase the user's question as a semantic search query for optimal retrieval (e.g., if user asks 'How does it work?', rephrase to include context from conversation history)."
      },
      "max_results": {
        "type": "integer",
        "description": "Number of chunks to retrieve (1-10). Default is 5.",
        "default": 5,
        "minimum": 1,
        "maximum": 10
      }
    },
    "required": ["query"]
  }
}
```

**Tool Implementation** (conceptual flow):
1. Receive `query` and `max_results` from agent
2. Embed query using OpenAI text-embedding-3-large
3. Call retrieval service (Module 5) with query embedding
4. Retrieve top-N chunks (where N = max_results)
5. Filter by similarity threshold (>=0.7)
6. Format results as structured JSON:
   ```json
   {
     "results": [
       {
         "file_path": "/docs/...",
         "section_heading": "...",
         "content": "...",
         "relevance_score": 0.92
       }
     ]
   }
   ```
7. Return to agent as tool response

**Failure modes**: If retrieval fails, return empty results array (agent will respond with "I don't have information...")

#### 4.3 Agent Execution Flow

**Purpose**: Orchestrate agent's reasoning and tool use

**Execution Steps** (conceptual):
1. **Initialize agent** with system prompt and search_book tool
2. **Provide conversation history** (if multi-turn):
   - Format as list of {role: "user" | "assistant", content: "..."}
   - Include last 5 turns (max 10 turns total history)
3. **Submit user question** as new user message
4. **Agent reasoning loop** (handled by Agent SDK):
   - Agent decides to call search_book tool
   - Agent generates query (may rephrase user question)
   - Tool is executed, results returned
   - Agent synthesizes answer from tool results
   - Agent formats answer with citations
5. **Post-processing validation**:
   - Check if answer contains citations (regex: `\(Source:.*?\)`)
   - If no citations and answer is not a refusal, reject and retry with stronger prompt
   - If still no citations, manually append: "(Note: Please refer to the book for more details)"
6. **Extract sources** from agent's answer:
   - Parse citation text to extract file_path and section_heading
   - Match against retrieved chunks to get similarity scores
   - Build Source objects for ChatResponse
7. **Return answer and sources**

**Failure modes**:
- Agent hallucinates (rare): Caught by citation validation
- Agent refuses valid question: Manual review needed (log for analysis)
- Agent timeout (>5s): Return error, suggest rephrasing question

#### 4.4 Multi-Turn Context Handling

**Purpose**: Enable coherent conversations with pronoun resolution

**Context Management**:
- **Storage**: In-memory dict keyed by session_id
- **Structure**: {session_id: [{"role": "user", "content": "..."}, {"role": "assistant", "content": "..."}]}
- **Retention**: Last 10 turns (or max 3000 tokens of history)
- **Eviction**: LRU policy for sessions idle >30 minutes

**Context Passing**:
- When agent receives new question, prepend conversation history
- Agent can reference previous questions/answers
- Example: User asks "What is computer vision?" → Agent answers → User asks "How is it used in robotics?" → Agent understands "it" refers to computer vision from history

**Failure modes**: If history grows too large (>3000 tokens), truncate oldest messages

### Integration Points
- **Input from**: Module 3 (FastAPI routes)
- **Calls**: Module 5 (Retrieval via search_book tool)
- **Output to**: Module 3 (returns answer and sources)

### Testing
- **Unit tests**: Mock search_book tool, verify agent calls it correctly
- **Hallucination tests**: Ask off-topic questions, verify refusal
- **Citation tests**: Verify all answers include citations
- **Multi-turn tests**: 3-turn conversation, verify pronoun resolution

---

## Module 5: Retrieval Function Logic

### Purpose
Query Qdrant for semantically similar chunks, rerank results, filter by threshold, return top-N.

### Input
- Query embedding (float[1536])
- Optional: Selected text embedding (float[1536]) for hybrid scoring
- Max results (default: 5)
- Similarity threshold (default: 0.7)

### Output
- List of retrieved chunks:
  - file_path
  - section_heading
  - content_text
  - similarity_score (0-1)
  - chunk_index

### Dependencies
- **Qdrant Python client**: For vector search
- **OpenAI SDK**: For embedding generation (if not pre-embedded)

### Retrieval Flow

#### 5.1 Basic Retrieval (General Questions)

**Steps** (conceptual):
1. **Embed query**:
   - Call OpenAI text-embedding-3-large with question text
   - Receive 1536-dimensional vector
   - **Failure mode**: If embedding fails, retry 3 times, then return error

2. **Qdrant search**:
   - Query collection `book_content`
   - Search parameters:
     - `vector`: query embedding
     - `limit`: 10 (retrieve top-10 initially for reranking)
     - `score_threshold`: 0.6 (lower threshold for initial retrieval)
     - `with_payload`: True (include metadata)
   - **Failure mode**: If Qdrant unreachable, return empty results

3. **Reranking**:
   - For each of top-10 results, compute cosine similarity between query embedding and chunk embedding (already done by Qdrant, use returned scores)
   - Sort by similarity score descending
   - **Failure mode**: If scores are missing, skip reranking and use as-is

4. **Threshold filtering**:
   - Filter results to keep only those with similarity >= 0.7 (strict threshold)
   - **Failure mode**: If no results above threshold, return empty list

5. **Top-N selection**:
   - Take top-5 results (or max_results parameter)
   - **Failure mode**: If fewer than 5 results, return all available

6. **Metadata enrichment** (optional):
   - Join with Postgres to get additional metadata (e.g., token_count, created_at)
   - Not strictly necessary since Qdrant payload includes essential metadata
   - **Failure mode**: If Postgres join fails, use Qdrant payload only

7. **Format response**:
   - Return list of dicts with file_path, section_heading, content_text, similarity_score

#### 5.2 Hybrid Retrieval (Selected Text Questions)

**Purpose**: Boost relevance of chunks related to selected text

**Steps** (conceptual):
1. **Embed both query and selected text**:
   - Query embedding: from user's question
   - Selected text embedding: from highlighted passage
   - **Failure mode**: If selected text embedding fails, fall back to basic retrieval

2. **Qdrant search** (run twice in parallel):
   - Search A: Query embedding → get top-10 with query_similarity scores
   - Search B: Selected text embedding → get top-10 with selected_similarity scores
   - **Failure mode**: If either search fails, use whichever succeeded

3. **Merge and hybrid scoring**:
   - For each unique chunk in results A ∪ B:
     - Compute hybrid score: 0.7 * query_similarity + 0.3 * selected_similarity
     - (If chunk appears in only one set, use that score with weight)
   - Sort by hybrid score descending
   - **Failure mode**: If hybrid scoring breaks, fall back to query_similarity only

4. **Threshold filtering and Top-N selection**:
   - Filter by hybrid score >= 0.7
   - Take top-5 results
   - Return formatted response

**Rationale for weights**: 0.7 query + 0.3 selected prioritizes user's explicit question while still boosting context from selected text

#### 5.3 Context Window Assembly

**Purpose**: Prepare retrieved chunks for agent consumption

**Steps** (conceptual):
1. **Token counting**:
   - For each retrieved chunk, count tokens (already stored in metadata)
   - Sum total tokens across top-5 chunks
   - **Failure mode**: If total exceeds 3000 tokens, truncate

2. **Truncation** (if needed):
   - If total tokens > 3000:
     - Keep top-ranked chunks first
     - Remove lowest-ranked chunks until under 3000 tokens
   - **Failure mode**: If even top chunk exceeds 3000 tokens, truncate content to fit

3. **Formatting for agent**:
   - Concatenate chunks with delimiters:
     ```
     === Chunk 1 (Similarity: 0.92) ===
     Source: /docs/module-02/chapter-05.md
     Section: Sensor Integration
     Content: [chunk text]

     === Chunk 2 (Similarity: 0.87) ===
     ...
     ```
   - This format helps agent understand source boundaries and similarity

### Integration Points
- **Called by**: Module 4 (Agent SDK via search_book tool), Module 3 (Routes for direct retrieval testing)
- **Input from**: OpenAI Embedding API, Qdrant, Postgres
- **Output to**: Module 4 (formatted context for agent)

### Testing
- **Unit tests**: Mock Qdrant responses, verify filtering and reranking logic
- **Integration tests**: Query with known test question, verify expected chunks retrieved
- **Threshold tests**: Verify chunks below 0.7 similarity are filtered out
- **Hybrid scoring tests**: Verify selected text boosts relevance correctly

---

## Module 6: Docusaurus Widget Integration

### Purpose
Embed chatbot UI in Docusaurus site, handle user interactions, communicate with backend API.

### Input
- User interactions (button clicks, text input, text selection)
- Backend API responses (ChatResponse, ErrorResponse)

### Output
- Rendered chatbot UI (floating button, chat panel)
- API requests to backend
- User-facing error messages

### Dependencies
- **Docusaurus**: Host environment
- **Fetch API**: For HTTP requests to backend
- **DOM APIs**: For UI manipulation
- **sessionStorage**: For persisting session_id

### Component Structure (Conceptual)

#### 6.1 Floating Button

**Purpose**: Entry point to open chat panel

**HTML Structure** (conceptual):
- Button element with fixed position (bottom-right corner)
- Icon: Chat bubble or robot icon
- Z-index: 9999 (above all other elements)
- Hover state: Slight scale animation

**Behavior**:
- **On click**: Toggle chat panel visibility (show if hidden, hide if shown)
- **State tracking**: Track panel open/closed state
- **Animation**: Smooth fade-in/fade-out transition

**Styling Considerations**:
- Use Docusaurus theme variables for colors (e.g., `--ifm-color-primary`)
- Responsive: Adjust size for mobile vs desktop
- Accessibility: ARIA labels, keyboard navigation support

#### 6.2 Chat Panel

**Purpose**: Display conversation, accept user input

**HTML Structure** (conceptual):
- Container div (fixed position, slides in from right)
- Header: Title "AI Humanoid Robotics Book Assistant" + Close button
- Messages container: Scrollable list of messages
- Input area: Text input field + Submit button
- Loading indicator: Spinner while waiting for response

**Message Rendering**:
- **User messages**: Right-aligned, blue background
- **Bot messages**: Left-aligned, gray background
- **Citations**: Inline links within bot messages (e.g., "(Source: chapter-05.md)")
- **Error messages**: Red background, warning icon

**Behavior**:
- **On submit**: Send question to backend, display loading indicator
- **On response**: Append bot message, hide loading indicator, scroll to bottom
- **On error**: Display error message inline (e.g., "Chatbot temporarily unavailable")
- **Auto-scroll**: Always scroll to latest message

**State Management** (client-side):
- **Conversation history**: Array of {role: "user" | "bot", content: "..."}
- **Session ID**: Retrieved from sessionStorage or generated on first use
- **Panel state**: Open/closed boolean

#### 6.3 API Communication

**Purpose**: Send requests to backend, handle responses

**Request Flow** (conceptual):

1. **POST /ask**:
   - Collect user input from text field
   - Retrieve session_id from sessionStorage (or generate UUID)
   - Build request body: `{question: "...", session_id: "..."}`
   - Set headers: `Content-Type: application/json`
   - Call fetch() with POST method
   - Handle response:
     - 200 OK: Parse ChatResponse, render answer and citations
     - 429 Rate Limit: Show "Please try again in 60 seconds"
     - 503 Service Unavailable: Show "Chatbot temporarily unavailable"
     - Network error: Show "Connection lost. Please check your internet"

2. **POST /ask-selected** (if selected text mode enabled):
   - Detect text selection on page (window.getSelection())
   - Show "Ask about this" button near selection
   - On click, open chat panel with selected text pre-filled
   - Build request body: `{question: "...", selected_text: "...", session_id: "..."}`
   - Same response handling as /ask

**Error Handling**:
- **Network errors**: Display user-friendly message, suggest retrying
- **Timeout (>10s)**: Show "Request timed out. Please try again."
- **Invalid responses**: Log error to console, show generic error message

**Session Management**:
- On first use, generate UUIDv4 and store in sessionStorage
- Include session_id in all API requests
- Session persists for browser session (cleared on tab close)

#### 6.4 Selected Text Handler

**Purpose**: Enable "Ask about this" feature for highlighted text

**Behavior** (conceptual):
1. **Detect selection**:
   - Listen for `mouseup` event on document
   - Check if selection length > 10 characters
   - Ignore selections within chat panel itself

2. **Show "Ask about this" button**:
   - Position button near selected text (above or below)
   - Button text: "Ask Chatbot"
   - Style: Small, subtle, Docusaurus-themed

3. **On button click**:
   - Capture selected text (window.getSelection().toString())
   - Open chat panel
   - Pre-populate input with prompt: "Explain: [first 50 chars of selected text]..."
   - Store full selected_text for API call
   - When user submits, use POST /ask-selected

**Edge Cases**:
- Selected text >5000 chars: Truncate with warning
- Selection contains HTML/formatting: Strip to plain text
- Selection disappears before button click: Graceful fallback

#### 6.5 UX Enhancements

**Loading States**:
- Show typing indicator while waiting for response (animated "..." dots)
- Disable input field during request to prevent duplicate submissions

**Accessibility**:
- ARIA labels for all interactive elements
- Keyboard navigation: Enter to submit, Escape to close panel
- Screen reader announcements for new messages

**Mobile Responsiveness**:
- On mobile: Panel slides up from bottom (not right)
- Full-screen panel on small screens (<600px width)
- Touch-friendly button sizes (min 44x44px)

**Performance**:
- Lazy load: Only initialize widget when button is clicked (reduces initial page load)
- Throttle scroll events (for auto-scroll)
- Debounce input typing (for future autocomplete feature)

### Integration with Docusaurus

**Step 1: Add JavaScript File**:
- Create file: `frontend/static/js/chatbot-widget.js`
- Add to docusaurus.config.js: `scripts: ['/js/chatbot-widget.js']`

**Step 2: Add CSS File**:
- Create file: `frontend/static/js/chatbot-styles.css`
- Use CSS variables from Docusaurus theme:
  - `var(--ifm-color-primary)` for primary color
  - `var(--ifm-font-family-base)` for font

**Step 3: Initialize Widget**:
- Use DOMContentLoaded event to initialize after page load
- Create Web Component (custom element `<chatbot-widget>`) for encapsulation
- Shadow DOM for style isolation

**Step 4: CORS Configuration**:
- Ensure backend allows `https://asadaligith.github.io` origin
- Handle preflight OPTIONS requests
- Include credentials if needed (not required for this project)

### Integration Points
- **Calls**: Module 3 (FastAPI backend via HTTPS)
- **Embedded in**: Docusaurus site (GitHub Pages)
- **User interactions**: Text input, button clicks, text selection

### Testing
- **Unit tests**: Test message rendering, state management
- **Integration tests**: Test API calls with mock backend
- **E2E tests**: Cypress or Playwright to simulate user interactions
- **Cross-browser tests**: Chrome, Firefox, Safari
- **Mobile tests**: iOS Safari, Android Chrome

---

## Module 7: Deployment Workflow

### Purpose
Build, containerize, and deploy backend to Render; deploy frontend to GitHub Pages.

### Input
- Source code (backend and frontend)
- Environment variables (secrets)
- Deployment configuration (render.yaml, Dockerfile)

### Output
- Backend API running on Render (e.g., https://rag-chatbot.onrender.com)
- Frontend widget embedded in GitHub Pages (https://asadaligith.github.io/AI-Humanoid-Robotics-Book/)

### Dependencies
- **Docker**: For containerization
- **Render**: For backend hosting
- **GitHub Actions**: For automated deployment (optional)
- **GitHub Pages**: For frontend hosting

### Deployment Steps

#### 7.1 Backend Deployment to Render

**Step 1: Create Dockerfile** (conceptual structure):
- Base image: `python:3.11-slim`
- Working directory: `/app`
- Copy requirements.txt, install dependencies
- Copy source code (`src/`)
- Expose port 8000
- Command: `uvicorn src.main:app --host 0.0.0.0 --port 8000`
- **Optimizations**: Multi-stage build (build dependencies separately), minimize layers

**Step 2: Create render.yaml** (conceptual structure):
```yaml
services:
  - type: web
    name: rag-chatbot-api
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn src.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: OPENAI_API_KEY
        sync: false  # Manually set in dashboard
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: DATABASE_URL
        sync: false
    healthCheckPath: /health
```

**Step 3: Connect Repository to Render**:
- Login to Render dashboard
- Create new Web Service
- Connect to GitHub repo (select `001-rag-chatbot` branch)
- Render auto-detects render.yaml

**Step 4: Configure Environment Variables**:
- In Render dashboard, add secrets:
  - OPENAI_API_KEY: [from OpenAI dashboard]
  - QDRANT_URL: [from Qdrant Cloud console]
  - QDRANT_API_KEY: [from Qdrant]
  - DATABASE_URL: [from Neon project settings]

**Step 5: Deploy**:
- Click "Deploy" or push to branch
- Render builds Docker image
- Runs build command (install dependencies)
- Starts service with start command
- Health check: Pings /health endpoint
- **Failure modes**:
  - Build failure: Check logs for missing dependencies
  - Health check failure: Verify Qdrant/Postgres are accessible
  - Runtime crash: Check environment variables are set correctly

**Step 6: Monitor Deployment**:
- Check Render logs for startup messages
- Verify /health endpoint returns 200
- Test /ask endpoint with curl or Postman

**Step 7: Configure Custom Domain** (optional):
- Add custom domain in Render dashboard
- Update CORS in backend to allow custom domain

#### 7.2 Frontend Deployment to GitHub Pages

**Step 1: Build Docusaurus Site**:
- Run build command (e.g., `npm run build` or `yarn build`)
- Generates static files in `build/` directory
- **Failure modes**: Build errors due to broken links, missing files

**Step 2: Configure GitHub Pages**:
- In GitHub repo settings, enable Pages
- Set source to `gh-pages` branch or `docs/` folder
- Custom domain (optional): asadaligith.github.io

**Step 3: Deploy Static Files**:
- Option A: Manual deploy:
  - Run `npm run deploy` (uses gh-pages package)
  - Pushes build/ to gh-pages branch
- Option B: GitHub Actions (automated):
  - Create workflow file (.github/workflows/deploy.yml)
  - Trigger on push to main branch
  - Build and deploy automatically

**Step 4: Update Widget Configuration**:
- Ensure chatbot-widget.js points to correct backend URL (Render domain)
- Update CORS in backend to allow GitHub Pages domain

**Step 5: Verify Deployment**:
- Visit https://asadaligith.github.io/AI-Humanoid-Robotics-Book/
- Open browser console, verify no CORS errors
- Click chatbot button, test end-to-end flow

#### 7.3 Deployment Checklist

**Pre-Deployment Validation**:
- [ ] All environment variables configured in Render
- [ ] Qdrant collection `book_content` exists with 300 chunks
- [ ] Postgres `chunks_metadata` table populated
- [ ] Backend /health endpoint returns 200 locally
- [ ] Backend unit tests pass
- [ ] Frontend widget tested locally with mock API
- [ ] CORS configured for GitHub Pages domain

**Post-Deployment Validation**:
- [ ] Backend health check passes (https://rag-chatbot.onrender.com/health)
- [ ] Can send test question to /ask endpoint and receive response
- [ ] Frontend widget loads on GitHub Pages
- [ ] End-to-end flow works: Ask question → Receive answer with citations
- [ ] Response time <2s (P95)
- [ ] No CORS errors in browser console

**Rollback Plan** (if deployment fails):
- Render: Revert to previous deployment in Render dashboard
- GitHub Pages: Revert commit and re-deploy

### Integration Points
- **Deploys**: Module 3 (Backend to Render), Module 6 (Frontend to GitHub Pages)
- **Monitors**: Module 8 (Logging and Monitoring)

### Testing
- **Smoke tests**: Basic functionality after deployment
- **Load tests**: Simulate 100 concurrent users (using tools like Locust)
- **Deployment tests**: Verify health checks, environment variables

---

## Module 8: Logging and Monitoring

### Purpose
Track system health, debug issues, measure performance, detect errors.

### Input
- Application logs (from FastAPI)
- Metrics (latency, error rates, API usage)
- Health check results

### Output
- Structured logs (JSON format)
- Monitoring dashboards (optional: Render metrics, third-party tools)
- Alerts (optional: Slack, email)

### Logging Strategy

#### 8.1 Structured Logging

**Purpose**: Enable searchable, parseable logs

**Log Format** (JSON, conceptual):
```json
{
  "timestamp": "2025-12-09T10:30:00.123Z",
  "level": "INFO",
  "request_id": "uuid",
  "method": "POST",
  "path": "/ask",
  "status": 200,
  "latency_ms": 1234,
  "session_id": "uuid",
  "question_length": 50,
  "retrieved_chunks": 5,
  "agent_latency_ms": 900,
  "error": null
}
```

**Log Levels**:
- DEBUG: Detailed diagnostics (embedding vectors, chunk IDs)
- INFO: Request/response summary (default level)
- WARNING: Recoverable errors (retry after failure, low similarity results)
- ERROR: Unrecoverable errors (API failures, missing config)
- CRITICAL: System failures (Qdrant unreachable, Postgres down)

**What to Log**:
- **All requests**: method, path, status, latency
- **Errors**: Full stack trace, error code, context
- **Retrieval**: Query embedding time, Qdrant search time, chunks retrieved, similarity scores
- **Agent**: Tool calls, response generation time, citation count
- **Rate limiting**: Session ID, request count, time window

**What NOT to Log**:
- API keys (security risk)
- Full user questions (privacy concern - log length only)
- Embedding vectors (too verbose - log dimensions only)

#### 8.2 Performance Metrics

**Key Metrics to Track**:
- **Response time**: P50, P95, P99 latency for /ask and /ask-selected
- **Throughput**: Requests per minute
- **Error rate**: 4xx and 5xx responses as % of total
- **API usage**: OpenAI API calls (embeddings, chat completions), cost estimation
- **Retrieval quality**: Average similarity score, % of queries with no results

**Measurement Approach**:
- **Middleware**: Track start/end time for each request
- **Service-level instrumentation**: Measure embedding generation, Qdrant search, agent response
- **Aggregation**: Calculate percentiles from logs (or use APM tool)

#### 8.3 Error Tracking

**Error Categories**:
- **Client errors (4xx)**: Invalid input, rate limit exceeded
- **Server errors (5xx)**: Qdrant down, OpenAI API failure, timeout
- **Silent failures**: Low retrieval quality, hallucinations (detected post-hoc)

**Error Handling**:
- **Retry logic**: Exponential backoff for transient failures (429, 503)
- **Circuit breaker**: If Qdrant fails 10 times in 1 minute, return cached error response for 5 minutes
- **Graceful degradation**: If OpenAI API is down, return "Service temporarily unavailable" instead of crashing

#### 8.4 Monitoring Dashboards (Optional)

**Render Built-In Metrics**:
- CPU usage, memory usage
- Request count, error rate
- Response time (P50, P95)

**Third-Party Tools** (if budget allows):
- **Sentry**: Error tracking and alerting
- **Datadog or New Relic**: APM (Application Performance Monitoring)
- **Logflare or Logtail**: Log aggregation and search

**Dashboard Views** (conceptual):
- **Health Overview**: Service status (Qdrant, Postgres, OpenAI), uptime %
- **Performance**: Latency trends over time, throughput graph
- **Errors**: Error rate trends, top error messages
- **Usage**: API call counts, cost estimation, top queries (anonymized)

### Integration Points
- **Logs from**: All modules (embeddings, retrieval, agent, routes)
- **Sends to**: Render logs (stdout), optional third-party services
- **Consumed by**: Developers (debugging), monitoring systems (alerts)

### Testing
- **Log validation**: Verify all requests generate structured logs
- **Performance tests**: Measure latency under load
- **Error simulation**: Trigger errors, verify they're logged correctly

---

## Module 9: Local Testing Strategy

### Purpose
Test implementation locally before deployment.

### Testing Pyramid

```
       ┌─────────────────┐
       │  E2E Tests      │  (Manual testing, Cypress)
       └─────────────────┘
      ┌───────────────────────┐
      │  Integration Tests    │  (Pytest with real Qdrant/Postgres)
      └───────────────────────┘
   ┌──────────────────────────────┐
   │      Unit Tests              │  (Pytest with mocks)
   └──────────────────────────────┘
```

### Test Categories

#### 9.1 Unit Tests

**Purpose**: Test individual functions in isolation

**Tools**: pytest, unittest.mock

**Test Cases** (examples):
- **Chunking**: Verify 800-token chunks with 200-token overlap
- **Hashing**: Verify SHA256 hashing produces correct output
- **Embedding**: Mock OpenAI API, verify correct request format
- **Retrieval**: Mock Qdrant, verify filtering and reranking logic
- **Agent**: Mock search_book tool, verify citation extraction

**Mocking Strategy**:
- Mock external APIs (OpenAI, Qdrant, Postgres)
- Use fixtures for common test data
- Verify function calls with `assert_called_once_with()`

#### 9.2 Integration Tests

**Purpose**: Test multiple components together with real dependencies

**Tools**: pytest, docker-compose (for local Qdrant/Postgres)

**Test Cases** (examples):
- **Indexing workflow**: Index 10 test chunks, verify in Qdrant and Postgres
- **Retrieval workflow**: Query with known question, verify expected chunks returned
- **API endpoints**: POST /ask with test question, verify ChatResponse format
- **End-to-end**: Index → Query → Retrieve → Agent → Response

**Setup**:
- Use docker-compose to run local Qdrant and Postgres
- Use test OpenAI API key (separate from production)
- Create test database with sample data

**Teardown**:
- Drop test database after tests
- Clear Qdrant test collection

#### 9.3 End-to-End (E2E) Tests

**Purpose**: Test full user journey from browser

**Tools**: Cypress or Playwright

**Test Cases** (examples):
- Open book page → Click chatbot button → Type question → Receive answer
- Select text → Click "Ask about this" → Verify selected text mode
- Multi-turn conversation: Ask 3 related questions

**Challenges**:
- Requires deployed frontend and backend (or local dev servers)
- Slower than unit/integration tests
- Harder to debug

#### 9.4 Performance Tests

**Purpose**: Verify latency and throughput requirements

**Tools**: Locust or Apache Bench

**Test Cases** (examples):
- **Load test**: 100 concurrent users, each sending 10 requests
- **Latency test**: Measure P95 response time (<2s requirement)
- **Stress test**: Gradually increase load until errors occur (find breaking point)

**Metrics to Measure**:
- Requests per second (RPS)
- P50, P95, P99 latency
- Error rate under load

### Testing Workflow

**Local Development**:
1. Write unit tests for new feature
2. Run unit tests: `pytest tests/unit/`
3. Write integration test
4. Run integration tests: `pytest tests/integration/`
5. Manual testing in local environment
6. Run full test suite before pushing: `pytest`

**CI/CD** (optional):
- GitHub Actions workflow on pull request
- Run unit tests and integration tests
- Block merge if tests fail

---

## Module 10: Data Refresh and Reindexing

### Purpose
Update vector database when book content changes.

### Reindexing Workflow

**Trigger**: Manual script execution (not automated)

**Steps** (conceptual):
1. **Read all markdown files** from `/docs/**`
2. **Chunk and hash** each file (Module 1)
3. **Query Postgres** for existing content hashes
4. **Identify changes**:
   - New chunks: content_hash not in Postgres
   - Updated chunks: file exists in Postgres but different hash
   - Deleted chunks: content_hash in Postgres but file no longer exists
5. **Delete old chunks** (if updated or deleted):
   - Delete from Qdrant by chunk_id
   - Delete from Postgres by chunk_id
6. **Insert new/updated chunks**:
   - Embed new chunks (Module 2)
   - Insert into Qdrant and Postgres
7. **Report**: "X new, Y updated, Z deleted"

**Incremental vs Full Reindex**:
- **Incremental** (default): Only process changed files
- **Full** (--force flag): Delete all, reindex all (slower but ensures consistency)

**Failure Handling**:
- If indexing fails halfway, partial state is acceptable (some chunks indexed, some not)
- Run script again to catch missed chunks (idempotent)
- No rollback mechanism (Postgres/Qdrant don't support transactions across both)

### Reindexing Script CLI

**Usage** (conceptual):
```bash
python backend/scripts/reindex_book.py [OPTIONS]

Options:
  --source PATH        Path to /docs/ directory (default: ../docs/)
  --force              Full reindex (delete all, reindex all)
  --dry-run            Show changes without applying
  --verbose            Detailed logging
```

**Example Outputs**:
```
Dry Run:
  New chunks: 15
  Updated chunks: 3
  Deleted chunks: 2
  Total: 18 changes

Actual Run:
  Indexing...
  [Progress: 20/300 chunks]
  [Progress: 100/300 chunks]
  [Progress: 300/300 chunks]
  Done: 15 new, 3 updated, 2 deleted
```

### Testing Reindexing
- **Unit test**: Mock file system changes, verify correct diff calculation
- **Integration test**: Modify test file, run script, verify changes in Qdrant/Postgres

---

## Module 11: Security Considerations

### Purpose
Protect against common vulnerabilities and abuse.

### Security Measures

#### 11.1 Rate Limiting

**Purpose**: Prevent abuse and protect free-tier quotas

**Implementation** (conceptual):
- **Per session_id**: 10 requests per minute
- **Global**: 1000 requests per day (to stay within OpenAI free tier)
- **Storage**: In-memory dict with timestamps
- **Eviction**: Clear old entries every 5 minutes

**Response**: 429 Too Many Requests with Retry-After header

#### 11.2 Input Validation and Sanitization

**Purpose**: Prevent injection attacks

**Validation**:
- **Question length**: 5-2000 characters
- **Selected text length**: 10-5000 characters
- **Character whitelist**: Allow alphanumeric, punctuation, spaces; reject control characters

**Sanitization**:
- Strip HTML tags from user input (use bleach library or regex)
- Escape SQL special characters (handled by SQLAlchemy parameterized queries)
- Reject patterns like `<script>`, `DROP TABLE`, `UNION SELECT`

**Testing**: SQL injection attempts, XSS attempts

#### 11.3 CORS Configuration

**Purpose**: Only allow requests from trusted domains

**Configuration** (conceptual):
```
allow_origins: ["https://asadaligith.github.io"]
allow_methods: ["POST", "GET"]
allow_headers: ["Content-Type"]
allow_credentials: False
```

**Failure mode**: Requests from other origins will be blocked by browser

#### 11.4 API Key Protection

**Purpose**: Never expose keys in code or logs

**Best Practices**:
- Store in environment variables (not in .env file committed to git)
- Use Render secrets dashboard for production
- Never log API keys (even in debug mode)
- Rotate keys quarterly

**Testing**: Verify no keys in git history (`git log -p | grep "OPENAI_API_KEY"`)

#### 11.5 HTTPS Only

**Purpose**: Encrypt data in transit

**Configuration**:
- Render provides free SSL certificates
- Redirect HTTP to HTTPS (Render handles automatically)
- Use HTTPS for all API calls from frontend

#### 11.6 Error Message Sanitization

**Purpose**: Don't leak sensitive information in errors

**Bad Example**: "Connection to Postgres at db.neon.tech:5432 failed"
**Good Example**: "Database temporarily unavailable"

**Implementation**: Catch all exceptions, return generic user-facing messages, log details server-side

### Security Testing
- **Penetration testing**: Try common attacks (SQLi, XSS, CSRF)
- **Dependency scanning**: Check for vulnerabilities in npm/pip packages
- **OWASP Top 10**: Verify protection against common web vulnerabilities

---

## Module 12: Backend API Behavior Summary

### Request → Retrieval → Response Sequence

**POST /ask Flow**:

```
1. [Frontend] User types question, clicks submit
     ↓
2. [Frontend] Send POST /ask {question, session_id}
     ↓
3. [Backend: Routes] Validate request (Pydantic)
     ↓
4. [Backend: Routes] Check rate limit (session_id)
     ↓
5. [Backend: Embeddings] Embed question → 1536D vector
     ↓
6. [Backend: Retrieval] Query Qdrant with vector → top-10 chunks
     ↓
7. [Backend: Retrieval] Rerank by similarity → filter >=0.7 → top-5
     ↓
8. [Backend: Retrieval] If no results, return empty list
     ↓
9. [Backend: Agent] Call OpenAI Agent SDK with:
    - System prompt (grounding rules)
    - search_book tool (pre-populated with retrieved chunks)
    - Conversation history (if multi-turn)
    - User question
     ↓
10. [Backend: Agent] Agent calls search_book tool (already executed in step 7)
     ↓
11. [Backend: Agent] Agent generates answer with citations
     ↓
12. [Backend: Agent] Validate answer has citations
     ↓
13. [Backend: Routes] Build ChatResponse:
    - answer (from agent)
    - sources (extracted from citations)
    - session_id
     ↓
14. [Backend: Routes] Update conversation history (append Q&A)
     ↓
15. [Backend: Routes] Return 200 OK with ChatResponse
     ↓
16. [Frontend] Render answer in chat panel
```

**Latency Breakdown**:
- Step 5 (Embedding): ~300ms
- Step 6-7 (Retrieval): ~100ms
- Step 9-11 (Agent): ~1200ms
- Step 12-15 (Response): ~50ms
- Network: ~350ms
- **Total**: ~2000ms (within <2s requirement)

**Error Paths**:
- Rate limit exceeded (step 4) → 429
- Embedding fails (step 5) → 503
- No results (step 8) → 200 with "I don't have information..." answer
- Agent fails (step 11) → 503
- Timeout (>10s) → 504

---

## Module 13: Error Handling Patterns

### Error Handling Philosophy
- **Fail fast**: Detect errors early, don't propagate invalid state
- **Fail gracefully**: Return user-friendly messages, not stack traces
- **Retry transient errors**: API rate limits, network glitches
- **Log everything**: Errors with full context for debugging

### Error Handling by Layer

#### 13.1 Input Validation Errors

**Trigger**: Invalid request (question too long, missing fields)
**Response**: 422 Unprocessable Entity
**Message**: "Question exceeds maximum length of 2000 characters"
**Retry**: No (client should fix input)

#### 13.2 External Service Errors

**Trigger**: OpenAI API down, Qdrant unreachable

**Response**: 503 Service Unavailable
**Message**: "Chatbot temporarily unavailable. Please try again later."
**Retry**: Yes (exponential backoff, max 3 retries)

**Exponential Backoff Pattern** (conceptual):
- Attempt 1: Immediate
- Attempt 2: Wait 1s
- Attempt 3: Wait 2s
- Attempt 4: Wait 4s
- Give up after 3 retries

#### 13.3 Business Logic Errors

**Trigger**: No results above similarity threshold

**Response**: 200 OK (not an error, just no data)
**Message**: "I don't have information about that in the book."
**Retry**: No (expected behavior)

#### 13.4 Timeout Errors

**Trigger**: Request takes >10s

**Response**: 504 Gateway Timeout
**Message**: "Request timed out. Please try a simpler question."
**Retry**: Yes (user can retry manually)

**Implementation**: Use asyncio.timeout or middleware timeout

#### 13.5 Unexpected Errors

**Trigger**: Unhandled exceptions (bugs)

**Response**: 500 Internal Server Error
**Message**: "An unexpected error occurred. Please try again."
**Log**: Full stack trace with request context
**Retry**: Maybe (user can try, but likely won't help unless transient)

**Global Exception Handler** (conceptual):
- Catch all exceptions not handled by routes
- Log error with traceback
- Return ErrorResponse with generic message
- Never expose internal details to user

---

## Summary of Implementation Blueprint

This blueprint provides:

1. ✅ **Module-by-module implementation guide** (13 modules)
2. ✅ **Input/Output specifications** for each module
3. ✅ **Dependencies and integration points** clearly defined
4. ✅ **Failure modes and error handling** strategies
5. ✅ **Testing strategies** (unit, integration, E2E, performance)
6. ✅ **Security considerations** (rate limiting, CORS, input validation)
7. ✅ **Deployment workflow** (Render + GitHub Pages)
8. ✅ **Logging and monitoring** approach
9. ✅ **Local testing** and reindexing procedures
10. ✅ **Behavioral sequence** for request → response flow

**Key Architectural Decisions Preserved**:
- 800-token chunks with 200-token overlap
- text-embedding-3-large (1536D vectors)
- Qdrant (cosine similarity, HNSW indexing)
- Neon Postgres (metadata storage)
- FastAPI (Python 3.11+)
- OpenAI Agent SDK (with search_book tool)
- Vanilla JavaScript widget (Docusaurus integration)
- Render deployment (free tier)
- Multi-layer hallucination prevention

**Ready for Code Implementation**:
- Developers can now write actual code following this blueprint
- Each module has clear specifications and expected behavior
- Testing requirements defined for validation
- All integration points documented

**No executable code** was included - this is purely architectural and conceptual guidance.

---

**Next Steps**: Hand this blueprint to developers to implement modules in order (Setup → Foundational → User Story 1 → 2 → 3 → Polish).
