---
id: 004
title: Create RAG Chatbot Implementation Blueprint
stage: misc
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: asadaligith
command: /sp.implement
labels: ["rag", "chatbot", "implementation", "blueprint", "architecture", "non-code"]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/implementation-blueprint.md
tests:
  - Checklist validation (13/13 requirements passed)
  - Blueprint completeness check (13 modules, all sections covered)
---

## Prompt

/sp.implement

Task:
Produce an implementation blueprint for Phase 2 RAG Chatbot, without writing any actual executable code. Focus on sequence, responsibilities, and integration logic.

Instructions:
1. Create step-by-step implementation instructions for:
   - Data ingestion workflow
   - Embedding and Qdrant indexing workflow
   - FastAPI backend components
   - Agent SDK configuration (tools, system prompt, policies)
   - Retrieval function logic (conceptual, no code)
   - Docusaurus widget integration (conceptual DOM/UX flow)
   - Deployment workflow (build → container → deploy)

2. For each module, include:
   - Purpose
   - Input/Output
   - Dependencies
   - Integration points
   - Failure modes

3. Describe backend API behavior and expected request → retrieval → response sequence.

4. Include instructions for:
   - Local testing
   - Endpoint validation
   - Data refresh and reindexing
   - Logging and monitoring
   - Error-handling patterns
   - Security considerations (rate limiting, CORS, API keys)

5. Ensure all instructions are high-level and conceptual.
   - Do NOT generate any code.
   - Describe architecture, flows, and responsibilities only.

6. Ensure the output can be handed directly to a developer to convert into code later.

Goal:
Produce a complete implementation plan describing how the integrated RAG chatbot system will be built from the specification and plan documents.

## Response snapshot

Created comprehensive implementation blueprint (13 modules, 15,000+ words) with conceptual, code-free guidance for RAG Chatbot development.

**Checklist Status Verified**:
- ✅ requirements.md: 13/13 items complete
- Overall status: ALL CHECKLISTS PASS - Ready for implementation

**Blueprint Structure** (13 Modules):

**Module 1: Data Ingestion Workflow**
- Purpose: Extract, chunk, hash markdown content
- Steps: File discovery → Reading → Header extraction → Chunking (800 tokens, 200 overlap) → Token counting → SHA256 hashing → Metadata assembly
- Output: List of chunks with file_path, section_heading, chunk_index, content_hash, token_count
- Failure modes: Missing /docs/, unreadable files, malformed markdown
- Integration: Feeds Module 2 (Embedding/Indexing)

**Module 2: Embedding and Qdrant Indexing Workflow**
- Purpose: Generate embeddings, store in Qdrant + Postgres, handle deduplication
- Steps: Qdrant collection init → Postgres schema init → Deduplication check (SHA256) → Batch embedding (OpenAI text-embedding-3-large) → Vector insertion (Qdrant upsert) → Metadata insertion (Postgres) → Orphan cleanup
- Output: 300 chunks indexed with 1536D vectors, cosine distance, HNSW index
- Failure modes: Qdrant unreachable, OpenAI rate limit, Postgres connection failure
- Integration: Receives from Module 1, Consumed by Module 5 (Retrieval)

**Module 3: FastAPI Backend Components**
- Purpose: Expose REST API, validate requests, orchestrate services
- Components:
  - Config (environment variables)
  - Request models (AskRequest, AskSelectedRequest)
  - Response models (ChatResponse, HealthResponse, ErrorResponse)
  - Routes: POST /ask, POST /ask-selected, GET /health
  - Middleware: CORS, rate limiting (10/min per session), timeout (10s)
- Failure modes: Invalid input (422), service unavailable (503), rate limit (429), timeout (504)
- Integration: Calls Module 4 (Agent) and Module 5 (Retrieval)

**Module 4: Agent SDK Configuration**
- Purpose: Generate answers with citations, enforce grounding
- System Prompt: "ONLY use search_book tool, MUST include citations, refuse non-book questions"
- Tool: search_book (query, max_results) → retrieves chunks via Module 5
- Execution flow: Init agent → Load history → Submit question → Agent calls tool → Synthesize answer → Validate citations → Extract sources
- Multi-turn: Store last 10 turns, LRU eviction (30 min idle)
- Failure modes: Agent hallucination (caught by citation validation), refusal false positive
- Integration: Calls Module 5 via search_book, Returns to Module 3

**Module 5: Retrieval Function Logic**
- Purpose: Query Qdrant, rerank, filter, return top-N
- Basic Retrieval: Embed query → Qdrant search (top-10) → Rerank by cosine → Filter >=0.7 → Top-5 selection
- Hybrid Retrieval (selected text): Embed query + selected text → Search both → Merge with hybrid score (0.7 * query + 0.3 * selected) → Filter → Top-5
- Context window: Max 3000 tokens (truncate if needed)
- Failure modes: Embedding failure, Qdrant unreachable, no results above threshold
- Integration: Called by Module 4 (search_book tool), Queries Qdrant + Postgres

**Module 6: Docusaurus Widget Integration**
- Purpose: Embed chatbot UI, handle interactions, communicate with backend
- Components:
  - Floating button (bottom-right, toggles panel)
  - Chat panel (messages, input, submit, loading)
  - Selected text handler ("Ask about this" button)
- API Communication: POST /ask (fetch with session_id), POST /ask-selected (with selected_text)
- State: Conversation history (client-side), session_id (sessionStorage)
- Failure modes: Network error, timeout, CORS error
- Integration: Calls Module 3 endpoints, Embedded in Docusaurus

**Module 7: Deployment Workflow**
- Purpose: Build, containerize, deploy
- Backend (Render):
  - Dockerfile (Python 3.11-slim, install deps, expose 8000)
  - render.yaml (envVars, healthCheckPath: /health)
  - Deploy: Connect repo → Set secrets → Auto-deploy on push
- Frontend (GitHub Pages):
  - Build Docusaurus → Deploy static files to gh-pages branch
  - Update widget to point to Render backend URL
- Failure modes: Build failure, health check failure, CORS misconfiguration
- Deployment checklist: 13 items (env vars, indexing, tests, CORS)

**Module 8: Logging and Monitoring**
- Purpose: Track health, debug issues, measure performance
- Structured Logging (JSON): timestamp, level, request_id, method, path, status, latency_ms
- Metrics: P50/P95/P99 latency, throughput (RPS), error rate, API usage
- Error tracking: 4xx (client), 5xx (server), silent failures (hallucinations)
- Tools: Render metrics (built-in), optional Sentry/Datadog
- Failure modes: Log overflow, missing critical logs

**Module 9: Local Testing Strategy**
- Unit tests (pytest): Chunking, hashing, embedding (mocked), retrieval (mocked), agent (mocked)
- Integration tests: Index 10 chunks → Query → Verify retrieval → Test API endpoints
- E2E tests (Cypress): Open chatbot → Ask question → Verify answer
- Performance tests (Locust): 100 concurrent users, measure P95 (<2s)
- Testing workflow: Unit → Integration → Manual → Full suite before deploy

**Module 10: Data Refresh and Reindexing**
- Purpose: Update vectors when book content changes
- Workflow: Read /docs → Chunk/hash → Query Postgres for existing hashes → Identify new/updated/deleted → Delete old → Insert new → Report changes
- Incremental vs Full: Incremental (default), Full (--force flag deletes all)
- CLI: `reindex_book.py --source ../docs/ [--force] [--dry-run]`
- Failure handling: Idempotent, partial state acceptable, re-run to catch missed chunks

**Module 11: Security Considerations**
- Rate limiting: 10 req/min per session_id, 1000 req/day global
- Input validation: 5-2000 chars (question), 10-5000 chars (selected_text), reject control chars
- CORS: Allow only https://asadaligith.github.io, POST/GET methods
- API key protection: Environment variables, never log, rotate quarterly
- HTTPS only: Render SSL, redirect HTTP → HTTPS
- Error sanitization: Don't leak sensitive info (DB URLs, stack traces)

**Module 12: Backend API Behavior Summary**
- Request → Response flow (16 steps): Frontend → Validate → Rate limit → Embed → Retrieve → Agent → Citations → Response
- Latency breakdown: Embedding 300ms + Retrieval 100ms + Agent 1200ms + Network 350ms = 2000ms (P95)
- Error paths: 422 (invalid), 429 (rate limit), 503 (service down), 504 (timeout)

**Module 13: Error Handling Patterns**
- Fail fast: Detect early, don't propagate invalid state
- Fail gracefully: User-friendly messages, not stack traces
- Retry transient: Exponential backoff (1s, 2s, 4s), max 3 retries
- Log everything: Errors with full context
- Error types: Input validation (422), External service (503), Business logic (200 with refusal), Timeout (504), Unexpected (500)

**Key Architectural Decisions Preserved**:
- Chunking: 800 tokens, 200 overlap (from research.md Decision 1)
- Embedding: text-embedding-3-large, 1536D (Decision 2)
- Qdrant: Cosine distance, HNSW (M=16, ef_construct=100) (Decision 3)
- API: 3 RESTful endpoints (Decision 4)
- Context: Top-5 chunks, 0.7 threshold, 3000 tokens (Decision 6)
- Agent: search_book tool, grounding enforced (Decision 7)
- Deployment: Render free tier (Decision 8)
- Hallucination prevention: Multi-layer (system prompt + threshold + citations) (Decision 12)

**Integration Flow Summary**:
```
Module 1 (Ingestion) → Module 2 (Indexing) → Qdrant + Postgres
Frontend Widget (Module 6) → Module 3 (API) → Module 5 (Retrieval) → Module 4 (Agent) → Response
Module 7 (Deployment) → Render (backend) + GitHub Pages (frontend)
Module 8 (Logging) ← All modules
```

**Blueprint Completeness**:
- ✅ 13 modules, each with Purpose, Input/Output, Dependencies, Integration Points, Failure Modes
- ✅ Step-by-step implementation instructions (conceptual, no code)
- ✅ Testing strategies for all layers (unit, integration, E2E, performance)
- ✅ Security considerations (6 categories)
- ✅ Deployment workflow (backend + frontend)
- ✅ Logging, monitoring, reindexing procedures
- ✅ Backend API behavior (16-step flow, latency breakdown)
- ✅ Error handling patterns (5 types, retry strategies)

**No Executable Code**: Blueprint is purely architectural guidance, ready to be handed to developers for code implementation.

## Outcome

- ✅ Impact: Comprehensive, code-free implementation blueprint ready for developer handoff. Provides clear architectural guidance, integration flows, and failure handling strategies for all 13 modules. Enables systematic development without guesswork.
- 🧪 Tests: Checklist validation passed (13/13 requirements complete). Blueprint completeness verified (all sections present, no missing integration points or failure modes).
- 📁 Files: Created implementation-blueprint.md (15,000+ words, 13 modules) with detailed conceptual guidance for RAG Chatbot implementation.
- 🔁 Next prompts: Developer should use this blueprint to write actual code, following module order (Setup → Foundational → User Story 1 → 2 → 3 → Polish). Refer to tasks.md for specific file paths and task breakdowns.
- 🧠 Reflection: Conceptual blueprint separates architecture from implementation, allowing developers to focus on code logic without re-deriving design decisions. All technical choices from research.md are preserved and explained in context. Blueprint is technology-specific (FastAPI, Qdrant, OpenAI) yet code-agnostic.

## Evaluation notes (flywheel)

- Failure modes observed: None. Blueprint generation completed successfully. All checklists passed before implementation.
- Graders run and results (PASS/FAIL):
  - Checklist Validation: PASS (13/13 requirements complete)
  - Blueprint Completeness: PASS (13 modules, all required sections present)
  - Integration Points: PASS (all module interactions documented)
  - Failure Modes: PASS (failure handling specified for each module)
  - No-Code Requirement: PASS (no executable code, purely conceptual)
- Prompt variant (if applicable): /sp.implement command (blueprint mode, not actual execution)
- Next experiment (smallest change to try): During actual implementation, monitor if developers need additional clarification on any module. If specific module is frequently misunderstood, expand that section with more detailed conceptual examples. Track implementation time vs estimated time per module to refine future blueprints.
