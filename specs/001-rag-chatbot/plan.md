# Implementation Plan: RAG Chatbot for AI Humanoid Robotics Book

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

## Summary

Build a Retrieval-Augmented Generation (RAG) chatbot that allows readers of the AI Humanoid Robotics Book to ask questions and receive accurate, cited answers from book content. The system uses OpenAI's text-embedding-3-large for semantic search via Qdrant vector database, Neon Postgres for metadata storage, and the OpenAI Agent SDK for answer generation with strict grounding to prevent hallucinations.

**Technical Approach**:
- **Ingestion Pipeline**: Extract markdown from `/docs/**`, chunk into 800-token segments with 200-token overlap, embed using text-embedding-3-large, store in Qdrant + Postgres
- **Retrieval Pipeline**: Embed user questions, query Qdrant for top-10 similar chunks, rerank to select top-5, filter by 0.7 similarity threshold
- **Generation Pipeline**: Use OpenAI Agent SDK with `search_book` tool, enforce strict grounding via system prompts and citations, return answers with source references
- **Frontend Integration**: Vanilla JavaScript chatbot widget embedded in Docusaurus with floating button and chat panel
- **Deployment**: FastAPI backend on Render (free tier), CORS-enabled for GitHub Pages, health monitoring via `/health` endpoint

---

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI 0.109, OpenAI SDK 1.12, Qdrant Client 1.7, SQLAlchemy 2.0, LangChain 0.1
**Storage**: Qdrant Cloud (vector DB, free tier 1GB), Neon Serverless Postgres (metadata, free tier 3GB)
**Testing**: pytest 7.4+ for backend unit/integration tests; manual testing for frontend widget
**Target Platform**: Web (Linux server for backend via Render, browser-based frontend via Docusaurus)
**Project Type**: Web application (backend API + frontend widget)
**Performance Goals**: <2s response time (P95), <100ms Qdrant retrieval, <500ms embedding generation
**Constraints**: Free tier limits (Qdrant 1GB, Neon 3GB, Render 512MB RAM), OpenAI API rate limits (3500 RPM for embeddings)
**Scale/Scope**: 300 chunks initially, <1000 queries/day, single concurrent user per session

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Technical Accuracy (NON-NEGOTIABLE)

**Status**: ✅ **PASS**

- All RAG implementation decisions are backed by industry best practices (see `research.md`)
- Embedding model (text-embedding-3-large) is production-grade and documented by OpenAI
- Chunking strategy (800 tokens, 25% overlap) aligns with LangChain and Pinecone recommendations
- Retrieval metrics (cosine similarity, 0.7 threshold) are standard in semantic search literature
- OpenAI Agent SDK usage follows official documentation and examples

**Evidence**: All technical choices documented in `research.md` with supporting references to OpenAI docs, Qdrant docs, and academic RAG papers.

### Principle II: Clarity for Target Audience

**Status**: ✅ **PASS**

- Implementation plan is structured for intermediate-to-advanced developers
- Assumes familiarity with: Python, REST APIs, async programming, basic NLP concepts
- Provides step-by-step guidance in `quickstart.md` with clear phase breakdowns
- Technical jargon (embeddings, cosine similarity, HNSW) is used appropriately with context

### Principle III: Source-Backed Explanations

**Status**: ✅ **PASS**

- All technical decisions include rationale and references (OpenAI, Qdrant, FastAPI docs)
- Alternative approaches documented with justification for rejection
- Supporting references provided for each major decision (12 decisions in `research.md`)

### Principle IV: Modular Chapter Structure

**Status**: N/A (Not Applicable)

- This principle applies to book content, not chatbot implementation
- Chatbot enhances book modularity by enabling cross-chapter queries

### Principle V: Executable Code and Reproducible Examples

**Status**: ✅ **PASS** (Pending Implementation)

- Quickstart guide provides complete setup instructions
- `requirements.txt` specifies exact dependency versions for reproducibility
- Dockerfile ensures consistent deployment environment
- Testing plan includes unit, integration, and performance tests
- All code will be tested before deployment (commitment in testing strategy)

### Principle VI: Explainability and Step-by-Step Logic

**Status**: ✅ **PASS**

- Architecture broken down into clear pipelines (ingestion, retrieval, generation)
- Data flow diagram shows step-by-step processing (see `data-model.md`)
- Each API endpoint documented with request/response examples (see `contracts/api-openapi.yaml`)
- Quickstart guide provides phase-by-phase implementation order

**Overall Constitution Compliance**: ✅ **6/6 Applicable Principles PASS**

---

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── spec.md                  # Feature specification (completed)
├── plan.md                  # This file (implementation plan)
├── research.md              # Phase 0 output (decision logs)
├── data-model.md            # Phase 1 output (entities and schemas)
├── quickstart.md            # Phase 1 output (developer guide)
├── contracts/               # Phase 1 output (API contracts)
│   └── api-openapi.yaml     # OpenAPI 3.0 specification
├── checklists/
│   └── requirements.md      # Spec quality checklist (completed)
└── tasks.md                 # Phase 2 output (NOT created by /sp.plan - awaiting /sp.tasks command)
```

### Source Code (repository root)

```text
# Option 2: Web application (backend + frontend detected)

backend/                     # FastAPI backend
├── src/
│   ├── __init__.py
│   ├── main.py              # FastAPI app entry, CORS, routes registration
│   ├── config.py            # Environment variables (Pydantic Settings)
│   ├── models/
│   │   ├── __init__.py
│   │   ├── requests.py      # Pydantic models: AskRequest, AskSelectedRequest
│   │   └── responses.py     # Pydantic models: ChatResponse, HealthResponse, ErrorResponse
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embeddings.py    # OpenAI embedding generation (text-embedding-3-large)
│   │   ├── retrieval.py     # Qdrant search logic (top-10 → rerank → top-5)
│   │   ├── agent.py         # OpenAI Agent SDK integration (search_book tool)
│   │   └── database.py      # SQLAlchemy Postgres connection (Neon)
│   ├── routes/
│   │   ├── __init__.py
│   │   ├── chat.py          # POST /ask, POST /ask-selected
│   │   └── health.py        # GET /health
│   └── utils/
│       ├── __init__.py
│       ├── chunking.py      # LangChain RecursiveCharacterTextSplitter (800 tokens, 200 overlap)
│       ├── hashing.py       # SHA256 content hash for deduplication
│       └── session.py       # In-memory session management (UUID-based)
├── scripts/
│   └── reindex_book.py      # Standalone script: read /docs/**, chunk, embed, store
├── tests/
│   ├── __init__.py
│   ├── test_api.py          # Integration tests for /ask, /ask-selected, /health
│   ├── test_retrieval.py    # Unit tests for Qdrant search and reranking
│   ├── test_agent.py        # Unit tests for Agent SDK tool calls
│   └── test_chunking.py     # Unit tests for chunking logic
├── requirements.txt         # Python dependencies (pinned versions)
├── Dockerfile               # Multi-stage build for Render deployment
├── render.yaml              # Infrastructure-as-code for Render
├── .env.example             # Template for environment variables
└── README.md                # Backend setup and development guide

frontend/                    # Docusaurus integration (widget only)
├── static/
│   └── js/
│       ├── chatbot-widget.js    # Vanilla JS Web Component (floating button + chat panel)
│       └── chatbot-styles.css   # Widget styling (uses Docusaurus theme variables)
└── docusaurus.config.js     # Updated to include chatbot-widget.js script

docs/                        # Existing book content (not modified)
└── **/*.md                  # Markdown files to be indexed
```

**Structure Decision**: Web application structure chosen due to clear separation between backend (FastAPI API) and frontend (Docusaurus widget). Backend is deployed independently on Render; frontend is embedded in existing Docusaurus site on GitHub Pages. This separation enables independent scaling and deployment cycles.

---

## Complexity Tracking

**Status**: No violations; no complexity exceptions needed.

All design choices align with constitution principles and maintain simplicity:
- Single backend service (FastAPI) - no microservices overhead
- Single vector collection in Qdrant - no multi-collection management
- Stateless API design - no session persistence or distributed state
- Manual re-indexing - no automated CI/CD complexity
- Vanilla JS frontend - no framework build steps

---

## Architecture Overview

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                    User's Browser                            │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Docusaurus Book (GitHub Pages)                        │ │
│  │  ┌──────────────┐         ┌──────────────────────────┐ │ │
│  │  │  Book Content│         │ Chatbot Widget (JS)      │ │ │
│  │  │  /docs/**    │◄────────┤ - Floating Button        │ │ │
│  │  │              │         │ - Chat Panel             │ │ │
│  │  └──────────────┘         │ - Message Input          │ │ │
│  │                           │ - Selected Text Handler  │ │ │
│  │                           └────────┬─────────────────┘ │ │
│  └────────────────────────────────────┼───────────────────┘ │
└─────────────────────────────────────┼─────────────────────┘
                                        │ HTTPS/JSON
                                        ▼
┌─────────────────────────────────────────────────────────────┐
│           FastAPI Backend (Render)                           │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  API Routes                                           │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │  │
│  │  │ POST /ask    │  │ POST /ask-   │  │ GET /health│ │  │
│  │  │              │  │ selected     │  │            │ │  │
│  │  └──────┬───────┘  └──────┬───────┘  └────────────┘ │  │
│  │         │                  │                          │  │
│  │         └──────────┬───────┘                          │  │
│  │                    ▼                                   │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  Business Logic Services                       │  │  │
│  │  │  ┌─────────────────┐  ┌──────────────────────┐ │  │  │
│  │  │  │ Embeddings Svc  │  │  Retrieval Service   │ │  │  │
│  │  │  │ (OpenAI API)    │  │  (Qdrant Client)     │ │  │  │
│  │  │  └────────┬────────┘  └──────────┬───────────┘ │  │  │
│  │  │           │                       │             │  │  │
│  │  │  ┌────────▼───────────────────────▼───────────┐ │  │  │
│  │  │  │        OpenAI Agent Service                │ │  │  │
│  │  │  │  - Agent SDK orchestration                 │ │  │  │
│  │  │  │  - search_book tool integration           │ │  │  │
│  │  │  │  - Hallucination prevention (grounding)   │ │  │  │
│  │  │  └────────────────────────────────────────────┘ │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                      │                    │
                      │                    │
         ┌────────────▼─────────┐   ┌─────▼──────────────┐
         │  OpenAI API          │   │  Qdrant Cloud      │
         │  - text-embedding-   │   │  Collection:       │
         │    3-large           │   │  book_content      │
         │  - gpt-4 (Agent SDK) │   │  - 1536D vectors   │
         └──────────────────────┘   │  - Cosine distance │
                                     │  - HNSW index      │
                                     └────────────────────┘
                                              │
                                              │ Metadata Join
                                              ▼
                                     ┌────────────────────┐
                                     │  Neon Postgres     │
                                     │  Table:            │
                                     │  chunks_metadata   │
                                     │  - file_path       │
                                     │  - section_heading │
                                     │  - content_hash    │
                                     └────────────────────┘
```

### Data Flow: User Question → Answer

**Step-by-Step Flow**:

1. **User Input**:
   - User types question in chatbot widget
   - Widget sends POST to `/ask` with `{"question": str, "session_id": uuid}`

2. **Backend Processing**:
   - FastAPI route validates request (Pydantic model)
   - Calls `embeddings.embed_text(question)` → generates 1536D vector
   - Calls `retrieval.search(query_embedding, limit=10)` → Qdrant returns top-10 chunks
   - Reranks top-10 by cosine(query_embedding, chunk_embedding)
   - Filters by similarity threshold >=0.7
   - Selects top-5 chunks

3. **Agent Generation**:
   - Calls `agent.generate_answer(question, retrieved_chunks, session_id)`
   - Agent SDK invokes `search_book` tool (already executed in step 2)
   - System prompt enforces: "ONLY use search_book results; cite sources"
   - Agent generates answer with citations

4. **Response**:
   - Backend returns `{"answer": str, "sources": [...], "session_id": uuid}`
   - Widget displays answer in chat panel with clickable citations

**Latency Breakdown** (Target <2s):
- Embedding generation: ~300ms
- Qdrant search: ~80ms
- Reranking: ~20ms
- Agent generation: ~1200ms
- Network overhead: ~400ms
- **Total**: ~2000ms (P95)

---

## Implementation Phases

### Phase 0: Research & Decisions ✅ COMPLETE

**Output**: `research.md` with 12 documented decisions
**Key Decisions**:
1. Chunk size: 800 tokens, 200 overlap
2. Embedding model: text-embedding-3-large
3. Qdrant collection schema (cosine, 1536D, in-memory)
4. API route design (3 RESTful endpoints)
5. RAG context window (top-5, 3000 tokens max)
6. Agent tool schema (`search_book`)
7. Deployment platform (Render free tier)
8. Postgres schema (chunks_metadata table)
9. Distance metric (cosine similarity)
10. Hallucination prevention (multi-layer)
11. Frontend framework (Vanilla JS Web Components)
12. Data refresh pipeline (manual hash-based script)

**Status**: All decisions finalized with high/medium confidence ratings.

---

### Phase 1: Design & Contracts ✅ COMPLETE

**Outputs**:
1. `data-model.md`: Entity definitions, schemas, relationships
2. `contracts/api-openapi.yaml`: OpenAPI 3.0 specification
3. `quickstart.md`: Developer implementation guide

**Key Artifacts**:
- **Entities**: Document Chunk, Chunks Metadata, Search Query, Retrieval Result, Chat Response
- **Qdrant Schema**: Collection `book_content`, 1536D vectors, HNSW index, payload indexes
- **Postgres Schema**: Table `chunks_metadata`, 8 columns, 4 indexes, SHA256 deduplication
- **API Contracts**: 3 endpoints with request/response examples, error codes, validation rules

**Status**: Design approved; ready for Phase 2 (tasks generation via `/sp.tasks` command).

---

### Phase 2: Tasks Generation (Pending `/sp.tasks` command)

**Purpose**: Generate detailed, testable tasks for implementation
**Input**: This plan + data model + API contracts
**Output**: `tasks.md` with prioritized, dependency-ordered tasks

**Expected Task Structure** (Preview):
- **Epic 1: Infrastructure Setup** (Qdrant, Neon, OpenAI keys)
- **Epic 2: Backend Services** (embeddings, retrieval, agent, database)
- **Epic 3: API Routes** (chat endpoints, health check, error handling)
- **Epic 4: Indexing Pipeline** (chunking script, deduplication, batch processing)
- **Epic 5: Frontend Widget** (floating button, chat panel, API integration)
- **Epic 6: Deployment** (Dockerfile, Render config, CORS setup)
- **Epic 7: Testing & Validation** (unit tests, integration tests, UAT)

**Note**: `/sp.plan` command does NOT generate tasks; use `/sp.tasks` for Phase 2.

---

## Testing Strategy

### Test Pyramid

```
        ┌─────────────┐
        │  Manual UAT │  (10 test cases, user scenarios)
        └─────────────┘
       ┌───────────────────┐
       │ Integration Tests │  (API endpoints, full flow)
       └───────────────────┘
    ┌──────────────────────────┐
    │      Unit Tests          │  (Services, utils, models)
    └──────────────────────────┘
```

### Test Categories

#### 1. Retrieval Accuracy Tests

**Purpose**: Verify semantic search returns relevant chunks

**Test Cases**:
- TC-RA-001: Query "What are robot sensors?" → Verify top-5 chunks are from sensor-related chapters
- TC-RA-002: Query with typos "robut actuators" → Verify still retrieves actuator content (embeddings handle typos)
- TC-RA-003: Query "quantum computing" (off-topic) → Verify no results above 0.7 threshold
- TC-RA-004: Query very long question (>1000 words) → Verify truncation or error handling
- TC-RA-005: Selected text query → Verify selected text boosts relevance of related chunks

**Success Criteria**: 90%+ accuracy on 20 diverse queries (manual evaluation)

#### 2. Hallucination Prevention Tests

**Purpose**: Ensure agent only answers from retrieved content

**Test Cases**:
- TC-HP-001: Query "What's the weather in New York?" → Verify refusal message ("I don't have information...")
- TC-HP-002: Query "Who invented the transistor?" (general knowledge) → Verify refusal if not in book
- TC-HP-003: Query about Chapter 5 when no Chapter 5 chunks retrieved → Verify refusal
- TC-HP-004: Verify all answers include citations (file path + section)
- TC-HP-005: Check answer against retrieved chunks → Verify no facts outside retrieved content

**Success Criteria**: 0% hallucination rate on 50 test queries (manual review)

#### 3. Selected-Text Mode Tests

**Purpose**: Verify selected text context improves answer focus

**Test Cases**:
- TC-ST-001: Select paragraph, ask "Explain this" → Verify answer addresses selected text
- TC-ST-002: Select code snippet, ask "What does this do?" → Verify answer explains the code
- TC-ST-003: Select heading only (5 words) → Verify system handles gracefully
- TC-ST-004: Select 5000 characters → Verify system accepts or asks for shorter selection

**Success Criteria**: 80%+ of selected-text answers are more focused than general answers

#### 4. Stress Tests (Multiple Requests)

**Purpose**: Verify system handles concurrent load

**Test Cases**:
- TC-SR-001: Send 10 simultaneous requests → Verify all return within 5s
- TC-SR-002: Send 100 requests sequentially → Verify no rate limit errors (within OpenAI quota)
- TC-SR-003: Send same question 50 times → Verify consistent answers (no randomness issues)
- TC-SR-004: Test OpenAI API failure → Verify 503 error with friendly message

**Success Criteria**: <5% error rate under 10 concurrent users, 0 crashes

#### 5. Integration Test: End-to-End Flow

**Purpose**: Validate complete user journey

**Test Flow**:
1. Open Docusaurus book page
2. Click chatbot floating button → Verify chat panel opens
3. Type question "What is inverse kinematics?" → Submit
4. Verify loading indicator appears
5. Verify answer appears within 2s
6. Verify citations are clickable (optional: link to source file)
7. Click citation → Verify navigates to correct page (future enhancement)
8. Ask follow-up question "How is it used in robotics?"
9. Verify multi-turn context maintained
10. Ask off-topic question "What's for dinner?" → Verify refusal

**Success Criteria**: All 10 steps pass without errors

#### 6. User Acceptance Tests (UI Responsiveness)

**Purpose**: Validate user experience quality

**Test Cases**:
- TC-UA-001: Widget loads in <1s on page load
- TC-UA-002: Floating button visible on all page sizes (responsive)
- TC-UA-003: Chat panel scrollable when many messages
- TC-UA-004: Input field supports long questions (2000 chars)
- TC-UA-005: Error messages are user-friendly (no stack traces shown)
- TC-UA-006: Citations are clearly formatted (file path + section heading)

**Success Criteria**: 90%+ usability score (manual testing by 3+ users)

---

## Deployment Plan

### Pre-Deployment Checklist

- [ ] All environment variables configured in Render dashboard
- [ ] Qdrant collection created and accessible
- [ ] Neon Postgres database schema deployed
- [ ] OpenAI API billing enabled and quota verified (>$5 credit)
- [ ] Backend unit tests pass (pytest)
- [ ] API integration tests pass
- [ ] Health endpoint returns 200
- [ ] CORS configured for `https://asadaligith.github.io`
- [ ] Chatbot widget tested locally with mock API
- [ ] Indexing script run successfully (300+ chunks in Qdrant)

### Deployment Steps

#### Backend Deployment (Render)

1. **Prepare Repository**:
   - Commit all code to `001-rag-chatbot` branch
   - Ensure `requirements.txt`, `Dockerfile`, `render.yaml` are present
   - Push to GitHub

2. **Connect Render**:
   - Login to Render dashboard
   - Create new Web Service
   - Connect to GitHub repo (select `001-rag-chatbot` branch)
   - Render auto-detects `render.yaml`

3. **Configure Environment Variables**:
   - `OPENAI_API_KEY`: From OpenAI dashboard
   - `QDRANT_URL`: From Qdrant Cloud console (e.g., `https://xxx.qdrant.io`)
   - `QDRANT_API_KEY`: From Qdrant API keys
   - `DATABASE_URL`: From Neon project settings (PostgreSQL connection string)

4. **Deploy**:
   - Click "Deploy" (automatic build from `render.yaml`)
   - Monitor build logs for errors
   - Verify deployment succeeds (check `/health` endpoint)

5. **Test**:
   - Send test request to `https://[app-name].onrender.com/health`
   - Verify returns `{"status": "healthy", "services": {"qdrant": true, ...}}`
   - Send test question to `/ask` endpoint

#### Frontend Deployment (GitHub Pages)

1. **Add Widget to Docusaurus**:
   - Copy `chatbot-widget.js` to `static/js/` in main branch
   - Update `docusaurus.config.js` to include script
   - Update widget to point to Render backend URL

2. **Configure CORS**:
   - Ensure backend CORS allows `https://asadaligith.github.io`
   - Test cross-origin requests from browser console

3. **Deploy to GitHub Pages**:
   - Merge `001-rag-chatbot` branch to main
   - Run `npm run build && npm run deploy` (or use GitHub Actions)
   - Verify widget appears on deployed site

4. **Smoke Test**:
   - Visit https://asadaligith.github.io/AI-Humanoid-Robotics-Book/
   - Click chatbot button
   - Ask test question
   - Verify answer appears with citations

### Post-Deployment Monitoring

**Week 1**: Daily monitoring
- Check Render logs for errors
- Monitor OpenAI API usage and costs
- Verify Qdrant and Neon remain within free tier limits
- Test chatbot from different browsers and devices

**Week 2-4**: Weekly monitoring
- Review error rates (target: <1%)
- Analyze slow queries (identify P95 >2s responses)
- Collect user feedback (if feedback mechanism added)
- Optimize prompts if hallucination detected

### Rollback Plan

**If deployment fails**:
1. Revert to previous Render deployment (via Render dashboard)
2. Remove chatbot widget script from `docusaurus.config.js`
3. Redeploy Docusaurus site without chatbot
4. Debug locally, fix, re-deploy

**If runtime issues**:
1. Check health endpoint (`/health`)
2. Verify environment variables are set
3. Check Render logs for stack traces
4. Restart service via Render dashboard
5. If persistent, disable chatbot temporarily

---

## Risk Mitigation

### Risk 1: Cost Overruns

**Impact**: Medium | **Likelihood**: Low

**Mitigation**:
- Implement rate limiting: 10 requests/minute per session_id
- Monitor OpenAI usage daily via API dashboard
- Set up billing alerts ($10, $20, $50 thresholds)
- Cache frequent queries (future enhancement)

**Contingency**: Upgrade to paid tiers if traffic exceeds free limits (budget $20/month).

### Risk 2: API Availability

**Impact**: High | **Likelihood**: Medium

**Mitigation**:
- Implement retry logic with exponential backoff for transient failures
- Return 503 with friendly message when dependencies unavailable
- Health endpoint monitors all dependencies (Qdrant, Postgres, OpenAI)

**Contingency**: Display maintenance message in chatbot if prolonged outage.

### Risk 3: Hallucination Despite Safeguards

**Impact**: High | **Likelihood**: Low

**Mitigation**:
- Multi-layer prevention: system prompt + threshold + citations
- Post-processing validation (check for citations in answer)
- Manual review of 50+ responses during testing
- A/B test different system prompts

**Contingency**: Strengthen system prompt, increase threshold to 0.75, reduce context window to top-3.

### Risk 4: Poor Retrieval Quality

**Impact**: Medium | **Likelihood**: Medium

**Mitigation**:
- Test multiple chunk sizes during indexing (compare 600, 800, 1000 tokens)
- Evaluate retrieval on diverse query set (20+ questions)
- Adjust overlap (15%, 25%, 35%) if recall is low

**Contingency**: Switch to hybrid search (keyword + semantic) if pure semantic search underperforms.

### Risk 5: Performance Degradation

**Impact**: Medium | **Likelihood**: Low (initially)

**Mitigation**:
- Monitor P95 response times via logging
- Optimize Qdrant index parameters if needed (increase M, ef_construct)
- Reduce retrieval to top-3 if latency >2s

**Contingency**: Upgrade Render to Starter plan ($7/month) for 512MB → 2GB RAM.

### Risk 6: Security Vulnerabilities

**Impact**: High | **Likelihood**: Low

**Mitigation**:
- Validate all inputs (Pydantic models enforce length limits)
- Sanitize user questions (remove HTML, SQL injection patterns)
- Use environment variables for API keys (never in code)
- HTTPS only (enforced by Render and GitHub Pages)

**Contingency**: Add WAF (Cloudflare) if attacks detected; implement API key authentication.

---

## Quality Validation

### Completeness Checklist

- [x] Architecture overview with component diagram
- [x] Data model with entities, schemas, relationships
- [x] API contracts (OpenAPI 3.0 spec)
- [x] Testing strategy (6 categories, 30+ test cases)
- [x] Deployment plan with pre-deployment checklist
- [x] Risk mitigation for 6 identified risks
- [x] Decision logs for 12 technical choices
- [x] Quickstart guide for developers

### RAG Best Practices Alignment

- [x] Chunk size optimized for embeddings (800 tokens, within 512-8191 range)
- [x] Overlap prevents boundary loss (25% overlap)
- [x] Retrieval threshold filters irrelevant chunks (0.7 cosine similarity)
- [x] Context window balanced (top-5 chunks, ~3000 tokens)
- [x] Agent grounding enforced (system prompt + tool-only answering)
- [x] Citations required for transparency
- [x] Embedding model is production-grade (text-embedding-3-large)
- [x] Hallucination prevention multi-layered

### Clarity for Implementation

- [x] Quickstart provides phase-by-phase guide (6 phases, 10-15 hours)
- [x] File structure documented with purpose for each file
- [x] API endpoints documented with request/response examples
- [x] Error handling specified (400, 422, 429, 500, 503)
- [x] Testing plan provides specific test cases
- [x] Deployment steps are actionable (checklist format)

### Low Latency Target (<2s)

- [x] Latency breakdown provided (embedding 300ms, retrieval 80ms, agent 1200ms)
- [x] Performance optimizations identified (HNSW index, in-memory Qdrant, top-5 limit)
- [x] Performance tests included (stress test, P95 measurement)
- [x] Contingency for slow queries (reduce to top-3 chunks, optimize Qdrant)

**Overall Quality Assessment**: ✅ **READY FOR IMPLEMENTATION**

---

## Next Steps

1. **Run `/sp.tasks` command** to generate detailed tasks from this plan
2. **Begin implementation** following `quickstart.md` phases
3. **Iterate on medium-confidence decisions**:
   - Test context window size (3 vs 5 chunks)
   - A/B test chunk sizes (600 vs 800 vs 1000 tokens)
   - Optimize system prompt for hallucination prevention
4. **Monitor and adjust** during deployment week 1-4

---

## Appendix: Key Files Reference

| File | Purpose | Status |
|------|---------|--------|
| `spec.md` | Feature requirements, user stories, success criteria | ✅ Complete |
| `plan.md` | This file (architecture, design, testing, deployment) | ✅ Complete |
| `research.md` | Technical decisions with rationale and alternatives | ✅ Complete |
| `data-model.md` | Entity definitions, Qdrant/Postgres schemas | ✅ Complete |
| `quickstart.md` | Developer implementation guide (phases 1-6) | ✅ Complete |
| `contracts/api-openapi.yaml` | OpenAPI 3.0 specification (3 endpoints) | ✅ Complete |
| `checklists/requirements.md` | Spec quality validation | ✅ Complete |
| `tasks.md` | Implementation tasks (awaiting `/sp.tasks` command) | ⏳ Pending |

---

**Plan Version**: 1.0.0
**Last Updated**: 2025-12-09
**Ready for**: Phase 2 tasks generation via `/sp.tasks` command
