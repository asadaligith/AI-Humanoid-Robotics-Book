---
description: "Implementation tasks for RAG Chatbot feature"
---

# Tasks: RAG Chatbot for AI Humanoid Robotics Book

**Input**: Design documents from `/specs/001-rag-chatbot/`
**Prerequisites**: plan.md (✅), spec.md (✅), research.md (✅), data-model.md (✅), contracts/ (✅)

**Tests**: Tests are NOT explicitly requested in spec. Test tasks included for critical paths only (retrieval accuracy, hallucination prevention, integration).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure** (per plan.md): `backend/src/`, `frontend/static/js/`
- Backend uses Python 3.11+, FastAPI
- Frontend uses vanilla JavaScript (Docusaurus integration)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment configuration

- [X] T001 Create backend project structure with directories: backend/src/{models,services,routes,utils}, backend/tests/, backend/scripts/
- [X] T002 Initialize Python project with requirements.txt containing: fastapi==0.109.0, uvicorn==0.27.0, pydantic==2.5.3, openai==1.12.0, qdrant-client==1.7.0, psycopg2-binary==2.9.9, sqlalchemy==2.0.25, tiktoken==0.5.2, langchain==0.1.5
- [X] T003 [P] Create .env.example file with template variables: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL
- [X] T004 [P] Create Dockerfile for backend with Python 3.11-slim base image per plan.md deployment section
- [X] T005 [P] Create render.yaml with service config, health check path /health, environment variable placeholders
- [X] T006 [P] Setup pytest configuration in backend/tests/conftest.py with fixtures for test database and mock services

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

**NOTE**: Using **Google Gemini** instead of OpenAI (768D vectors instead of 1536D)

- [X] T007 Create Qdrant collection "book_content" with **768D vectors** (Gemini text-embedding-004), cosine distance, HNSW index (M=16, ef_construct=100) - **MANUAL SETUP REQUIRED**
- [X] T008 Create Neon Postgres database and execute schema from data-model.md: chunks_metadata table with indexes on file_path, section_heading, content_hash, created_at - **MANUAL SETUP REQUIRED**
- [X] T009 [P] Implement config.py in backend/src/ using Pydantic Settings to load environment variables (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL)
- [X] T010 [P] Implement database.py in backend/src/services/ with SQLAlchemy engine and session factory for Neon Postgres connection
- [X] T011 [P] Implement embeddings.py in backend/src/services/ with embed_text() function using **Gemini text-embedding-004** model (768D)
- [X] T012 [P] Implement chunking.py in backend/src/utils/ using LangChain RecursiveCharacterTextSplitter with 800 tokens, 200 overlap, markdown-aware splitting
- [X] T013 [P] Implement hashing.py in backend/src/utils/ with compute_hash() function using SHA256 for content deduplication
- [X] T014 [P] Create Pydantic request models in backend/src/models/requests.py: AskRequest (question, session_id), AskSelectedRequest (question, selected_text, session_id)
- [X] T015 [P] Create Pydantic response models in backend/src/models/responses.py: ChatResponse (answer, sources, session_id), HealthResponse (status, services, version, timestamp), ErrorResponse (error, code, details)
- [X] T021 [P] [US1] Implement retrieval.py in backend/src/services/ with search() function to query Qdrant for top-10 chunks, rerank by cosine similarity, filter by threshold >=0.7, return top-5
- [X] T022 [US1] Implement agent.py in backend/src/services/ using **Gemini 1.5 Pro** with RAG, system prompt enforcing grounding ("ONLY use retrieved content"), citation requirement
- [X] T023 [US1] Implement session.py in backend/src/utils/ with in-memory session management using UUID for multi-turn conversation tracking
- [X] T016 Implement reindex_book.py script in backend/scripts/ to read /docs/**, chunk, hash, embed, and store in Qdrant + Postgres with idempotent upsert logic
- [ ] T017 Run reindex_book.py script to index book content (estimated 300 chunks from /docs/** directory)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - General Book Questions (Priority: P1) 🎯 MVP

**Goal**: Enable readers to ask questions about book content and receive accurate, cited answers via chatbot widget

**Independent Test**: Access book website, open chatbot widget, ask "What are the key components of a humanoid robot?", receive answer with citations from relevant sections within 2 seconds

### Tests for User Story 1 (Critical Paths Only)

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T018 [P] [US1] Create retrieval accuracy test in backend/tests/test_retrieval.py to verify top-5 chunks retrieved for query "robot sensors" are from sensor-related chapters (>0.7 similarity)
- [ ] T019 [P] [US1] Create hallucination prevention test in backend/tests/test_agent.py to verify off-topic query "weather in New York" returns refusal message "I don't have information about that in the book"
- [ ] T020 [P] [US1] Create integration test in backend/tests/test_api.py for POST /ask endpoint to verify end-to-end flow: question → embedding → retrieval → agent → response with citations

### Implementation for User Story 1

- [ ] T021 [P] [US1] Implement retrieval.py in backend/src/services/ with search() function to query Qdrant for top-10 chunks, rerank by cosine similarity, filter by threshold >=0.7, return top-5
- [ ] T022 [US1] Implement agent.py in backend/src/services/ using OpenAI Agent SDK with search_book tool, system prompt enforcing grounding ("ONLY use search_book tool results"), citation requirement
- [ ] T023 [US1] Implement session.py in backend/src/utils/ with in-memory session management using UUID for multi-turn conversation tracking
- [ ] T024 [US1] Implement POST /ask endpoint in backend/src/routes/chat.py: validate request, embed question, call retrieval service, call agent service, return ChatResponse with citations
- [ ] T025 [US1] Implement GET /health endpoint in backend/src/routes/health.py to check connectivity to Qdrant, Postgres, OpenAI API, return HealthResponse with service status
- [ ] T026 [US1] Create main.py in backend/src/ to initialize FastAPI app, register routes, add CORS middleware for https://asadaligith.github.io domain, configure error handlers
- [ ] T027 [US1] Add request timeout (10s) and rate limiting (10 requests/minute per session_id) middleware in backend/src/main.py
- [ ] T028 [P] [US1] Create chatbot-widget.js in frontend/static/js/ with Web Component: floating button, chat panel UI, message display, input field, submit handler
- [ ] T029 [US1] Implement API communication in chatbot-widget.js: fetch() calls to POST /ask endpoint, handle responses, display answer with clickable citations
- [ ] T030 [US1] Create chatbot-styles.css in frontend/static/js/ using Docusaurus theme variables for consistent styling, responsive design for mobile/desktop
- [ ] T031 [US1] Update docusaurus.config.js to include chatbot-widget.js and chatbot-styles.css in scripts and stylesheets arrays

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. Readers can ask general questions and receive cited answers.

---

## Phase 4: User Story 2 - Context-Specific Questions from Selected Text (Priority: P2)

**Goal**: Enable readers to highlight text and ask focused questions about the selected passage

**Independent Test**: Select paragraph about "inverse kinematics" in book, click "Ask Chatbot" button, ask "Explain this in simpler terms", receive answer prioritizing the selected text context

### Tests for User Story 2

- [ ] T032 [P] [US2] Create selected-text mode test in backend/tests/test_api.py for POST /ask-selected endpoint to verify selected text boosts relevance of related chunks (cosine similarity with selected text embedding)

### Implementation for User Story 2

- [ ] T033 [P] [US2] Extend retrieval.py search() function with optional selected_text parameter to compute hybrid score: 0.7 * query_similarity + 0.3 * selected_text_similarity
- [ ] T034 [US2] Implement POST /ask-selected endpoint in backend/src/routes/chat.py: validate AskSelectedRequest (max 5000 chars for selected_text), embed both question and selected text, call retrieval with hybrid scoring, return ChatResponse
- [ ] T035 [US2] Add selected text handler to chatbot-widget.js: listen for text selection events, show "Ask about this" floating button near selection, pre-populate chat panel with selected context
- [ ] T036 [US2] Update chatbot-widget.js submit handler to detect selected text mode, call POST /ask-selected instead of POST /ask, display selected text snippet in chat UI

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Readers can ask general questions OR ask about selected text.

---

## Phase 5: User Story 3 - Multi-Turn Conversations (Priority: P3)

**Goal**: Enable readers to have ongoing conversations with context retention across multiple questions

**Independent Test**: Ask sequence of related questions: "What is computer vision?" → "How is it used in robotics?" → "What sensors are needed?", verify chatbot maintains topical coherence and references previous questions

### Tests for User Story 3

- [ ] T037 [P] [US3] Create multi-turn conversation test in backend/tests/test_agent.py to verify 3 sequential related questions maintain context and chatbot correctly resolves pronouns ("it", "that concept")

### Implementation for User Story 3

- [ ] T038 [US3] Extend session.py to store conversation history (list of {role, content} messages) per session_id with max 10 turns, implement LRU eviction for old sessions (>30 min idle)
- [ ] T039 [US3] Update agent.py to accept conversation_history parameter, pass to OpenAI Agent SDK as message context, enable agent to reference previous questions/answers
- [ ] T040 [US3] Update POST /ask and POST /ask-selected endpoints in backend/src/routes/chat.py to retrieve conversation history from session, pass to agent, append new Q&A to history before returning
- [ ] T041 [US3] Update chatbot-widget.js to persist session_id in browser sessionStorage, include in all API requests, display conversation history in chat panel

**Checkpoint**: All user stories should now be independently functional. Readers can ask general questions, selected-text questions, and multi-turn conversations.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and deployment readiness

- [ ] T042 [P] Add structured logging to all services (embeddings, retrieval, agent) in backend/src/services/ with request_id, latency_ms, error tracking
- [ ] T043 [P] Implement retry logic with exponential backoff for OpenAI API calls in embeddings.py and agent.py to handle transient failures (429, 503 errors)
- [ ] T044 [P] Add input validation and sanitization in chat.py routes to prevent injection attacks (strip HTML tags, SQL patterns from user questions)
- [ ] T045 [P] Create comprehensive error messages for common failures in chat.py: Qdrant unavailable → "Chatbot temporarily unavailable", no results found → "I don't have information about that", rate limit → "Please try again in 60 seconds"
- [ ] T046 Add performance monitoring to track P95 response times, log slow queries (>2s) in backend/src/routes/chat.py
- [ ] T047 Create deployment checklist in backend/README.md covering: environment variables set, Qdrant collection created, Postgres schema deployed, 300 chunks indexed, health endpoint returns 200, CORS configured
- [ ] T048 Add widget loading optimization in chatbot-widget.js: lazy load (initialize only when button clicked), minify JavaScript, use requestAnimationFrame for smooth animations
- [ ] T049 [P] Create backend/tests/test_chunking.py unit tests to verify 800-token chunks with 200-token overlap, markdown header preservation
- [ ] T050 [P] Create backend/tests/test_stress.py to send 10 concurrent requests and verify <5% error rate, all responses <5s
- [ ] T051 Run quickstart.md validation: verify all 6 phases execute successfully, total time 10-15 hours, all success criteria met

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion (T007-T017)
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Extends US1 retrieval but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Extends US1 agent but independently testable

### Within Each User Story

- Tests MUST be written and FAIL before implementation (T018-T020 before T021+, T032 before T033+, T037 before T038+)
- Models/Services before routes (T021-T023 before T024-T027)
- Backend before frontend (T021-T027 before T028-T031)
- Core implementation before integration (T024-T027 before T028-T031)

### Critical Path (Blocks MVP)

**Foundational Phase (Phase 2)** is the critical path:
- T007 (Qdrant collection) blocks all retrieval tasks
- T008 (Postgres schema) blocks indexing and metadata storage
- T009-T015 (config, database, embeddings, chunking, models) block all services
- T016-T017 (indexing script + execution) blocks retrieval accuracy

Until T007-T017 complete, NO user story work can begin.

### Parallel Opportunities

- **Setup Phase (Phase 1)**: T003, T004, T005, T006 can run in parallel after T001-T002
- **Foundational Phase (Phase 2)**: T009, T010, T011, T012, T013, T014, T015 can all run in parallel after T007-T008
- **User Story 1 Tests**: T018, T019, T020 can run in parallel
- **User Story 1 Models/Services**: T021, T023 can run in parallel
- **User Story 1 Frontend**: T028, T030 can run in parallel after T024-T027
- **User Story 2 Test**: T032 runs independently
- **User Story 3 Test**: T037 runs independently
- **Polish Phase (Phase 6)**: T042, T043, T044, T045, T049, T050 can all run in parallel

**Between User Stories**: US2 (T032-T036) and US3 (T037-T041) can run in parallel after US1 completes if team capacity allows.

---

## Parallel Example: User Story 1

```bash
# After Foundational Phase completes, launch User Story 1 tests in parallel:
Task T018: "Create retrieval accuracy test in backend/tests/test_retrieval.py"
Task T019: "Create hallucination prevention test in backend/tests/test_agent.py"
Task T020: "Create integration test in backend/tests/test_api.py"

# After tests written (and failing), launch services in parallel:
Task T021: "Implement retrieval.py in backend/src/services/"
Task T023: "Implement session.py in backend/src/utils/"

# After services complete, implement routes sequentially:
Task T024: "Implement POST /ask endpoint in backend/src/routes/chat.py"
Task T025: "Implement GET /health endpoint in backend/src/routes/health.py"
Task T026: "Create main.py in backend/src/"

# After backend routes complete, launch frontend tasks in parallel:
Task T028: "Create chatbot-widget.js in frontend/static/js/"
Task T030: "Create chatbot-styles.css in frontend/static/js/"
```

---

## Parallel Example: User Stories 2 and 3 (After US1)

```bash
# If team has capacity, launch US2 and US3 in parallel after US1 completes:

# Developer A works on US2:
Task T032: "Create selected-text mode test"
Task T033: "Extend retrieval.py with selected_text parameter"
Task T034: "Implement POST /ask-selected endpoint"
Task T035: "Add selected text handler to chatbot-widget.js"
Task T036: "Update submit handler for selected text mode"

# Developer B works on US3 (parallel to Developer A):
Task T037: "Create multi-turn conversation test"
Task T038: "Extend session.py to store conversation history"
Task T039: "Update agent.py to accept conversation_history"
Task T040: "Update endpoints to use conversation history"
Task T041: "Update widget to persist session_id"

# Both streams complete independently and integrate seamlessly
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

**Recommended for initial launch**:

1. Complete **Phase 1: Setup** (T001-T006) → ~30 minutes
2. Complete **Phase 2: Foundational** (T007-T017) → ~4 hours (including indexing)
   - **CRITICAL CHECKPOINT**: Verify Qdrant collection has 300 chunks, Postgres has 300 rows, health checks pass
3. Complete **Phase 3: User Story 1** (T018-T031) → ~6 hours
   - Write tests first (T018-T020), verify they FAIL
   - Implement backend (T021-T027)
   - Implement frontend (T028-T031)
4. **STOP and VALIDATE**:
   - Run all US1 tests (should now PASS)
   - Manual test: Ask 10 diverse questions, verify answers with citations
   - Performance test: Verify <2s P95 response time
5. Deploy MVP to Render + GitHub Pages
6. Collect user feedback before adding US2/US3

**Total MVP Time**: ~10-11 hours

### Incremental Delivery (Add Features After MVP)

**After MVP is live and validated**:

1. **Iteration 2**: Add User Story 2 (Selected Text) - Priority P2
   - Complete Phase 4 (T032-T036) → ~2 hours
   - Test independently (US1 should still work)
   - Deploy update

2. **Iteration 3**: Add User Story 3 (Multi-Turn) - Priority P3
   - Complete Phase 5 (T037-T041) → ~2 hours
   - Test independently (US1 and US2 should still work)
   - Deploy update

3. **Iteration 4**: Polish and optimize
   - Complete Phase 6 (T042-T051) → ~3 hours
   - Deploy final version

**Total Time for All Features**: ~18-20 hours

### Parallel Team Strategy

With 2 developers:

1. **Both developers**: Complete Setup + Foundational together (Phases 1-2) → ~4.5 hours
2. **Once Foundational complete**:
   - **Developer A**: User Story 1 (Phase 3) → ~6 hours
   - **Developer B**: Can prepare tests/infrastructure for US2/US3
3. **After US1 deployed**:
   - **Developer A**: User Story 2 (Phase 4) → ~2 hours
   - **Developer B**: User Story 3 (Phase 5) → ~2 hours (parallel)
4. **Both developers**: Polish (Phase 6) together → ~3 hours

**Total Time with 2 Developers**: ~10-12 hours calendar time

---

## Task Count Summary

- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 11 tasks ⚠️ CRITICAL PATH
- **Phase 3 (User Story 1 - MVP)**: 14 tasks (3 tests + 11 implementation)
- **Phase 4 (User Story 2)**: 5 tasks (1 test + 4 implementation)
- **Phase 5 (User Story 3)**: 5 tasks (1 test + 4 implementation)
- **Phase 6 (Polish)**: 10 tasks

**Total**: 51 tasks

**Parallel Opportunities**: 18 tasks marked [P] can run in parallel within their phase

**MVP Scope**: Phases 1-3 (31 tasks) = Core chatbot functionality

---

## Notes

- **[P] tasks**: Different files, no dependencies, can run in parallel
- **[Story] label**: Maps task to specific user story (US1, US2, US3) for traceability
- **Test-first approach**: Write tests (T018-T020, T032, T037) BEFORE implementation, verify they FAIL
- **Independent stories**: Each user story (US1, US2, US3) can be tested independently
- **Commit strategy**: Commit after completing each phase or logical task group
- **Stop at checkpoints**: Validate story independently before proceeding to next priority
- **Avoid**: Vague tasks, same-file conflicts, cross-story dependencies that break independence

**Critical Success Factors**:
1. Complete Foundational Phase (T007-T017) fully before any user story work
2. Index book content successfully (300 chunks in Qdrant + Postgres)
3. Verify tests FAIL before implementation (TDD discipline)
4. Test each user story independently before moving to next
5. Monitor <2s P95 response time throughout development
6. Deploy MVP (US1 only) first, validate with real users, then add US2/US3

**Risk Mitigation**:
- If indexing (T016-T017) fails: Debug chunk count, verify Qdrant/Postgres connectivity before proceeding
- If tests don't fail (T018-T020): Review test logic, ensure no implementation leaked in
- If response time >2s: Profile bottleneck (embedding, retrieval, agent), optimize critical path first
- If hallucination detected: Strengthen system prompt (T022), increase threshold to 0.75 (T021), reduce context window to top-3
