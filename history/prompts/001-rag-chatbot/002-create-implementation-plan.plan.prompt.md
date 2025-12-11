---
id: 002
title: Create RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: asadaligith
command: /sp.plan
labels: ["rag", "chatbot", "architecture", "fastapi", "qdrant", "openai", "planning"]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/plan.md
  - specs/001-rag-chatbot/research.md
  - specs/001-rag-chatbot/data-model.md
  - specs/001-rag-chatbot/quickstart.md
  - specs/001-rag-chatbot/contracts/api-openapi.yaml
tests:
  - Constitution compliance check (6/6 principles passed)
  - Quality validation checklist (completeness, RAG best practices, clarity, latency targets)
---

## Prompt

/sp.plan

Task: Create a full technical plan for Phase 2 RAG Chatbot integration for the AI Humanoid Robotics Book project.

Plan Requirements:
1. Architecture Overview
   - Describe interactions between Docusaurus frontend, chatbot widget, FastAPI backend, Qdrant vector DB, Neon metadata DB, and the OpenAI Agent SDK.

2. Detailed Section Structure
   - Ingestion pipeline
   - Embeddings & vector storage
   - Retrieval pipeline
   - FastAPI routes
   - Agent SDK integration
   - Frontend chatbot widget integration
   - Deployment plan
   - Testing + validation plan

3. Research-Concurrent Approach
   - For each section, document decisions made while developing plan
   - Evaluate options (e.g., cosine vs euclidean metric, chunk sizes, backend hosting options)

4. Decisions Requiring Documentation
   Provide decision logs for:
   - Chunk size and overlap
   - Embedding model selection
   - Qdrant collection schema
   - API route design
   - RAG context window design
   - Agent SDK tool schema
   - Deployment platform selection
   - Data refresh pipeline (optional)

5. Testing Strategy
   - Retrieval accuracy tests
   - Hallucination prevention tests
   - Selected-text-mode tests
   - Stress test (multiple requests)
   - Integration test: Docusaurus → API → Agent → Qdrant → Response
   - User acceptance tests (UI responsiveness)

6. Quality Validation
   - Ensure completeness
   - Ensure plan follows RAG best practices
   - Ensure clarity for implementation phase
   - Ensure low latency (<2s target)

Output Format:
- Clear, structured plan document
- Use headings, bullet points, tables
- Avoid writing any actual code

## Response snapshot

Created comprehensive technical plan for RAG Chatbot implementation following Spec-Driven Development (SDD) workflow.

**Phase 0: Research & Decisions (research.md)**
- Documented 12 critical technical decisions with rationale, alternatives, and references
- Key decisions: 800-token chunks with 25% overlap, text-embedding-3-large (1536D), Qdrant cosine similarity, FastAPI with 3 RESTful endpoints, top-5 retrieval with 0.7 threshold, OpenAI Agent SDK with search_book tool, Render free tier deployment, manual hash-based refresh pipeline
- All decisions backed by industry best practices (OpenAI docs, Qdrant docs, LangChain, Pinecone RAG guides)
- Confidence ratings: 9 high-confidence, 2 medium-confidence decisions

**Phase 1: Design & Contracts**
1. **data-model.md**:
   - Defined 5 core entities (Document Chunk, Chunks Metadata, Search Query, Retrieval Result, Chat Response)
   - Qdrant schema: Collection "book_content", 1536D vectors, HNSW indexing, payload with 7 fields
   - Postgres schema: chunks_metadata table, 8 columns, 4 indexes, SHA256 deduplication
   - Data flow diagram: Question → Embed → Qdrant Search → Rerank → Agent → Response
   - Scaling considerations: 300 chunks initially, can scale to 1000+ with minor adjustments

2. **contracts/api-openapi.yaml**:
   - OpenAPI 3.0 specification for 3 endpoints (POST /ask, POST /ask-selected, GET /health)
   - Pydantic request/response models with validation rules
   - Error responses (400, 422, 429, 500, 503) with example payloads
   - Security considerations (no auth for MVP, optional API key for future)

3. **quickstart.md**:
   - 6-phase developer implementation guide (10-15 hours total)
   - Phase 1: Infrastructure setup (Qdrant, Neon, OpenAI keys) - 30 min
   - Phase 2: Backend development (services, routes, indexing) - 4-6 hours
   - Phase 3: Frontend widget integration (vanilla JS) - 2-3 hours
   - Phase 4: Deployment (Render, CORS, Docker) - 1-2 hours
   - Phase 5: Testing & validation (unit, integration, UAT) - 2-3 hours
   - Phase 6: Monitoring & iteration (logging, cost tracking, prompt optimization) - ongoing

**plan.md (Main Deliverable)**:
- Architecture overview with system component diagram and data flow visualization
- Technical context: Python 3.11+, FastAPI, OpenAI SDK, Qdrant, Neon Postgres, Render deployment
- Constitution compliance: 6/6 applicable principles passed
- Project structure: Backend (FastAPI), Frontend (Docusaurus widget), detailed file tree
- Testing strategy: 6 categories, 30+ test cases covering retrieval accuracy, hallucination prevention, stress testing, integration, UAT
- Deployment plan: Pre-deployment checklist, backend/frontend deployment steps, monitoring, rollback procedures
- Risk mitigation: 6 risks identified with impact/likelihood ratings and contingency plans
- Latency breakdown: Target <2s (embedding 300ms, Qdrant 80ms, agent 1200ms, network 400ms)

**Quality Validation Results**:
- ✅ Completeness: All requested sections present (architecture, pipelines, testing, deployment)
- ✅ RAG Best Practices: Aligned with industry standards (chunk size, overlap, threshold, context window, grounding)
- ✅ Clarity: Quickstart guide provides actionable phases, file structure documented, API contracts with examples
- ✅ Latency Target: <2s P95 with detailed breakdown and optimization strategies

**Key Technical Highlights**:
- Dual storage strategy: Qdrant (vectors) + Postgres (metadata) for flexibility
- Hallucination prevention: Multi-layer (system prompt + 0.7 threshold + citations + tool enforcement)
- Stateless API design: Session IDs in-memory, no persistent conversation history
- Free-tier friendly: Qdrant 1GB, Neon 3GB, Render 512MB RAM - all within limits for 300 chunks
- Manual indexing: Hash-based deduplication, incremental updates, avoids CI/CD complexity

**Architecture Decisions**:
- Web application structure (backend/frontend separation)
- RESTful API (not GraphQL/WebSocket)
- Vanilla JS widget (not React/Vue)
- Single Qdrant collection (not multi-collection)
- Cosine similarity (not Euclidean)
- Top-5 retrieval with reranking (not top-3 or top-10)

## Outcome

- ✅ Impact: Comprehensive technical plan ready for implementation. Provides clear architecture, design decisions, testing strategy, and deployment procedures. Enables `/sp.tasks` command to generate actionable tasks.
- 🧪 Tests: Constitution check passed (6/6 principles), quality validation passed (completeness, RAG best practices, clarity, latency targets), 30+ test cases defined across 6 categories.
- 📁 Files: Created 5 files (plan.md, research.md, data-model.md, quickstart.md, api-openapi.yaml) totaling ~15,000 words of technical documentation.
- 🔁 Next prompts: User should run `/sp.tasks` to generate detailed implementation tasks from this plan. Then proceed with backend development (Phase 2 in quickstart).
- 🧠 Reflection: Research-concurrent approach enabled informed decisions backed by industry best practices. Multi-layer documentation (research → data model → contracts → plan → quickstart) provides comprehensive blueprint from high-level architecture to implementation details.

## Evaluation notes (flywheel)

- Failure modes observed: None. Planning workflow completed successfully. PowerShell script unavailability handled by manual structure creation.
- Graders run and results (PASS/FAIL):
  - Constitution Check: PASS (6/6 applicable principles)
  - Quality Validation: PASS (completeness, RAG best practices, clarity, latency targets)
  - Spec-Plan Alignment: PASS (all functional requirements addressed in plan)
- Prompt variant (if applicable): Standard /sp.plan workflow from Spec-Kit-Plus
- Next experiment (smallest change to try): Test medium-confidence decisions during implementation (context window: 3 vs 5 chunks, chunk size: 600 vs 800 vs 1000 tokens). A/B test system prompts for hallucination prevention.
