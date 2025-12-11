---
id: 001
title: Create RAG Chatbot Specification
stage: spec
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: asadaligith
command: /sp.specify
labels: ["rag", "chatbot", "docusaurus", "openai", "qdrant", "postgres", "phase-2"]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/spec.md
  - specs/001-rag-chatbot/checklists/requirements.md
tests:
  - Specification quality validation (all items passed)
---

## Prompt

/sp.specify

Project: AI Humanoid Robotics Book — Phase 2 RAG Chatbot

Goal:
Develop and embed a Retrieval-Augmented Generation (RAG) chatbot into the published Docusaurus-based book. The chatbot must answer questions about the book using content stored in a vector database (Qdrant), metadata stored in Neon Serverless Postgres, and AI reasoning handled through the OpenAI Agent SDK.

Context:
Phase 1 (Book creation using Spec-Kit-Plus + Claude Code + Docusaurus) is complete and deployed at:
https://asadaligith.github.io/AI-Humanoid-Robotics-Book/

This specification defines Phase 2 requirements for integrated chatbot development.

Core Requirements:
1. Extract markdown documents from `/docs/**` in the book repository.
2. Chunk and embed content using text-embedding-3-large.
3. Store embeddings + metadata in Qdrant Cloud (Free Tier).
4. Create a FastAPI backend exposing RAG endpoints:
   - POST /ask
   - POST /ask-selected
   - GET /health
5. Implement retrieval logic using Qdrant + Postgres metadata join.
6. Build an OpenAI Agent that:
   - Uses RAG context
   - Uses retrieval tool functions (search_book)
   - Answers ONLY from book content
   - Refuses if content is missing
7. Embed a JS-based chatbot widget in Docusaurus:
   - Floating button
   - Chat panel
   - API requests to backend
   - Optional feature: Ask about highlighted text
8. Deploy FastAPI backend on Render/Fly/Railway.

Functional Expectations:
- Chatbot must answer from book only.
- Responses must cite section headings or file names.
- RAG must work for both general queries and selected-text queries.
- System must avoid hallucinations.

Non-Functional:
- Backend response time under 2 seconds.
- Vector store must handle >300 chunks.
- System must remain low-cost (free tier friendly).

Deliverables:
- High-quality specification
- Architecture diagram description
- Section structure for Phase 2 documentation
- Risk identification
- Acceptance criteria

## Response snapshot

Created comprehensive specification for RAG Chatbot feature (Phase 2 of AI Humanoid Robotics Book project).

**Key Outputs:**
1. **Feature Branch**: Created `001-rag-chatbot` with spec directory structure
2. **Specification Document**: `specs/001-rag-chatbot/spec.md` containing:
   - 3 prioritized user stories (P1: General Q&A, P2: Context-specific queries, P3: Multi-turn conversations)
   - 16 functional requirements covering content extraction, embedding, storage, API, and deployment
   - 8 measurable success criteria (response time, accuracy, cost constraints)
   - 7 edge cases addressing service failures and input validation
   - Clear scope boundaries (out-of-scope items documented)
   - Dependencies, assumptions, and 6 identified risks

3. **Quality Validation**: Created `specs/001-rag-chatbot/checklists/requirements.md`
   - All 13 quality checklist items passed
   - No [NEEDS CLARIFICATION] markers
   - Ready for planning phase

**Architecture Highlights (Requirements-Level):**
- RAG pipeline: `/docs/**` → chunking → embedding (text-embedding-3-large) → Qdrant storage
- Dual storage: Qdrant (vectors) + Neon Postgres (metadata)
- FastAPI backend with 3 endpoints (/ask, /ask-selected, /health)
- OpenAI Agent SDK with search_book tool
- Docusaurus widget integration (floating button + chat panel)
- Free-tier deployment strategy (Qdrant Cloud, Neon, Render/Fly/Railway)

**Success Metrics:**
- <2s response time (P95)
- 300+ chunks indexed
- 90%+ answer accuracy with citations
- 100% refusal rate for off-topic questions
- Zero-cost operation on free tiers

## Outcome

- ✅ Impact: Comprehensive, testable specification created for Phase 2 RAG chatbot feature. Establishes clear requirements, user scenarios, and success criteria for implementation.
- 🧪 Tests: Specification quality checklist validated (13/13 items passed). No clarifications needed.
- 📁 Files: Created 2 files (spec.md, requirements.md checklist) in feature directory 001-rag-chatbot
- 🔁 Next prompts: User can proceed with `/sp.clarify` (if refinement needed) or `/sp.plan` (to create architecture and implementation plan)
- 🧠 Reflection: Specification balances technical precision with business focus. Technology stack mentioned as "informational" rather than prescriptive, keeping spec implementation-agnostic. All requirements are independently testable.

## Evaluation notes (flywheel)

- Failure modes observed: None. Specification workflow completed successfully despite PowerShell script unavailability (manually created branch and directories).
- Graders run and results (PASS/FAIL): Spec Quality Checklist - PASS (13/13 items). All content quality, requirement completeness, and feature readiness criteria met.
- Prompt variant (if applicable): Standard /sp.specify workflow from Spec-Kit-Plus
- Next experiment (smallest change to try): Monitor if the "Ask about selected text" feature (P2) should be elevated to P1 based on user feedback during implementation.
