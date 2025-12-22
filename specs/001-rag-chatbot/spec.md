# Feature Specification: RAG Chatbot for AI Humanoid Robotics Book

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Develop and embed a Retrieval-Augmented Generation (RAG) chatbot into the published Docusaurus-based book. The chatbot must answer questions about the book using content stored in a vector database (Qdrant), metadata stored in Neon Serverless Postgres, and AI reasoning handled through the OpenAI Agent SDK."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - General Book Questions (Priority: P1)

A reader visiting the deployed book wants to ask questions about the book's content (e.g., "What are the key components of a humanoid robot?", "How does sensor integration work?"). The chatbot should retrieve relevant content from the book and provide accurate, cited answers.

**Why this priority**: This is the core functionality of the RAG chatbot. Without this, the feature has no value. It represents the most common use case where readers need quick answers without manually searching through the book.

**Independent Test**: Can be fully tested by accessing the book website, opening the chatbot widget, typing a question about any topic covered in the book, and receiving a contextual answer with citations (section names or file paths).

**Acceptance Scenarios**:

1. **Given** a reader is on any page of the book, **When** they click the floating chatbot button, **Then** the chat panel opens with a welcome message
2. **Given** the chat panel is open, **When** the reader types "What is the role of actuators in humanoid robots?" and submits, **Then** the chatbot retrieves relevant content and provides an answer citing specific sections or chapters
3. **Given** the chatbot has provided an answer, **When** the reader asks a follow-up question, **Then** the chatbot maintains conversation context and provides a relevant response
4. **Given** a reader asks a question about content not in the book (e.g., "What's the weather today?"), **When** the chatbot processes the query, **Then** it responds with a message indicating it can only answer questions about the book content

---

### User Story 2 - Context-Specific Questions from Selected Text (Priority: P2)

A reader is reading a specific section and highlights/selects text they find confusing or want to learn more about. They can right-click or use a context menu to ask the chatbot specifically about that selected text, providing more focused answers.

**Why this priority**: This enhances user experience by allowing readers to ask about specific passages without needing to rephrase or copy-paste. It's a high-value feature but not essential for MVP since users can still manually type questions.

**Independent Test**: Can be tested by selecting any paragraph in the book, triggering the "Ask about this" option (via context menu or dedicated button), and receiving an answer specifically addressing the selected content.

**Acceptance Scenarios**:

1. **Given** a reader has selected text on a book page, **When** they click "Ask Chatbot" from the context menu or a floating button, **Then** the chatbot panel opens with the selected text pre-populated as context
2. **Given** the chatbot receives a query with selected text context, **When** it retrieves content, **Then** it prioritizes chunks related to the selected passage and provides a focused answer
3. **Given** a reader selects code snippets or technical diagrams, **When** they ask about the selection, **Then** the chatbot provides explanations specific to that code or diagram

---

### User Story 3 - Multi-Turn Conversations (Priority: P3)

A reader wants to have an ongoing conversation with the chatbot, asking multiple related questions to progressively understand a complex topic (e.g., first asking "What is computer vision?", then "How is it used in robotics?", then "What sensors are needed?").

**Why this priority**: While valuable for deep learning, this is an enhancement over basic Q&A. Readers can still get value from single-turn interactions. Multi-turn context improves user experience but isn't critical for initial launch.

**Independent Test**: Can be tested by asking a sequence of related questions (e.g., 3-5 questions building on each other) and verifying the chatbot maintains topical coherence and references previous questions appropriately.

**Acceptance Scenarios**:

1. **Given** a reader has asked a question about "motor control", **When** they follow up with "How does this relate to balance?", **Then** the chatbot understands the context and provides an answer connecting motor control to balance
2. **Given** a conversation has multiple turns, **When** the reader uses pronouns or references like "it" or "that concept", **Then** the chatbot correctly resolves these references based on conversation history
3. **Given** a long conversation (>10 turns), **When** the reader asks a new unrelated question, **Then** the chatbot handles the topic shift appropriately without being overly constrained by old context

---

### Edge Cases

- **What happens when the book content hasn't been indexed yet?** System should display a friendly error message indicating the chatbot is not yet ready.
- **What happens when Qdrant or Postgres services are unavailable?** Chatbot should gracefully degrade with a message like "The chatbot is temporarily unavailable. Please try again later."
- **What happens when a reader asks very long questions (>500 words)?** System should handle gracefully, either processing the question or politely asking the user to shorten it.
- **What happens when multiple users query simultaneously?** Backend should handle concurrent requests without timeouts or failures (up to reasonable limits based on deployment tier).
- **What happens when the OpenAI API rate limit is exceeded?** System should queue requests or display a message asking users to try again in a moment.
- **What happens when selected text is extremely long (>1000 words)?** System should either chunk it or ask the user to select a more specific passage.
- **How does the system handle ambiguous questions?** Chatbot should ask clarifying questions or provide answers for multiple interpretations.
- **What happens when a user sends a conversational greeting instead of a question?** System should detect greetings ("hi", "hello", "hey", etc.) and respond with a friendly welcome message explaining chatbot capabilities and prompting the user to ask book-related questions, WITHOUT performing RAG retrieval or consuming API quota for embeddings.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST extract all markdown documents from the `/docs/**` directory in the book repository
- **FR-002**: System MUST chunk extracted content into semantic segments suitable for embedding (recommended 500-1000 tokens per chunk with overlap)
- **FR-003**: System MUST generate embeddings for each chunk using the `text-embedding-3-large` model from OpenAI
- **FR-004**: System MUST store embeddings in Qdrant Cloud (Free Tier) with associated metadata (file path, section heading, chunk index)
- **FR-005**: System MUST store structured metadata (file paths, section headings, timestamps, chunk relationships) in Neon Serverless Postgres
- **FR-006**: System MUST provide a FastAPI backend with the following endpoints:
  - `POST /ask` - accepts user questions and returns AI-generated answers
  - `POST /ask-selected` - accepts user questions with selected text context
  - `GET /health` - returns service health status
- **FR-007**: System MUST implement retrieval logic that queries Qdrant for semantically similar chunks and joins with Postgres metadata
- **FR-008**: System MUST build an OpenAI Agent using the OpenAI Agent SDK that uses retrieved context to generate answers
- **FR-009**: OpenAI Agent MUST be configured with a `search_book` tool/function that performs RAG retrieval
- **FR-010**: System MUST ensure the chatbot answers substantive questions ONLY from book content and refuses to answer off-topic questions outside this scope; conversational responses (greetings, clarifications, error messages, usage guidance) MAY be provided outside the RAG pipeline
- **FR-011**: System MUST include citations in responses (section headings, chapter names, or file paths) for transparency
- **FR-012**: System MUST embed a JavaScript-based chatbot widget in the Docusaurus site with:
  - A floating button/icon to open the chat
  - A chat panel UI for message display and input
  - API integration to communicate with the FastAPI backend
- **FR-013**: Chatbot widget MUST support the "Ask about selected text" feature (optional enhancement, can be P2/P3)
- **FR-014**: System MUST deploy the FastAPI backend on a cloud platform (Render, Fly.io, or Railway)
- **FR-015**: System MUST avoid hallucinations by grounding all answers strictly in retrieved content
- **FR-016**: System MUST handle cases where no relevant content is found by responding with "I don't have information about that in the book"
- **FR-017**: System MUST detect and respond appropriately to conversational greetings (e.g., "hi", "hello", "hey", "good morning", "good afternoon") with a friendly welcome message explaining chatbot capabilities and prompting user to ask book-related questions, WITHOUT triggering RAG retrieval
- **FR-018**: System MUST format citations in responses with clear section references including module name, chapter name, and optionally file path; citations MAY include hyperlinks to source documentation sections if technically feasible

### Key Entities *(include if feature involves data)*

- **Document Chunk**: Represents a segment of book content. Attributes include chunk text, embedding vector (1536 dimensions for text-embedding-3-large), source file path, section heading, and chunk index within the document.
- **Metadata Record**: Stored in Postgres. Represents structured information about each chunk including file path, section heading, creation timestamp, content hash, and relationships to parent/sibling chunks.
- **Chat Message**: Represents a single user query or bot response. Attributes include message text, timestamp, user session ID, and retrieved context references.
- **Retrieval Result**: Represents the output of a RAG query. Contains retrieved chunks, similarity scores, and metadata used to generate the final answer.
- **Embedding**: Vector representation of text chunks (1536-dimensional float array) stored in Qdrant for similarity search.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Readers can submit a question and receive a relevant, cited answer within 2 seconds (95th percentile response time)
- **SC-002**: System successfully handles at least 300 unique book content chunks indexed in the vector database
- **SC-003**: Chatbot provides accurate answers (verified through manual testing of 20+ diverse questions) with citations in 90%+ of cases
- **SC-004**: Chatbot correctly refuses to answer non-book questions (verified through testing 10+ off-topic questions) in 100% of cases
- **SC-005**: System remains operational with zero critical downtime during the deployment period, using free-tier services (Qdrant Cloud Free, Neon Free, Render/Fly/Railway free tiers)
- **SC-006**: Readers can open the chatbot widget, type a question, and receive a response in 3 clicks or less
- **SC-007**: Multi-turn conversations (3+ turns) maintain context coherence in 80%+ of test cases
- **SC-008**: Selected text feature (if implemented) allows users to get focused answers in 2 clicks or less (select text → ask)
- **SC-009**: System MUST provide non-empty, relevant answers for 95%+ of book-related queries with proper citations; greeting responses MUST return within 500ms without RAG retrieval overhead

## Assumptions

- The book's markdown content is already organized in the `/docs/**` directory with clear section headings and structure
- Readers have modern web browsers with JavaScript enabled
- The deployment platform (Render/Fly/Railway) provides sufficient compute resources under free tier for expected traffic (estimated <1000 queries/day)
- OpenAI API access is available with sufficient quota for embeddings and chat completions
- Qdrant Cloud Free Tier provides sufficient storage for ~300 chunks (estimated ~1-2MB of vectors)
- Neon Serverless Postgres Free Tier provides sufficient storage for metadata (estimated <100MB)
- The Docusaurus site supports custom JavaScript integration for the chatbot widget
- Content updates to the book will require re-indexing (manual or scripted process)

## Dependencies

- **External Services**:
  - OpenAI API (for embeddings via `text-embedding-3-large` and chat completions)
  - Qdrant Cloud (vector database for embeddings)
  - Neon Serverless Postgres (metadata storage)
  - Deployment platform (Render, Fly.io, or Railway for hosting FastAPI backend)
- **Existing Systems**:
  - Docusaurus-based book deployment (Phase 1) at https://asadaligith.github.io/AI-Humanoid-Robotics-Book/
  - Book repository with `/docs/**` markdown content
- **Technology Stack** (informational, not prescriptive):
  - FastAPI (backend framework)
  - OpenAI Agent SDK (for agent implementation)
  - Qdrant Python client
  - PostgreSQL client (e.g., psycopg2 or asyncpg)
  - JavaScript/TypeScript for Docusaurus widget

## Out of Scope

- **Admin dashboard**: No UI for managing indexed content or viewing analytics
- **User authentication**: Chatbot is publicly accessible without login
- **Multi-language support**: Chatbot operates in English only (same as book content)
- **Voice input/output**: Text-based interaction only
- **Mobile app**: Web-only deployment through Docusaurus
- **Content versioning**: No tracking of book version changes or rollback capabilities
- **Advanced analytics**: No user behavior tracking, query analytics, or feedback collection
- **Custom embedding models**: Only using `text-embedding-3-large` from OpenAI
- **Real-time content updates**: Indexing is a separate, manual/scripted process
- **Integration with other book platforms**: Only integrated with the Docusaurus deployment

## Risks

1. **Cost overruns**: While using free tiers, unexpected traffic spikes could exceed quota limits. Mitigation: Implement rate limiting and monitor usage closely.
2. **API availability**: Dependency on third-party services (OpenAI, Qdrant, Neon) means outages affect the chatbot. Mitigation: Implement graceful error handling and fallback messages.
3. **Hallucination despite safeguards**: Even with strict grounding, LLMs may occasionally generate incorrect information. Mitigation: Include disclaimers and encourage users to verify critical information.
4. **Poor retrieval quality**: If chunking strategy is suboptimal, relevant content may not be retrieved. Mitigation: Test multiple chunking strategies during implementation and allow for iteration.
5. **Performance degradation**: If the book grows significantly (>1000 chunks), retrieval speed may suffer. Mitigation: Monitor performance and consider upgrading to paid tiers if needed.
6. **Security vulnerabilities**: Prompt injection or malicious queries could exploit the system. Mitigation: Implement input validation and sanitization in the backend.
