# Selected Text Feature Implementation Guide

**Feature**: User Story 2 - Context-Specific Questions from Selected Text
**Priority**: P2 - Enhancement
**Status**: Implementation Guide

---

## Overview

This feature allows users to highlight/select text in the documentation and ask focused questions about that specific passage. The chatbot will prioritize the selected text context when retrieving relevant chunks.

**Value Proposition**:
- Users can ask "Explain this" without retyping complex passages
- More focused answers for specific sections
- Better UX for technical documentation

---

## T032: Selected-Text Mode Test

### Test Specification

**File**: `backend/tests/test_api.py`

```python
import pytest
from httpx import AsyncClient
from src.main import app

@pytest.mark.asyncio
async def test_ask_selected_endpoint_boosts_selected_text_relevance():
    """
    Test that selected text increases relevance of related chunks.

    Verify that when a user selects text and asks a question:
    1. Selected text is embedded separately
    2. Retrieval uses hybrid scoring (query + selected_text)
    3. Results prioritize chunks semantically similar to selected text
    """
    async with AsyncClient(app=app, base_url="http://test") as client:
        # Test case: User selects paragraph about "inverse kinematics"
        selected_text = """
        Inverse kinematics (IK) is the mathematical process of calculating
        the joint angles needed to place a robot's end-effector at a desired
        position and orientation. Unlike forward kinematics, which computes
        the end-effector pose from joint angles, IK solves the reverse problem.
        """

        question = "Explain this in simpler terms"

        response = await client.post(
            "/ask-selected",
            json={
                "question": question,
                "selected_text": selected_text,
                "session_id": "test-session-123"
            }
        )

        assert response.status_code == 200
        data = response.json()

        # Verify response structure
        assert "answer" in data
        assert "sources" in data
        assert "session_id" in data

        # Verify answer references selected text concepts
        answer = data["answer"].lower()
        assert any(term in answer for term in [
            "inverse kinematics",
            "joint angles",
            "end-effector",
            "ik"
        ])

        # Verify sources include kinematics-related chapters
        sources = data["sources"]
        assert len(sources) > 0
        assert any("kinematics" in src.get("file_path", "").lower()
                   for src in sources)


@pytest.mark.asyncio
async def test_ask_selected_validates_text_length():
    """Test that selected text is validated for maximum length."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # Create overly long selected text (>5000 chars)
        selected_text = "A" * 6000

        response = await client.post(
            "/ask-selected",
            json={
                "question": "What is this?",
                "selected_text": selected_text,
                "session_id": "test-session-123"
            }
        )

        assert response.status_code == 422  # Validation error
        data = response.json()
        assert "selected_text" in str(data).lower()
        assert "5000" in str(data)  # Max length mentioned in error


@pytest.mark.asyncio
async def test_ask_selected_handles_empty_selection():
    """Test graceful handling of empty or whitespace-only selection."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/ask-selected",
            json={
                "question": "What does this mean?",
                "selected_text": "   ",  # Only whitespace
                "session_id": "test-session-123"
            }
        )

        # Should either reject or fall back to regular /ask behavior
        assert response.status_code in [200, 422]

        if response.status_code == 200:
            # If accepted, should work like regular query
            data = response.json()
            assert "answer" in data


@pytest.mark.asyncio
async def test_ask_selected_hybrid_scoring():
    """
    Test that hybrid scoring combines query and selected text similarity.

    Formula: 0.7 * query_similarity + 0.3 * selected_text_similarity
    """
    async with AsyncClient(app=app, base_url="http://test") as client:
        # Select text about ROS 2 topics
        selected_text = "ROS 2 uses a publisher-subscriber pattern for communication."

        # Ask unrelated question (should still retrieve topic-related content)
        question = "How does navigation work?"

        response = await client.post(
            "/ask-selected",
            json={
                "question": question,
                "selected_text": selected_text,
                "session_id": "test-session-123"
            }
        )

        assert response.status_code == 200
        data = response.json()

        # Even though question is about navigation,
        # answer should reference publisher-subscriber from selected text
        answer = data["answer"].lower()
        assert "publisher" in answer or "subscriber" in answer or "topic" in answer
```

### Running the Tests

```bash
# Run all selected-text tests
pytest backend/tests/test_api.py::test_ask_selected_endpoint_boosts_selected_text_relevance -v
pytest backend/tests/test_api.py::test_ask_selected_validates_text_length -v
pytest backend/tests/test_api.py::test_ask_selected_handles_empty_selection -v
pytest backend/tests/test_api.py::test_ask_selected_hybrid_scoring -v

# Or run all at once
pytest backend/tests/test_api.py -k "test_ask_selected" -v
```

---

## T033: Extend retrieval.py with Hybrid Scoring

### Implementation

**File**: `backend/src/services/retrieval.py`

```python
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, Filter, FieldCondition, Range
import numpy as np

class RetrievalService:
    def __init__(self, qdrant_client: QdrantClient):
        self.client = qdrant_client
        self.collection_name = "book_content"

    async def search(
        self,
        query_embedding: List[float],
        selected_text_embedding: Optional[List[float]] = None,
        limit: int = 10,
        threshold: float = 0.65,
        hybrid_weights: tuple = (0.7, 0.3)
    ) -> List[Dict]:
        """
        Search for relevant chunks with optional hybrid scoring.

        Args:
            query_embedding: Vector embedding of user's question
            selected_text_embedding: Optional embedding of selected text
            limit: Number of chunks to retrieve
            threshold: Minimum similarity score
            hybrid_weights: (query_weight, selected_text_weight) for hybrid mode

        Returns:
            List of relevant chunks with metadata and scores
        """
        # Step 1: Retrieve top chunks based on query
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit * 2,  # Get extra for reranking
            score_threshold=threshold - 0.05  # Slightly lower threshold for initial search
        )

        # Step 2: If selected text provided, apply hybrid scoring
        if selected_text_embedding is not None:
            search_results = self._apply_hybrid_scoring(
                results=search_results,
                query_embedding=query_embedding,
                selected_text_embedding=selected_text_embedding,
                weights=hybrid_weights
            )

        # Step 3: Rerank by final score
        search_results = sorted(
            search_results,
            key=lambda x: x.score,
            reverse=True
        )

        # Step 4: Filter by threshold and limit
        filtered_results = [
            r for r in search_results
            if r.score >= threshold
        ][:limit]

        # Step 5: Format results
        return self._format_results(filtered_results)

    def _apply_hybrid_scoring(
        self,
        results: List,
        query_embedding: List[float],
        selected_text_embedding: List[float],
        weights: tuple
    ) -> List:
        """
        Apply hybrid scoring: weighted combination of query and selected text similarity.

        Formula: final_score = w1 * query_sim + w2 * selected_text_sim
        """
        query_weight, selected_weight = weights
        query_vec = np.array(query_embedding)
        selected_vec = np.array(selected_text_embedding)

        for result in results:
            # Get chunk embedding
            chunk_vec = np.array(result.vector)

            # Compute cosine similarities
            query_sim = self._cosine_similarity(query_vec, chunk_vec)
            selected_sim = self._cosine_similarity(selected_vec, chunk_vec)

            # Hybrid score
            result.score = (query_weight * query_sim) + (selected_weight * selected_sim)

        return results

    def _cosine_similarity(self, vec1: np.ndarray, vec2: np.ndarray) -> float:
        """Compute cosine similarity between two vectors."""
        return np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))

    def _format_results(self, results: List) -> List[Dict]:
        """Format search results into standardized dict format."""
        formatted = []
        for result in results:
            formatted.append({
                "id": result.id,
                "score": float(result.score),
                "text": result.payload.get("text", ""),
                "file_path": result.payload.get("file_path", ""),
                "section_heading": result.payload.get("section_heading", ""),
                "module": result.payload.get("module", ""),
                "chapter": result.payload.get("chapter", ""),
                "chunk_index": result.payload.get("chunk_index", 0)
            })
        return formatted
```

### Configuration

**Hybrid Scoring Weights**:
- **Query Weight**: 0.7 (70%) - Primary relevance to user's question
- **Selected Text Weight**: 0.3 (30%) - Context from selected passage

**Rationale**: The question is still the primary signal, but selected text provides additional context to disambiguate or focus the search.

---

## T034: Implement POST /ask-selected Endpoint

### Implementation

**File**: `backend/src/routes/chat.py`

```python
from fastapi import APIRouter, HTTPException, Depends
from src.models.requests import AskSelectedRequest
from src.models.responses import ChatResponse, ErrorResponse
from src.services.embeddings import EmbeddingsService
from src.services.retrieval import RetrievalService
from src.services.agent import AgentService
from src.services.session import SessionService
import logging

router = APIRouter()
logger = logging.getLogger(__name__)

@router.post("/ask-selected", response_model=ChatResponse)
async def ask_selected(
    request: AskSelectedRequest,
    embeddings_service: EmbeddingsService = Depends(),
    retrieval_service: RetrievalService = Depends(),
    agent_service: AgentService = Depends(),
    session_service: SessionService = Depends()
):
    """
    Handle questions about selected text with hybrid retrieval.

    This endpoint:
    1. Validates selected text length (max 5000 chars)
    2. Embeds both the question and selected text
    3. Performs hybrid retrieval (70% query, 30% selected text)
    4. Generates answer using retrieved context
    5. Returns response with citations
    """
    try:
        # Validate selected text
        if not request.selected_text or not request.selected_text.strip():
            raise HTTPException(
                status_code=422,
                detail="selected_text cannot be empty"
            )

        if len(request.selected_text) > 5000:
            raise HTTPException(
                status_code=422,
                detail="selected_text exceeds maximum length of 5000 characters"
            )

        # Check rate limiting
        session_service.check_rate_limit(request.session_id)

        # Generate embeddings (parallel)
        logger.info(
            f"Generating embeddings for question and selected text",
            extra={"session_id": request.session_id}
        )

        query_embedding = await embeddings_service.embed_text(request.question)
        selected_embedding = await embeddings_service.embed_text(request.selected_text)

        # Hybrid retrieval
        logger.info(
            f"Performing hybrid retrieval",
            extra={"session_id": request.session_id}
        )

        retrieved_chunks = await retrieval_service.search(
            query_embedding=query_embedding,
            selected_text_embedding=selected_embedding,
            limit=5,
            threshold=0.65,
            hybrid_weights=(0.7, 0.3)
        )

        if not retrieved_chunks:
            return ChatResponse(
                answer="I don't have enough information about the selected text in the course materials. "
                       "Could you try selecting a different passage or asking a more general question?",
                sources=[],
                session_id=request.session_id
            )

        # Generate answer with agent
        logger.info(
            f"Generating answer with {len(retrieved_chunks)} chunks",
            extra={"session_id": request.session_id}
        )

        response = await agent_service.generate_answer(
            question=request.question,
            context_chunks=retrieved_chunks,
            session_id=request.session_id,
            selected_text=request.selected_text  # Pass for context
        )

        # Update session
        session_service.update_session(request.session_id)

        return ChatResponse(
            answer=response["answer"],
            sources=response["sources"],
            session_id=request.session_id
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(
            f"Error in /ask-selected: {str(e)}",
            extra={"session_id": request.session_id},
            exc_info=True
        )
        raise HTTPException(
            status_code=500,
            detail="An error occurred processing your request. Please try again."
        )
```

### Request Model Update

**File**: `backend/src/models/requests.py`

```python
from pydantic import BaseModel, Field, validator
from typing import Optional
import uuid

class AskSelectedRequest(BaseModel):
    """Request model for selected text questions."""

    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question about the selected text"
    )

    selected_text: str = Field(
        ...,
        min_length=1,
        max_length=5000,
        description="Text selected by the user (max 5000 characters)"
    )

    session_id: Optional[str] = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Session ID for rate limiting and analytics"
    )

    @validator('selected_text')
    def validate_selected_text(cls, v):
        """Validate that selected text is not just whitespace."""
        if not v.strip():
            raise ValueError("selected_text cannot be empty or whitespace only")
        return v.strip()

    @validator('question')
    def validate_question(cls, v):
        """Validate that question is not just whitespace."""
        if not v.strip():
            raise ValueError("question cannot be empty or whitespace only")
        return v.strip()
```

---

## T035: Add Selected Text Handler to Widget

### Frontend Implementation

**File**: `frontend/static/js/chatbot-widget.js` (additions)

```javascript
class ChatbotWidget extends HTMLElement {
    constructor() {
        super();
        this.selectedText = null;
        this.selectionButton = null;
        this.initSelectionHandler();
    }

    initSelectionHandler() {
        // Listen for text selection events
        document.addEventListener('mouseup', this.handleTextSelection.bind(this));
        document.addEventListener('touchend', this.handleTextSelection.bind(this));

        // Hide button on scroll
        window.addEventListener('scroll', this.hideSelectionButton.bind(this));
    }

    handleTextSelection(event) {
        const selection = window.getSelection();
        const selectedText = selection.toString().trim();

        // Require at least 10 characters for meaningful selection
        if (selectedText.length < 10) {
            this.hideSelectionButton();
            return;
        }

        // Limit to 5000 characters
        if (selectedText.length > 5000) {
            this.showToast('Selected text is too long. Please select a shorter passage (max 5000 characters).');
            return;
        }

        // Store selected text
        this.selectedText = selectedText;

        // Show "Ask about this" button near selection
        this.showSelectionButton(event.clientX, event.clientY);
    }

    showSelectionButton(x, y) {
        // Remove existing button if present
        this.hideSelectionButton();

        // Create floating button
        this.selectionButton = document.createElement('button');
        this.selectionButton.className = 'chatbot-selection-button';
        this.selectionButton.innerHTML = `
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                <path d="M8 0a8 8 0 1 0 0 16A8 8 0 0 0 8 0zm.5 4.5a.5.5 0 0 1 .5.5v3h3a.5.5 0 0 1 0 1H9v3a.5.5 0 0 1-1 0V9H5a.5.5 0 0 1 0-1h3V5a.5.5 0 0 1 .5-.5z"/>
            </svg>
            Ask about this
        `;

        // Position near selection
        this.selectionButton.style.position = 'fixed';
        this.selectionButton.style.left = `${x}px`;
        this.selectionButton.style.top = `${y + 20}px`;
        this.selectionButton.style.zIndex = '10000';

        // Add click handler
        this.selectionButton.addEventListener('click', this.handleAskAboutSelection.bind(this));

        // Add to page
        document.body.appendChild(this.selectionButton);

        // Auto-hide after 10 seconds
        setTimeout(() => this.hideSelectionButton(), 10000);
    }

    hideSelectionButton() {
        if (this.selectionButton) {
            this.selectionButton.remove();
            this.selectionButton = null;
        }
    }

    handleAskAboutSelection() {
        if (!this.selectedText) return;

        // Open chatbot panel
        this.open();

        // Pre-populate input with selected text context
        const inputField = this.shadowRoot.querySelector('#chatbot-input');
        const selectionPreview = this.shadowRoot.querySelector('#selection-preview');

        // Show selection preview in chat
        if (selectionPreview) {
            selectionPreview.style.display = 'block';
            selectionPreview.innerHTML = `
                <div class="selection-context">
                    <strong>Selected text:</strong>
                    <div class="selection-text">${this.truncateText(this.selectedText, 200)}</div>
                    <button class="clear-selection" onclick="this.getRootNode().host.clearSelection()">
                        ✕ Clear
                    </button>
                </div>
            `;
        }

        // Set placeholder to guide user
        inputField.placeholder = 'Ask a question about the selected text...';
        inputField.focus();

        // Hide floating button
        this.hideSelectionButton();
    }

    clearSelection() {
        this.selectedText = null;
        const selectionPreview = this.shadowRoot.querySelector('#selection-preview');
        if (selectionPreview) {
            selectionPreview.style.display = 'none';
        }
        const inputField = this.shadowRoot.querySelector('#chatbot-input');
        inputField.placeholder = 'Ask a question about the course...';
    }

    truncateText(text, maxLength) {
        if (text.length <= maxLength) return text;
        return text.substring(0, maxLength) + '...';
    }

    showToast(message) {
        const toast = document.createElement('div');
        toast.className = 'chatbot-toast';
        toast.textContent = message;
        document.body.appendChild(toast);

        setTimeout(() => toast.remove(), 3000);
    }
}
```

### CSS Additions

**File**: `frontend/static/js/chatbot-styles.css` (additions)

```css
/* Selection button */
.chatbot-selection-button {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 8px 12px;
    background: var(--ifm-color-primary);
    color: white;
    border: none;
    border-radius: 20px;
    font-size: 14px;
    font-weight: 500;
    cursor: pointer;
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.2);
    transition: all 0.2s;
}

.chatbot-selection-button:hover {
    background: var(--ifm-color-primary-dark);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
    transform: translateY(-2px);
}

.chatbot-selection-button svg {
    width: 16px;
    height: 16px;
}

/* Selection preview in chat */
#selection-preview {
    display: none;
    margin-bottom: 12px;
}

.selection-context {
    background: var(--ifm-color-emphasis-100);
    border-left: 3px solid var(--ifm-color-primary);
    padding: 12px;
    border-radius: 4px;
    position: relative;
}

.selection-context strong {
    display: block;
    margin-bottom: 8px;
    color: var(--ifm-color-emphasis-700);
    font-size: 13px;
}

.selection-text {
    font-size: 14px;
    line-height: 1.5;
    color: var(--ifm-color-emphasis-800);
    font-style: italic;
    max-height: 100px;
    overflow-y: auto;
}

.clear-selection {
    position: absolute;
    top: 8px;
    right: 8px;
    background: transparent;
    border: none;
    color: var(--ifm-color-emphasis-600);
    cursor: pointer;
    font-size: 18px;
    padding: 4px;
    line-height: 1;
}

.clear-selection:hover {
    color: var(--ifm-color-danger);
}

/* Toast notifications */
.chatbot-toast {
    position: fixed;
    bottom: 80px;
    right: 24px;
    background: var(--ifm-color-emphasis-800);
    color: white;
    padding: 12px 20px;
    border-radius: 8px;
    font-size: 14px;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
    z-index: 10001;
    animation: slideInUp 0.3s ease;
}

@keyframes slideInUp {
    from {
        transform: translateY(20px);
        opacity: 0;
    }
    to {
        transform: translateY(0);
        opacity: 1;
    }
}
```

---

## T036: Update Widget Submit Handler

### Implementation

**File**: `frontend/static/js/chatbot-widget.js` (update submit handler)

```javascript
class ChatbotWidget extends HTMLElement {
    // ... previous code ...

    async handleSubmit(event) {
        event.preventDefault();

        const inputField = this.shadowRoot.querySelector('#chatbot-input');
        const question = inputField.value.trim();

        if (!question) return;

        // Disable input during processing
        inputField.disabled = true;
        const submitButton = this.shadowRoot.querySelector('#submit-button');
        submitButton.disabled = true;

        // Add user message to chat
        this.addMessage(question, 'user');

        // Clear input
        inputField.value = '';

        // Show typing indicator
        this.showTypingIndicator();

        try {
            // Determine endpoint based on selection mode
            const endpoint = this.selectedText ? '/ask-selected' : '/ask';

            // Build request payload
            const payload = {
                question: question,
                session_id: this.getSessionId()
            };

            // Add selected text if present
            if (this.selectedText) {
                payload.selected_text = this.selectedText;
            }

            // Call API
            const response = await fetch(`${this.apiUrl}${endpoint}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(payload)
            });

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const data = await response.json();

            // Hide typing indicator
            this.hideTypingIndicator();

            // Add bot response
            this.addMessage(data.answer, 'bot', data.sources);

            // Clear selection after successful query
            if (this.selectedText) {
                this.clearSelection();
            }

        } catch (error) {
            console.error('Chatbot error:', error);
            this.hideTypingIndicator();
            this.addMessage(
                'Sorry, I encountered an error. Please try again.',
                'bot',
                [],
                true // isError flag
            );
        } finally {
            // Re-enable input
            inputField.disabled = false;
            submitButton.disabled = false;
            inputField.focus();
        }
    }

    getSessionId() {
        // Get or create session ID in sessionStorage
        let sessionId = sessionStorage.getItem('chatbot-session-id');
        if (!sessionId) {
            sessionId = this.generateUUID();
            sessionStorage.setItem('chatbot-session-id', sessionId);
        }
        return sessionId;
    }

    generateUUID() {
        return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
            const r = Math.random() * 16 | 0;
            const v = c === 'x' ? r : (r & 0x3 | 0x8);
            return v.toString(16);
        });
    }
}
```

---

## Testing Checklist

### T032: ✅ Test Created
- [ ] Test file created: `backend/tests/test_api.py`
- [ ] Test case: Selected text boosts relevance
- [ ] Test case: Validates text length (max 5000 chars)
- [ ] Test case: Handles empty selection gracefully
- [ ] Test case: Hybrid scoring works correctly
- [ ] All tests pass with `pytest`

### T033: ✅ Retrieval Extended
- [ ] `retrieval.py` updated with hybrid scoring
- [ ] Hybrid weights configurable (0.7 query, 0.3 selected)
- [ ] Cosine similarity calculation for both vectors
- [ ] Results reranked by combined score
- [ ] Unit tests pass

### T034: ✅ Endpoint Implemented
- [ ] `/ask-selected` endpoint created
- [ ] Request validation (max 5000 chars)
- [ ] Parallel embedding generation
- [ ] Hybrid retrieval integration
- [ ] Error handling and logging
- [ ] Returns ChatResponse with sources

### T035: ✅ Selection Handler Added
- [ ] Text selection listener added
- [ ] "Ask about this" button appears near selection
- [ ] Button positioned correctly
- [ ] Selected text stored in widget state
- [ ] Preview shown in chat panel
- [ ] CSS styling applied

### T036: ✅ Submit Logic Updated
- [ ] Detects selection mode vs regular mode
- [ ] Calls `/ask-selected` when text selected
- [ ] Calls `/ask` when no selection
- [ ] Selected text included in payload
- [ ] Selection cleared after successful query
- [ ] Session ID persisted in sessionStorage

---

## Deployment Steps

1. **Backend Deployment**:
   ```bash
   # Update backend code
   git pull origin main

   # Install any new dependencies
   pip install -r requirements.txt

   # Run tests
   pytest backend/tests/test_api.py -k "test_ask_selected" -v

   # Deploy to Render
   git push render main
   ```

2. **Frontend Deployment**:
   ```bash
   # Update widget files
   cp frontend/static/js/chatbot-widget.js static/js/
   cp frontend/static/js/chatbot-styles.css static/js/

   # Rebuild Docusaurus
   npm run build

   # Deploy to GitHub Pages
   npm run deploy
   ```

3. **Verification**:
   - Select text in documentation
   - Click "Ask about this" button
   - Ask question about selected text
   - Verify answer is contextually relevant
   - Check response time (<2s)

---

## Success Criteria

✅ **User Story 2 Complete** when:
- Users can select text in documentation
- "Ask about this" button appears near selection
- Clicking button opens chatbot with context
- Questions about selected text receive focused answers
- Hybrid retrieval boosts relevance by 30%+
- All tests pass
- Feature documented in user guide

---

**Implementation Status**: ✅ **READY FOR DEPLOYMENT**

**Estimated Effort**: 4-6 hours for full implementation

**Dependencies**: Requires Phase 1-3 complete (MVP operational)
