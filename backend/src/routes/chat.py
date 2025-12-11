"""Chat endpoints for RAG chatbot.

Implements POST /ask and POST /ask-selected endpoints.
"""

from fastapi import APIRouter, HTTPException, status
from typing import Dict
import time

from ..models.requests import AskRequest, AskSelectedRequest
from ..models.responses import ChatResponse, ErrorResponse
from ..services.embeddings import embed_query
from ..services.retrieval import search
from ..services.agent import generate_answer
from ..utils.session import session_manager

router = APIRouter(prefix="/api", tags=["chat"])


@router.post("/ask", response_model=ChatResponse)
async def ask_question(request: AskRequest) -> ChatResponse:
    """Answer a general question about the book.

    Uses RAG to retrieve relevant content and generate an answer with citations.

    Args:
        request: AskRequest with question and optional session_id

    Returns:
        ChatResponse with answer, sources, and session_id

    Raises:
        HTTPException: If retrieval or generation fails
    """
    start_time = time.time()

    try:
        # Step 1: Get conversation history (if multi-turn)
        conversation_history = session_manager.get_history(request.session_id)

        # Step 2: Generate query embedding
        try:
            query_embedding = embed_query(request.question)
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Embedding service unavailable: {str(e)}",
            )

        # Step 3: Retrieve similar chunks
        try:
            retrieved_chunks = search(
                query_embedding=query_embedding,
                limit=10,  # Retrieve top-10, will be reranked to top-5
            )
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Retrieval service unavailable: {str(e)}",
            )

        # Step 4: Generate answer with Gemini
        try:
            result = generate_answer(
                question=request.question,
                retrieved_chunks=retrieved_chunks,
                conversation_history=conversation_history if conversation_history else None,
            )
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Answer generation failed: {str(e)}",
            )

        # Step 5: Add to conversation history
        session_manager.add_message(
            session_id=request.session_id,
            role="user",
            content=request.question,
        )

        session_manager.add_message(
            session_id=request.session_id,
            role="assistant",
            content=result["answer"],
            sources=result["sources"],
        )

        # Calculate latency
        latency_ms = int((time.time() - start_time) * 1000)

        # Return response
        return ChatResponse(
            answer=result["answer"],
            sources=result["sources"],
            session_id=request.session_id,
            latency_ms=latency_ms,
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}",
        )


@router.post("/ask-selected", response_model=ChatResponse)
async def ask_about_selected_text(
    request: AskSelectedRequest,
) -> ChatResponse:
    """Answer a question about selected text.

    Uses hybrid scoring to boost relevance of chunks related to selected text.

    Args:
        request: AskSelectedRequest with question, selected_text, and optional session_id

    Returns:
        ChatResponse with answer, sources, and session_id

    Raises:
        HTTPException: If retrieval or generation fails
    """
    start_time = time.time()

    try:
        # Step 1: Get conversation history
        conversation_history = session_manager.get_history(request.session_id)

        # Step 2: Generate embeddings for both question and selected text
        try:
            query_embedding = embed_query(request.question)
            selected_embedding = embed_query(request.selected_text)
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Embedding service unavailable: {str(e)}",
            )

        # Step 3: Retrieve with hybrid scoring
        try:
            retrieved_chunks = search(
                query_embedding=query_embedding,
                limit=10,
                selected_text_embedding=selected_embedding,  # Enable hybrid scoring
            )
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Retrieval service unavailable: {str(e)}",
            )

        # Step 4: Generate answer (include selected text context in question)
        enhanced_question = f"""Context: The user selected this text: "{request.selected_text[:200]}..."

Question: {request.question}"""

        try:
            result = generate_answer(
                question=enhanced_question,
                retrieved_chunks=retrieved_chunks,
                conversation_history=conversation_history if conversation_history else None,
            )
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail=f"Answer generation failed: {str(e)}",
            )

        # Step 5: Add to conversation history
        session_manager.add_message(
            session_id=request.session_id,
            role="user",
            content=f"{request.question} (about selected text)",
        )

        session_manager.add_message(
            session_id=request.session_id,
            role="assistant",
            content=result["answer"],
            sources=result["sources"],
        )

        # Calculate latency
        latency_ms = int((time.time() - start_time) * 1000)

        return ChatResponse(
            answer=result["answer"],
            sources=result["sources"],
            session_id=request.session_id,
            latency_ms=latency_ms,
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}",
        )
