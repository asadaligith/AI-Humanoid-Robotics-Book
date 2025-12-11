"""FastAPI application entry point for RAG Chatbot backend.

Initializes the FastAPI app, registers routes, and configures middleware.
"""

import sys

# Fix Windows console encoding for emoji support
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
import time

from .config import settings
from .routes import chat, health

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for AI Humanoid Robotics Book",
    version="1.0.0",
    docs_url="/docs" if settings.is_development else None,  # Disable docs in production
    redoc_url="/redoc" if settings.is_development else None,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
)


# Request timing middleware
@app.middleware("http")
async def add_process_time_header(request: Request, call_next):
    """Add X-Process-Time header to responses."""
    start_time = time.time()
    response = await call_next(request)
    process_time = time.time() - start_time
    response.headers["X-Process-Time"] = f"{process_time:.3f}"
    return response


# Register routes
app.include_router(health.router)
app.include_router(chat.router)


# Exception handlers
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    """Handle Pydantic validation errors."""
    return JSONResponse(
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        content={
            "error": "Validation error",
            "code": "VALIDATION_ERROR",
            "details": exc.errors(),
        },
    )


@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Handle uncaught exceptions."""
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={
            "error": "Internal server error",
            "code": "INTERNAL_ERROR",
            "details": str(exc) if settings.is_development else "An error occurred",
        },
    )


# Root endpoint
@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running",
        "docs": "/docs" if settings.is_development else None,
        "health": "/health",
        "endpoints": {
            "ask": "/api/ask",
            "ask_selected": "/api/ask-selected",
        },
    }


# Startup event
@app.on_event("startup")
async def startup_event():
    """Run on application startup."""
    print("=" * 60)
    print("🚀 RAG Chatbot API Starting")
    print("=" * 60)
    print(f"Environment: {settings.environment}")
    print(f"CORS Origins: {', '.join(settings.cors_origins)}")
    print(f"Similarity Threshold: {settings.similarity_threshold}")
    print(f"Max Chunks: {settings.max_chunks}")
    print("=" * 60)


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Run on application shutdown."""
    print("\n" + "=" * 60)
    print("🛑 RAG Chatbot API Shutting Down")
    print("=" * 60)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.is_development,
    )
