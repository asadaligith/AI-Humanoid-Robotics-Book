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
from .utils.logging import setup_logger, log_slow_operation, set_request_id
import uuid

# Initialize logger
logger = setup_logger(__name__)

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


# Performance monitoring middleware
@app.middleware("http")
async def performance_monitoring_middleware(request: Request, call_next):
    """Track request latency and log slow queries.

    - Adds X-Process-Time header to responses
    - Sets request_id for distributed tracing
    - Logs slow operations (>2s threshold)
    - Tracks endpoint performance metrics
    """
    # Generate unique request ID
    request_id = str(uuid.uuid4())
    set_request_id(request_id)

    # Extract path and method
    path = request.url.path
    method = request.method

    # Start timing
    start_time = time.time()

    try:
        response = await call_next(request)

        # Calculate latency
        latency_ms = (time.time() - start_time) * 1000

        # Add performance headers
        response.headers["X-Process-Time"] = f"{latency_ms / 1000:.3f}"
        response.headers["X-Request-ID"] = request_id

        # Log request performance
        logger.info(
            f"{method} {path}",
            extra={
                "request_id": request_id,
                "method": method,
                "path": path,
                "status_code": response.status_code,
                "latency_ms": round(latency_ms, 2),
            },
        )

        # Log slow queries (>2000ms threshold)
        log_slow_operation(
            logger,
            operation=f"{method} {path}",
            latency_ms=latency_ms,
            threshold_ms=2000,
        )

        return response

    except Exception as e:
        # Log error with request context
        latency_ms = (time.time() - start_time) * 1000
        logger.error(
            f"Request failed: {method} {path}",
            exc_info=True,
            extra={
                "request_id": request_id,
                "method": method,
                "path": path,
                "latency_ms": round(latency_ms, 2),
                "error_type": type(e).__name__,
            },
        )
        raise


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
