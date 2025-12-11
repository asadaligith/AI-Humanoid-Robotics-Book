"""Health check endpoint for monitoring and deployment validation."""

from fastapi import APIRouter
from datetime import datetime
import time

from ..models.responses import HealthResponse, ServiceStatus
from ..config import settings

router = APIRouter(tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """Check health of all external dependencies.

    Returns:
        HealthResponse with status of each service

    Status codes:
        - healthy: All services operational
        - degraded: Some services down but core functionality works
        - unhealthy: Critical services down
    """
    services = []

    # Check Qdrant
    qdrant_status = await check_qdrant()
    services.append(qdrant_status)

    # Check Postgres
    postgres_status = await check_postgres()
    services.append(postgres_status)

    # Check Gemini API
    gemini_status = await check_gemini()
    services.append(gemini_status)

    # Determine overall status
    healthy_count = sum(1 for s in services if s.healthy)

    if healthy_count == len(services):
        overall_status = "healthy"
    elif healthy_count >= len(services) // 2:
        overall_status = "degraded"
    else:
        overall_status = "unhealthy"

    return HealthResponse(
        status=overall_status,
        services=services,
        version="1.0.0",
        timestamp=datetime.utcnow(),
    )


async def check_qdrant() -> ServiceStatus:
    """Check Qdrant connection."""
    start_time = time.time()

    try:
        from ..services.retrieval import get_collection_info

        info = get_collection_info()
        latency_ms = int((time.time() - start_time) * 1000)

        if "error" in info:
            return ServiceStatus(
                name="qdrant",
                healthy=False,
                latency_ms=latency_ms,
                error=info["error"],
            )

        return ServiceStatus(
            name="qdrant",
            healthy=True,
            latency_ms=latency_ms,
        )

    except Exception as e:
        latency_ms = int((time.time() - start_time) * 1000)
        return ServiceStatus(
            name="qdrant",
            healthy=False,
            latency_ms=latency_ms,
            error=str(e),
        )


async def check_postgres() -> ServiceStatus:
    """Check Postgres connection."""
    start_time = time.time()

    try:
        from ..services.database import get_db, ChunkMetadata

        with get_db() as db:
            # Simple query to check connection
            count = db.query(ChunkMetadata).count()

        latency_ms = int((time.time() - start_time) * 1000)

        return ServiceStatus(
            name="postgres",
            healthy=True,
            latency_ms=latency_ms,
        )

    except Exception as e:
        latency_ms = int((time.time() - start_time) * 1000)
        return ServiceStatus(
            name="postgres",
            healthy=False,
            latency_ms=latency_ms,
            error=str(e),
        )


async def check_gemini() -> ServiceStatus:
    """Check Gemini API access."""
    start_time = time.time()

    try:
        from ..services.embeddings import embed_text

        # Generate a test embedding
        test_embedding = embed_text("health check test", task_type="RETRIEVAL_QUERY")

        latency_ms = int((time.time() - start_time) * 1000)

        if len(test_embedding) == 768:
            return ServiceStatus(
                name="gemini",
                healthy=True,
                latency_ms=latency_ms,
            )
        else:
            return ServiceStatus(
                name="gemini",
                healthy=False,
                latency_ms=latency_ms,
                error=f"Unexpected embedding dimensions: {len(test_embedding)}",
            )

    except Exception as e:
        latency_ms = int((time.time() - start_time) * 1000)
        return ServiceStatus(
            name="gemini",
            healthy=False,
            latency_ms=latency_ms,
            error=str(e),
        )
