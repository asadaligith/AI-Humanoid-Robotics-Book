"""Structured logging utility for all services.

Provides consistent logging format with request IDs, latency tracking, and error categorization.
"""

import logging
import json
import time
from typing import Dict, Optional, Any
from contextvars import ContextVar
from datetime import datetime
import traceback

# Context variable for request ID (thread-safe)
request_id_context: ContextVar[Optional[str]] = ContextVar("request_id", default=None)


class StructuredFormatter(logging.Formatter):
    """Custom formatter for structured JSON logging."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON.

        Args:
            record: Log record to format

        Returns:
            JSON-formatted log string
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }

        # Add request ID if available
        request_id = request_id_context.get()
        if request_id:
            log_data["request_id"] = request_id

        # Add extra fields from record
        if hasattr(record, "request_id"):
            log_data["request_id"] = record.request_id
        if hasattr(record, "latency_ms"):
            log_data["latency_ms"] = record.latency_ms
        if hasattr(record, "error_type"):
            log_data["error_type"] = record.error_type
        if hasattr(record, "user_id"):
            log_data["user_id"] = record.user_id
        if hasattr(record, "service"):
            log_data["service"] = record.service
        if hasattr(record, "operation"):
            log_data["operation"] = record.operation
        if hasattr(record, "metadata"):
            log_data["metadata"] = record.metadata

        # Add exception info if present
        if record.exc_info:
            log_data["exception"] = {
                "type": record.exc_info[0].__name__ if record.exc_info[0] else "Unknown",
                "message": str(record.exc_info[1]) if record.exc_info[1] else "No message",
                "traceback": traceback.format_exception(*record.exc_info),
            }

        return json.dumps(log_data)


def setup_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """Set up a structured logger.

    Args:
        name: Logger name (typically __name__ of the module)
        level: Logging level (default: INFO)

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Remove existing handlers to avoid duplicates
    logger.handlers.clear()

    # Create console handler with structured formatter
    handler = logging.StreamHandler()
    handler.setFormatter(StructuredFormatter())
    logger.addHandler(handler)

    # Don't propagate to root logger
    logger.propagate = False

    return logger


def set_request_id(request_id: str):
    """Set request ID in context for current request.

    Args:
        request_id: Unique request identifier
    """
    request_id_context.set(request_id)


def get_request_id() -> Optional[str]:
    """Get current request ID from context.

    Returns:
        Request ID if set, None otherwise
    """
    return request_id_context.get()


class LogContext:
    """Context manager for timed operations with automatic logging."""

    def __init__(
        self,
        logger: logging.Logger,
        operation: str,
        service: str,
        level: int = logging.INFO,
        metadata: Optional[Dict[str, Any]] = None,
    ):
        """Initialize log context.

        Args:
            logger: Logger instance
            operation: Operation name (e.g., "embed_text", "search_qdrant")
            service: Service name (e.g., "embeddings", "retrieval", "agent")
            level: Log level for success (default: INFO)
            metadata: Optional metadata dict to include in logs
        """
        self.logger = logger
        self.operation = operation
        self.service = service
        self.level = level
        self.metadata = metadata or {}
        self.start_time = None

    def __enter__(self):
        """Start timing."""
        self.start_time = time.time()
        self.logger.log(
            self.level,
            f"Starting {self.operation}",
            extra={
                "service": self.service,
                "operation": self.operation,
                "metadata": self.metadata,
            },
        )
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Log completion with latency.

        Args:
            exc_type: Exception type if error occurred
            exc_val: Exception value
            exc_tb: Exception traceback
        """
        latency_ms = (time.time() - self.start_time) * 1000

        if exc_type is None:
            # Success case
            self.logger.log(
                self.level,
                f"Completed {self.operation}",
                extra={
                    "service": self.service,
                    "operation": self.operation,
                    "latency_ms": round(latency_ms, 2),
                    "metadata": self.metadata,
                },
            )
        else:
            # Error case
            self.logger.error(
                f"Failed {self.operation}",
                exc_info=(exc_type, exc_val, exc_tb),
                extra={
                    "service": self.service,
                    "operation": self.operation,
                    "latency_ms": round(latency_ms, 2),
                    "error_type": exc_type.__name__ if exc_type else "Unknown",
                    "metadata": self.metadata,
                },
            )

        # Return False to propagate exception
        return False


def log_slow_operation(
    logger: logging.Logger, operation: str, latency_ms: float, threshold_ms: float = 2000
):
    """Log warning if operation exceeds latency threshold.

    Args:
        logger: Logger instance
        operation: Operation name
        latency_ms: Actual latency in milliseconds
        threshold_ms: Threshold for slow operations (default: 2000ms = 2s)
    """
    if latency_ms > threshold_ms:
        logger.warning(
            f"Slow operation detected: {operation}",
            extra={
                "operation": operation,
                "latency_ms": round(latency_ms, 2),
                "threshold_ms": threshold_ms,
                "slowdown_factor": round(latency_ms / threshold_ms, 2),
            },
        )
