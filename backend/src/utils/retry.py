"""Retry logic with exponential backoff for API calls.

Handles transient failures for OpenAI API calls (429, 503 errors).
"""

import time
import functools
from typing import Callable, TypeVar, Any, Tuple, Type
import logging
from openai import APIError, RateLimitError, APIConnectionError, APITimeoutError

# Type variable for generic return type
T = TypeVar("T")

# Default logger
logger = logging.getLogger(__name__)

# Retryable error types from OpenAI
RETRYABLE_ERRORS: Tuple[Type[Exception], ...] = (
    RateLimitError,  # 429 Too Many Requests
    APIConnectionError,  # Network/connection errors
    APITimeoutError,  # Timeout errors
)


def with_exponential_backoff(
    max_retries: int = 3,
    initial_delay: float = 1.0,
    max_delay: float = 60.0,
    exponential_base: float = 2.0,
    retryable_errors: Tuple[Type[Exception], ...] = RETRYABLE_ERRORS,
):
    """Decorator to retry function with exponential backoff.

    Args:
        max_retries: Maximum number of retry attempts (default: 3)
        initial_delay: Initial delay in seconds (default: 1.0)
        max_delay: Maximum delay in seconds (default: 60.0)
        exponential_base: Base for exponential backoff (default: 2.0)
        retryable_errors: Tuple of exception types to retry on

    Returns:
        Decorated function with retry logic

    Example:
        @with_exponential_backoff(max_retries=3, initial_delay=1.0)
        def call_openai_api():
            # API call that might fail with 429 or 503
            pass
    """

    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> T:
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)

                except retryable_errors as e:
                    last_exception = e

                    if attempt >= max_retries:
                        logger.error(
                            f"Max retries ({max_retries}) exceeded for {func.__name__}",
                            extra={
                                "function": func.__name__,
                                "attempt": attempt + 1,
                                "error_type": type(e).__name__,
                            },
                        )
                        raise

                    # Calculate delay with exponential backoff
                    delay = min(initial_delay * (exponential_base ** attempt), max_delay)

                    logger.warning(
                        f"Retrying {func.__name__} after {delay:.2f}s (attempt {attempt + 1}/{max_retries})",
                        extra={
                            "function": func.__name__,
                            "attempt": attempt + 1,
                            "delay_seconds": delay,
                            "error_type": type(e).__name__,
                            "error_message": str(e),
                        },
                    )

                    time.sleep(delay)

                except Exception as e:
                    # Non-retryable error, raise immediately
                    logger.error(
                        f"Non-retryable error in {func.__name__}",
                        exc_info=True,
                        extra={
                            "function": func.__name__,
                            "error_type": type(e).__name__,
                        },
                    )
                    raise

            # Should never reach here, but just in case
            if last_exception:
                raise last_exception

        return wrapper

    return decorator


async def async_retry_with_backoff(
    func: Callable[..., T],
    *args: Any,
    max_retries: int = 3,
    initial_delay: float = 1.0,
    max_delay: float = 60.0,
    exponential_base: float = 2.0,
    retryable_errors: Tuple[Type[Exception], ...] = RETRYABLE_ERRORS,
    **kwargs: Any,
) -> T:
    """Async retry helper with exponential backoff.

    Args:
        func: Async function to retry
        *args: Positional arguments for func
        max_retries: Maximum number of retry attempts
        initial_delay: Initial delay in seconds
        max_delay: Maximum delay in seconds
        exponential_base: Base for exponential backoff
        retryable_errors: Tuple of exception types to retry on
        **kwargs: Keyword arguments for func

    Returns:
        Result from successful function call

    Raises:
        Last exception if all retries fail

    Example:
        result = await async_retry_with_backoff(
            async_api_call,
            param1="value",
            max_retries=3
        )
    """
    import asyncio

    last_exception = None

    for attempt in range(max_retries + 1):
        try:
            return await func(*args, **kwargs)

        except retryable_errors as e:
            last_exception = e

            if attempt >= max_retries:
                logger.error(
                    f"Max retries ({max_retries}) exceeded for {func.__name__}",
                    extra={
                        "function": func.__name__,
                        "attempt": attempt + 1,
                        "error_type": type(e).__name__,
                    },
                )
                raise

            # Calculate delay with exponential backoff
            delay = min(initial_delay * (exponential_base ** attempt), max_delay)

            logger.warning(
                f"Retrying {func.__name__} after {delay:.2f}s (attempt {attempt + 1}/{max_retries})",
                extra={
                    "function": func.__name__,
                    "attempt": attempt + 1,
                    "delay_seconds": delay,
                    "error_type": type(e).__name__,
                    "error_message": str(e),
                },
            )

            await asyncio.sleep(delay)

        except Exception as e:
            # Non-retryable error, raise immediately
            logger.error(
                f"Non-retryable error in {func.__name__}",
                exc_info=True,
                extra={
                    "function": func.__name__,
                    "error_type": type(e).__name__,
                },
            )
            raise

    # Should never reach here, but just in case
    if last_exception:
        raise last_exception
