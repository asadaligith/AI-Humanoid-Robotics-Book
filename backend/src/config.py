"""Configuration management using Pydantic Settings.

Loads environment variables and provides type-safe configuration access.
"""

from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Gemini Configuration
    gemini_api_key: str

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str

    # Database Configuration
    database_url: str

    # Application Configuration
    environment: str = "development"

    # CORS Configuration
    allowed_origins: str = "http://localhost:3000"

    # Rate Limiting
    rate_limit_per_minute: int = 10

    # Retrieval Configuration
    similarity_threshold: float = 0.5
    max_chunks: int = 5

    class Config:
        """Pydantic configuration."""

        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False

    @property
    def cors_origins(self) -> List[str]:
        """Parse comma-separated CORS origins."""
        return [origin.strip() for origin in self.allowed_origins.split(",")]

    @property
    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.environment.lower() == "production"

    @property
    def is_development(self) -> bool:
        """Check if running in development environment."""
        return self.environment.lower() == "development"


# Singleton instance
settings = Settings()
