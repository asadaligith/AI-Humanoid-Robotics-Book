"""PostgreSQL database connection and ORM models using SQLAlchemy.

Manages connection to Neon Serverless Postgres for chunks metadata storage.
"""

from sqlalchemy import (
    create_engine,
    Column,
    String,
    Integer,
    TIMESTAMP,
    CheckConstraint,
    Index,
    func,
)
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session
from contextlib import contextmanager
from typing import Generator
import uuid

from ..config import settings

# Prepare database URL for psycopg3 driver (not psycopg2)
database_url = settings.database_url
if database_url.startswith('postgresql://'):
    database_url = database_url.replace('postgresql://', 'postgresql+psycopg://', 1)

# Create SQLAlchemy engine
engine = create_engine(
    database_url,
    pool_pre_ping=True,  # Verify connections before using
    pool_recycle=3600,  # Recycle connections after 1 hour
    echo=settings.is_development,  # Log SQL in development
)

# Create session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create declarative base
Base = declarative_base()


class ChunkMetadata(Base):
    """SQLAlchemy model for chunks_metadata table.

    Stores metadata for document chunks indexed in Qdrant.
    """

    __tablename__ = "chunks_metadata"

    chunk_id = Column(
        PG_UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4,
        comment="Unique chunk identifier (matches Qdrant point ID)",
    )

    file_path = Column(
        String(512),
        nullable=False,
        comment="Relative path from /docs/",
    )

    section_heading = Column(
        String(512),
        nullable=True,
        comment="Section/chapter heading",
    )

    chunk_index = Column(
        Integer,
        nullable=False,
        comment="Position within document (0-indexed)",
    )

    content_hash = Column(
        String(64),
        nullable=False,
        unique=True,
        comment="SHA256 hash for deduplication",
    )

    token_count = Column(
        Integer,
        nullable=False,
        comment="Number of tokens in chunk",
    )

    created_at = Column(
        TIMESTAMP,
        server_default=func.now(),
        nullable=False,
        comment="Creation timestamp",
    )

    updated_at = Column(
        TIMESTAMP,
        server_default=func.now(),
        onupdate=func.now(),
        nullable=False,
        comment="Last update timestamp",
    )

    # Constraints
    __table_args__ = (
        CheckConstraint("chunk_index >= 0", name="chunk_index_non_negative"),
        CheckConstraint(
            "token_count BETWEEN 100 AND 2000", name="token_count_range"
        ),
        Index("idx_file_path", "file_path"),
        Index("idx_section_heading", "section_heading"),
        Index("idx_content_hash", "content_hash"),
        Index("idx_created_at", "created_at"),
    )

    def __repr__(self) -> str:
        """String representation."""
        return f"<ChunkMetadata(chunk_id={self.chunk_id}, file_path={self.file_path})>"


def create_tables():
    """Create all database tables.

    Run this function to initialize the database schema.
    """
    Base.metadata.create_all(bind=engine)


def drop_tables():
    """Drop all database tables.

    WARNING: This will delete all data!
    """
    Base.metadata.drop_all(bind=engine)


@contextmanager
def get_db() -> Generator[Session, None, None]:
    """Get database session with automatic cleanup.

    Usage:
        with get_db() as db:
            # Use db session
            db.query(ChunkMetadata).all()
    """
    db = SessionLocal()
    try:
        yield db
        db.commit()
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()


def get_db_session() -> Generator[Session, None, None]:
    """Dependency for FastAPI routes.

    Usage:
        @app.get("/")
        def read_root(db: Session = Depends(get_db_session)):
            return db.query(ChunkMetadata).count()
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
