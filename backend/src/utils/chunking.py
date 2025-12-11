"""Text chunking utilities using LangChain.

Implements markdown-aware chunking with 800 tokens and 200-token overlap.
"""

from langchain_text_splitters import RecursiveCharacterTextSplitter
from typing import List, Dict
import tiktoken


# Constants
CHUNK_SIZE = 800  # tokens
CHUNK_OVERLAP = 200  # tokens (25% overlap)
ENCODING_NAME = "cl100k_base"  # For text-embedding-3-large


def count_tokens(text: str) -> int:
    """Count tokens in text using tiktoken.

    Args:
        text: Input text

    Returns:
        Number of tokens
    """
    encoding = tiktoken.get_encoding(ENCODING_NAME)
    return len(encoding.encode(text))


def create_text_splitter() -> RecursiveCharacterTextSplitter:
    """Create configured text splitter for markdown.

    Returns:
        Configured RecursiveCharacterTextSplitter
    """
    # Markdown-aware separators (ordered by priority)
    separators = [
        "\n\n##",  # H2 headers
        "\n\n###",  # H3 headers
        "\n\n",  # Paragraph breaks
        "\n",  # Line breaks
        ". ",  # Sentence breaks
        " ",  # Word breaks
        "",  # Character breaks (last resort)
    ]

    return RecursiveCharacterTextSplitter.from_tiktoken_encoder(
        encoding_name=ENCODING_NAME,
        chunk_size=CHUNK_SIZE,
        chunk_overlap=CHUNK_OVERLAP,
        separators=separators,
        keep_separator=True,  # Preserve markdown headers
    )


def chunk_text(text: str, file_path: str) -> List[Dict[str, any]]:
    """Split text into chunks with metadata.

    Args:
        text: Full document text
        file_path: Source file path for metadata

    Returns:
        List of chunk dictionaries with text, index, and metadata
    """
    if not text or not text.strip():
        return []

    # Create splitter
    splitter = create_text_splitter()

    # Split text
    chunks = splitter.split_text(text)

    # Add metadata to each chunk
    chunked_data = []
    current_section = "Introduction"  # Default section

    for idx, chunk_text in enumerate(chunks):
        # Extract section heading from chunk if present
        if chunk_text.strip().startswith("#"):
            lines = chunk_text.strip().split("\n", 1)
            first_line = lines[0].strip()
            # Remove markdown heading markers
            section = first_line.lstrip("#").strip()
            if section:
                current_section = section

        # Create chunk metadata
        chunk_data = {
            "chunk_text": chunk_text.strip(),
            "chunk_index": idx,
            "file_path": file_path,
            "section_heading": current_section,
            "token_count": count_tokens(chunk_text),
        }

        chunked_data.append(chunk_data)

    return chunked_data


def validate_chunk(chunk_data: Dict[str, any]) -> bool:
    """Validate chunk meets requirements.

    Args:
        chunk_data: Chunk dictionary from chunk_text()

    Returns:
        True if valid, False otherwise
    """
    # Check required fields
    required_fields = [
        "chunk_text",
        "chunk_index",
        "file_path",
        "section_heading",
        "token_count",
    ]

    if not all(field in chunk_data for field in required_fields):
        return False

    # Check token count range
    token_count = chunk_data["token_count"]
    if not (100 <= token_count <= 2000):
        return False

    # Check chunk text is not empty
    if not chunk_data["chunk_text"].strip():
        return False

    return True
