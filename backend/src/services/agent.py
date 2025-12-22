"""RAG-based chatbot agent using OpenAI Agents SDK.

Uses OpenAI GPT-4o-mini for generating answers with strict grounding
to retrieved content from the AI Humanoid Robotics Book.
"""

from agents import Agent, Runner, SQLiteSession, ModelSettings, set_default_openai_key
from typing import List, Dict, Optional
import asyncio
import os

from ..config import settings

# Set OpenAI API key for the Agents SDK
os.environ["OPENAI_API_KEY"] = settings.openai_api_key
set_default_openai_key(settings.openai_api_key)

# System instructions for the agent
AGENT_INSTRUCTIONS = """You are a helpful assistant for the AI Humanoid Robotics Book. Your role is to answer questions based ONLY on the provided book content.

CRITICAL RULES:
1. ONLY use information from the retrieved book chunks provided in the context
2. If the retrieved content doesn't contain relevant information, respond: "I don't have information about that in the book."
3. ALWAYS cite your sources by mentioning the chapter/section
4. Do NOT use your general knowledge - only use the provided content
5. Be concise but thorough in your explanations
6. If asked about topics not covered in the retrieved content, clearly state that
"""

# Create the agent with OpenAI model
chatbot_agent = Agent(
    name="AI Robotics Book Assistant",
    instructions=AGENT_INSTRUCTIONS,
    model="gpt-4o-mini",  # Fast, cost-effective OpenAI model
    model_settings=ModelSettings(
        temperature=0.3,  # Lower temperature for more factual responses
        max_tokens=2048,
    ),
)


async def generate_answer_async(
    question: str,
    retrieved_chunks: List[Dict],
    session_id: Optional[str] = None,
) -> Dict:
    """Generate answer using OpenAI Agents SDK with Gemini model.

    Args:
        question: User's question
        retrieved_chunks: List of retrieved document chunks from Qdrant
        session_id: Optional session ID for conversation history

    Returns:
        Dictionary with 'answer' and 'sources'

    Example:
        {
            "answer": "Humanoid robots consist of...",
            "sources": [
                {
                    "file": "/docs/chapter1.md",
                    "section": "Introduction",
                    "chunk": "Humanoid robots...",
                    "similarity": 0.85
                }
            ]
        }
    """
    if not retrieved_chunks:
        return {
            "answer": "I don't have information about that in the book. Could you try rephrasing your question or asking about a different topic?",
            "sources": [],
        }

    # Format retrieved content
    retrieved_content = format_retrieved_content(retrieved_chunks)

    # Build full prompt with context
    full_prompt = f"""Answer the user's question using the following retrieved content from the book:

{retrieved_content}

Question: {question}

Answer:"""

    try:
        # Create session for conversation history if session_id provided
        session = None
        if session_id:
            session = SQLiteSession(session_id, "/tmp/chat_sessions.db")

        # Run the agent
        result = await Runner.run(
            chatbot_agent,
            input=full_prompt,
            session=session,
        )

        answer = result.final_output

        # Extract sources
        sources = extract_sources(retrieved_chunks)

        return {"answer": answer, "sources": sources}

    except Exception as e:
        import traceback
        print(f"Error generating answer: {e}")
        traceback.print_exc()
        return {
            "answer": "I encountered an error while generating the answer. Please try again.",
            "sources": [],
        }


async def generate_answer(
    question: str,
    retrieved_chunks: List[Dict],
    conversation_history: Optional[List[Dict]] = None,
) -> Dict:
    """Generate answer with optional conversation history context.

    Args:
        question: User's question
        retrieved_chunks: List of retrieved document chunks from Qdrant
        conversation_history: Optional list of previous messages [{"role": "user/assistant", "content": "..."}]

    Returns:
        Dictionary with 'answer' and 'sources'
    """
    # Check for greeting first (quick path, no RAG needed)
    if is_greeting(question):
        return handle_greeting()

    if not retrieved_chunks:
        return {
            "answer": "I don't have information about that in the book. Could you try rephrasing your question or asking about a different topic?",
            "sources": [],
        }

    # Format retrieved content
    retrieved_content = format_retrieved_content(retrieved_chunks)

    # Build conversation context if history provided
    conversation_context = ""
    if conversation_history and len(conversation_history) > 0:
        # Include last 3 turns for context (avoid token limit)
        recent_history = conversation_history[-6:]  # Last 3 Q&A pairs
        history_parts = []
        for msg in recent_history:
            role = "User" if msg["role"] == "user" else "Assistant"
            history_parts.append(f"{role}: {msg['content']}")
        conversation_context = "\n".join(history_parts)

    # Build full prompt with context
    if conversation_context:
        full_prompt = f"""Previous conversation:
{conversation_context}

Retrieved content from the book:
{retrieved_content}

Current question: {question}

Please answer the current question using the retrieved content. You may reference the conversation history for context, but base your answer primarily on the retrieved content."""
    else:
        full_prompt = f"""Answer the user's question using the following retrieved content from the book:

{retrieved_content}

Question: {question}

Answer:"""

    try:
        # Use session_id approach if provided (for SQLite persistence)
        session_id = None
        if conversation_history:
            # Generate stable session ID from history
            import hashlib
            history_str = str([msg.get("content", "")[:50] for msg in conversation_history[:2]])  # Use first 2 messages
            session_id = hashlib.md5(history_str.encode()).hexdigest()

        # Run the agent
        session = None
        if session_id:
            session = SQLiteSession(session_id, "/tmp/chat_sessions.db")

        result = await Runner.run(
            chatbot_agent,
            input=full_prompt,
            session=session,
        )

        answer = result.final_output

        # Extract sources
        sources = extract_sources(retrieved_chunks)

        return {"answer": answer, "sources": sources}

    except Exception as e:
        import traceback
        print(f"Error generating answer: {e}")
        traceback.print_exc()
        return {
            "answer": "I encountered an error while generating the answer. Please try again.",
            "sources": [],
        }


def is_greeting(text: str) -> bool:
    """Check if text is a greeting.

    Args:
        text: User input text

    Returns:
        True if text is a greeting
    """
    greetings = [
        "hi",
        "hello",
        "hey",
        "good morning",
        "good afternoon",
        "good evening",
        "greetings",
        "howdy",
    ]

    normalized = text.lower().strip()

    # Exact match or starts with greeting
    return any(normalized == greeting or normalized.startswith(greeting + " ") for greeting in greetings)


def handle_greeting() -> Dict:
    """Return greeting response without RAG.

    Returns:
        Dictionary with greeting answer and empty sources
    """
    return {
        "answer": """Hello! 👋 Welcome to the AI & Humanoid Robotics course!

I'm your learning assistant with access to all 5 modules and 32 chapters covering:
• ROS 2 fundamentals and architecture
• Gazebo simulation and world building
• Isaac Sim and perception systems
• Vision-Language-Action (VLA) models
• Autonomous humanoid robotics projects

How can I help you today? You can ask me about any topic from the course!""",
        "sources": [],
        "type": "greeting",
    }


def format_retrieved_content(chunks: List[Dict]) -> str:
    """Format retrieved chunks for prompt.

    Args:
        chunks: List of retrieved chunks with payload

    Returns:
        Formatted string of retrieved content
    """
    formatted_parts = []

    for idx, chunk in enumerate(chunks, 1):
        payload = chunk.get("payload", {})
        file_path = payload.get("file_path", "Unknown")
        section = payload.get("section_heading", "Unknown Section")
        content = payload.get("content_text", "")

        part = f"""
[Source {idx}]
File: {file_path}
Section: {section}
Content: {content}
---
"""
        formatted_parts.append(part.strip())

    return "\n\n".join(formatted_parts)


def extract_sources(chunks: List[Dict]) -> List[Dict]:
    """Extract source citations from retrieved chunks.

    Args:
        chunks: List of retrieved chunks

    Returns:
        List of source dictionaries matching frontend expectations
    """
    sources = []

    for chunk in chunks:
        payload = chunk.get("payload", {})
        score = chunk.get("score", 0.0)

        # Truncate content for citation (max 200 chars)
        content = payload.get("content_text", "")
        truncated_content = content[:200] + "..." if len(content) > 200 else content

        # Match Pydantic response model structure
        source = {
            "file": payload.get("file_path", "Unknown"),
            "section": payload.get("section_heading", "Unknown Section"),
            "chunk": truncated_content,
            "similarity": round(score, 2),
        }

        sources.append(source)

    return sources


def validate_answer(answer: str, sources: List[Dict]) -> bool:
    """Validate that answer includes citations.

    Args:
        answer: Generated answer
        sources: Source citations

    Returns:
        True if answer appears to be grounded, False otherwise
    """
    # Check if answer mentions not having information
    if "don't have information" in answer.lower():
        return len(sources) == 0  # Should have no sources if no info

    # Check if answer has sources
    if not sources:
        return False

    # Basic check: answer should be substantial
    if len(answer.split()) < 10:
        return False

    return True
