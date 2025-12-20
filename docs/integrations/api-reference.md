# API Integration Reference

## Overview

This guide covers integration with external APIs used in the AI & Humanoid Robotics course: Anthropic Claude API for LLM reasoning, OpenAI API for embeddings, Qdrant for vector search, and Neon PostgreSQL for data storage.

## Anthropic Claude API

### Authentication

```python
import anthropic
import os

# Initialize client
client = anthropic.Anthropic(
    api_key=os.environ.get("ANTHROPIC_API_KEY")
)
```

### Available Models

| Model | Context Window | Cost (Input/Output per 1M tokens) | Best For |
|-------|----------------|-----------------------------------|----------|
| Claude 3.5 Sonnet | 200K | $3 / $15 | Production (balanced speed/quality) |
| Claude 3 Opus | 200K | $15 / $75 | Complex reasoning tasks |
| Claude 3 Haiku | 200K | $0.25 / $1.25 | Fast, simple tasks |

### Basic Usage

```python
# Simple message
message = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    messages=[
        {"role": "user", "content": "Explain ROS 2 publishers"}
    ]
)

print(message.content[0].text)
```

### Tool Use (Structured Outputs)

```python
# Define tool
tools = [{
    "name": "get_robot_status",
    "description": "Get current robot status including battery and position",
    "input_schema": {
        "type": "object",
        "properties": {
            "include_battery": {"type": "boolean"},
            "include_position": {"type": "boolean"}
        },
        "required": ["include_battery"]
    }
}]

# Call with tools
message = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    tools=tools,
    messages=[
        {"role": "user", "content": "Check robot battery and position"}
    ]
)

# Extract tool use
for block in message.content:
    if block.type == "tool_use":
        print(f"Tool: {block.name}")
        print(f"Input: {block.input}")
```

### Vision Capabilities

```python
import base64

# Read image
with open("robot_camera.jpg", "rb") as f:
    image_data = base64.b64encode(f.read()).decode('utf-8')

# Send image + text
message = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    messages=[{
        "role": "user",
        "content": [
            {
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": "image/jpeg",
                    "data": image_data
                }
            },
            {
                "type": "text",
                "text": "What objects do you see in this image?"
            }
        ]
    }]
)
```

### Streaming Responses

```python
# Stream for real-time output
with client.messages.stream(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    messages=[{"role": "user", "content": "Explain SLAM"}]
) as stream:
    for text in stream.text_stream:
        print(text, end="", flush=True)
```

### Rate Limits & Best Practices

- **Rate Limits**: Tier-based (starts at 50 RPM)
- **Best Practices**:
  - Use prompt caching for repeated system prompts
  - Batch requests when possible
  - Implement exponential backoff
  - Monitor token usage

```python
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10)
)
def call_claude_with_retry(prompt):
    return client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=1024,
        messages=[{"role": "user", "content": prompt}]
    )
```

## OpenAI API

### Embeddings for RAG

```python
from openai import OpenAI

client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

# Generate embeddings
response = client.embeddings.create(
    model="text-embedding-3-small",
    input=["ROS 2 is a middleware framework", "Gazebo simulates robots"]
)

embeddings = [item.embedding for item in response.data]
# Shape: (2, 1536)
```

### Model Options

| Model | Dimensions | Cost per 1M tokens | Use Case |
|-------|------------|-------------------|----------|
| text-embedding-3-small | 1536 | $0.02 | Production (recommended) |
| text-embedding-3-large | 3072 | $0.13 | Higher quality needed |
| text-embedding-ada-002 | 1536 | $0.10 | Legacy |

### Batch Processing

```python
# Process multiple texts
texts = ["text1", "text2", "text3", ...]  # Up to 2048 texts

response = client.embeddings.create(
    model="text-embedding-3-small",
    input=texts
)

embeddings = [item.embedding for item in response.data]
```

### Error Handling

```python
from openai import OpenAIError, RateLimitError

try:
    response = client.embeddings.create(
        model="text-embedding-3-small",
        input=text
    )
except RateLimitError:
    # Wait and retry
    time.sleep(60)
    response = client.embeddings.create(...)
except OpenAIError as e:
    print(f"API error: {e}")
```

## Qdrant Vector Database

### Initialization

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection
client.create_collection(
    collection_name="my_collection",
    vectors_config=VectorParams(
        size=1536,  # Must match embedding dimension
        distance=Distance.COSINE
    )
)
```

### Inserting Vectors

```python
from qdrant_client.models import PointStruct

# Insert points
client.upsert(
    collection_name="my_collection",
    points=[
        PointStruct(
            id=1,
            vector=[0.1, 0.2, ...],  # 1536 dimensions
            payload={
                "text": "Original text",
                "metadata": {"module": "01", "chapter": "02"}
            }
        ),
        PointStruct(id=2, vector=[...], payload={...})
    ]
)
```

### Searching

```python
# Basic search
results = client.search(
    collection_name="my_collection",
    query_vector=[0.15, 0.25, ...],  # Query embedding
    limit=5
)

for result in results:
    print(f"Score: {result.score}")
    print(f"Text: {result.payload['text']}")
```

### Filtered Search

```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

# Search with filter
results = client.search(
    collection_name="my_collection",
    query_vector=query_embedding,
    query_filter=Filter(
        must=[
            FieldCondition(
                key="metadata.module",
                match=MatchValue(value="01")
            )
        ]
    ),
    limit=5
)
```

### Batch Operations

```python
# Batch upsert
points = [
    PointStruct(id=i, vector=embeddings[i], payload=payloads[i])
    for i in range(len(embeddings))
]

client.upsert(
    collection_name="my_collection",
    points=points,
    wait=True  # Wait for operation to complete
)
```

## Neon PostgreSQL

### Connection

```python
import psycopg2
from psycopg2.pool import SimpleConnectionPool

# Connection string
DATABASE_URL = os.getenv("DATABASE_URL")

# Single connection
conn = psycopg2.connect(DATABASE_URL)
cur = conn.cursor()

# Connection pool (production)
pool = SimpleConnectionPool(
    minconn=1,
    maxconn=10,
    dsn=DATABASE_URL
)
```

### Basic Operations

```python
# Insert
cur.execute(
    "INSERT INTO documents (title, url) VALUES (%s, %s)",
    ("ROS 2 Guide", "/docs/ros2")
)
conn.commit()

# Query
cur.execute("SELECT * FROM documents WHERE module = %s", ("01",))
rows = cur.fetchall()

# Update
cur.execute(
    "UPDATE documents SET updated_at = NOW() WHERE id = %s",
    (doc_id,)
)
conn.commit()
```

### Transactions

```python
try:
    cur.execute("BEGIN")

    cur.execute("INSERT INTO sessions (id) VALUES (%s)", (session_id,))
    cur.execute("INSERT INTO messages (session_id, content) VALUES (%s, %s)",
                (session_id, message))

    conn.commit()
except Exception as e:
    conn.rollback()
    print(f"Transaction failed: {e}")
```

### Prepared Statements

```python
# More efficient for repeated queries
cur.execute("PREPARE insert_msg AS INSERT INTO messages (session_id, content) VALUES ($1, $2)")

# Execute prepared statement
for msg in messages:
    cur.execute("EXECUTE insert_msg (%s, %s)", (session_id, msg))

conn.commit()
```

## Complete RAG Pipeline Example

```python
from openai import OpenAI
from qdrant_client import QdrantClient
import anthropic
import psycopg2

class RAGPipeline:
    def __init__(self):
        self.openai = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.qdrant = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.claude = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))
        self.db = psycopg2.connect(os.getenv("DATABASE_URL"))

    def query(self, user_question: str, session_id: str = None):
        """Process RAG query."""

        # 1. Generate query embedding
        response = self.openai.embeddings.create(
            model="text-embedding-3-small",
            input=user_question
        )
        query_embedding = response.data[0].embedding

        # 2. Search vector database
        results = self.qdrant.search(
            collection_name="course_docs",
            query_vector=query_embedding,
            limit=5
        )

        # 3. Format context
        context = "\n\n".join([
            f"[Source {i+1}] {r.payload['title']}\n{r.payload['text']}"
            for i, r in enumerate(results)
        ])

        # 4. Generate response with Claude
        message = self.claude.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=2048,
            system=f"You are a helpful assistant. Use this context:\n\n{context}",
            messages=[{"role": "user", "content": user_question}]
        )

        answer = message.content[0].text

        # 5. Log to database
        cur = self.db.cursor()
        cur.execute(
            """
            INSERT INTO query_analytics (session_id, query, contexts_retrieved, tokens_used)
            VALUES (%s, %s, %s, %s)
            """,
            (session_id, user_question, len(results), message.usage.total_tokens)
        )
        self.db.commit()
        cur.close()

        return {
            'answer': answer,
            'sources': [r.payload for r in results],
            'tokens_used': message.usage.total_tokens
        }

# Usage
rag = RAGPipeline()
result = rag.query("How do I create a ROS 2 package?")
print(result['answer'])
```

## Cost Estimation

### Example Usage (1000 queries/day)

```
Embeddings (OpenAI):
- Query: 1000 * 50 tokens = 50K tokens
- Documents: 1000 * 5 * 500 tokens = 2.5M tokens
- Cost: (50K + 2.5M) / 1M * $0.02 = $0.051/day

LLM (Claude 3.5 Sonnet):
- Input: 1000 * (2000 context + 50 query) = 2.05M tokens
- Output: 1000 * 500 tokens = 500K tokens
- Cost: (2.05M / 1M * $3) + (500K / 1M * $15) = $13.65/day

Qdrant Cloud:
- 1GB cluster: ~$25/month = $0.83/day

Neon PostgreSQL:
- Scale tier: $19/month = $0.63/day

Total: ~$15.15/day or $455/month
```

## Rate Limiting Strategies

```python
from redis import Redis
import time

class RateLimiter:
    def __init__(self, redis_url="redis://localhost:6379"):
        self.redis = Redis.from_url(redis_url)

    def check_limit(self, user_id: str, limit: int = 10, window: int = 60):
        """Check if user exceeded rate limit."""
        key = f"ratelimit:{user_id}"
        count = self.redis.incr(key)

        if count == 1:
            self.redis.expire(key, window)

        if count > limit:
            ttl = self.redis.ttl(key)
            raise Exception(f"Rate limit exceeded. Try again in {ttl}s")

        return count

# Usage
limiter = RateLimiter()
try:
    limiter.check_limit(user_id="user123", limit=10, window=60)
    # Process request
except Exception as e:
    return {"error": str(e)}
```

## Security Best Practices

1. **API Keys**: Never commit to git, use environment variables
2. **Input Validation**: Sanitize user inputs before API calls
3. **Rate Limiting**: Prevent abuse and cost overruns
4. **Logging**: Log all API calls for debugging and auditing
5. **Error Handling**: Never expose API errors to end users

## References

- [Anthropic API Docs](https://docs.anthropic.com/)
- [OpenAI API Docs](https://platform.openai.com/docs/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Neon PostgreSQL](https://neon.tech/docs)
