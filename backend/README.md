# RAG Chatbot Backend

AI-powered chatbot backend for the AI Humanoid Robotics Book using Google Gemini, Qdrant, and Neon Postgres.

## 🎯 Overview

This backend provides a **Retrieval-Augmented Generation (RAG)** system that:
- Indexes book content (markdown files) into a vector database
- Answers user questions with accurate, cited responses
- Uses **Google Gemini** for embeddings and answer generation
- Prevents hallucinations through strict grounding to book content

## 🏗️ Architecture

```
User Question
    ↓
[Gemini Embeddings] → Query Vector (768D)
    ↓
[Qdrant Search] → Top 5 Similar Chunks
    ↓
[Gemini 1.5 Pro] → Answer + Citations
    ↓
Response to User
```

## 🔑 Key Technologies

- **Google Gemini** (`text-embedding-004`, `gemini-1.5-pro-latest`)
  - 768-dimensional embeddings
  - Advanced reasoning for RAG
- **Qdrant** - Vector database (cosine similarity)
- **Neon Postgres** - Metadata storage
- **FastAPI** - REST API framework
- **LangChain** - Text chunking utilities

## 📁 Project Structure

```
backend/
├── src/
│   ├── models/          # Pydantic request/response models
│   │   ├── requests.py
│   │   └── responses.py
│   ├── services/        # Business logic
│   │   ├── embeddings.py   # Gemini embedding generation
│   │   ├── retrieval.py    # Qdrant search
│   │   ├── agent.py        # Gemini answer generation
│   │   └── database.py     # Postgres connection
│   ├── routes/          # API endpoints
│   │   ├── chat.py
│   │   └── health.py
│   ├── utils/           # Utilities
│   │   ├── chunking.py     # Text splitting (800 tokens, 200 overlap)
│   │   ├── hashing.py      # SHA256 content hashing
│   │   └── session.py      # Conversation session management
│   ├── config.py        # Configuration management
│   └── main.py          # FastAPI app (TO BE CREATED)
├── scripts/
│   └── reindex_book.py  # Indexing script (TO BE CREATED)
├── tests/               # Test suite
│   └── conftest.py      # Pytest fixtures
├── requirements.txt     # Python dependencies
├── Dockerfile           # Docker build configuration
├── render.yaml          # Deployment configuration
└── .env.example         # Environment variables template
```

## 🚀 Quick Start

### Prerequisites

1. **Google Gemini API Key**
   - Get it from: https://aistudio.google.com/app/apikey
   - Free tier: 60 requests per minute

2. **Qdrant Cloud Account**
   - Sign up at: https://cloud.qdrant.io
   - Free tier: 1GB storage

3. **Neon Postgres Account**
   - Sign up at: https://neon.tech
   - Free tier: 3GB storage

### Installation

1. **Clone and navigate to backend:**
   ```bash
   cd backend
   ```

2. **Create virtual environment:**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure environment variables:**
   ```bash
   cp .env.example .env
   # Edit .env with your actual credentials
   ```

### Setup Infrastructure

#### 1. Create Qdrant Collection

```python
from src.services.retrieval import create_collection
create_collection()
```

Or use Qdrant console to create collection:
- Name: `book_content`
- Vector size: **768** (Gemini text-embedding-004)
- Distance: Cosine

#### 2. Setup Neon Postgres

Run this SQL in Neon console:

```sql
CREATE TABLE chunks_metadata (
  chunk_id UUID PRIMARY KEY,
  file_path VARCHAR(512) NOT NULL,
  section_heading VARCHAR(512),
  chunk_index INTEGER NOT NULL,
  content_hash CHAR(64) NOT NULL UNIQUE,
  token_count INTEGER NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  CONSTRAINT chunk_index_non_negative CHECK (chunk_index >= 0),
  CONSTRAINT token_count_range CHECK (token_count BETWEEN 100 AND 2000)
);

CREATE INDEX idx_file_path ON chunks_metadata(file_path);
CREATE INDEX idx_section_heading ON chunks_metadata(section_heading);
CREATE INDEX idx_content_hash ON chunks_metadata(content_hash);
CREATE INDEX idx_created_at ON chunks_metadata(created_at);
```

#### 3. Index Book Content

Once the reindex script is created:

```bash
python scripts/reindex_book.py --source ../docs/
```

This will:
- Read all markdown files from `/docs/**`
- Chunk into 800-token segments (200 overlap)
- Generate embeddings with Gemini
- Store in Qdrant + Postgres

## 📋 Implementation Status

### ✅ Completed (Phase 1 & 2 - Partial)

- [x] Backend project structure
- [x] Python dependencies configuration
- [x] Docker and deployment configuration
- [x] Pytest test suite setup
- [x] Configuration management (config.py)
- [x] Database models (SQLAlchemy ORM)
- [x] **Gemini embeddings service** (text-embedding-004, 768D)
- [x] **Gemini agent service** (gemini-1.5-pro with RAG)
- [x] Qdrant retrieval service with hybrid scoring
- [x] Session management for multi-turn conversations
- [x] Text chunking utilities (800 tokens, 25% overlap)
- [x] Content hashing for deduplication
- [x] Pydantic request/response models

### ⏳ Next Steps

#### Phase 2 - Foundational (Remaining)

- [ ] **T007**: Create Qdrant collection (manual - see setup above)
- [ ] **T008**: Setup Neon Postgres database (manual - see SQL above)
- [ ] **T016**: Create `scripts/reindex_book.py` script
- [ ] **T017**: Run indexing script to populate ~300 chunks

#### Phase 3 - User Story 1 (MVP)

- [ ] **T018-T020**: Write tests for retrieval, hallucination prevention, integration
- [ ] **T021-T027**: Implement API routes (POST /ask, GET /health)
- [ ] **T028-T031**: Create frontend chatbot widget (JavaScript)

## 🔧 Configuration

### Environment Variables

```bash
# Gemini API (required)
GEMINI_API_KEY=your-gemini-api-key

# Qdrant (required)
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres (required)
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# Application settings
ENVIRONMENT=development
ALLOWED_ORIGINS=http://localhost:3000,https://asadaligith.github.io
RATE_LIMIT_PER_MINUTE=10
SIMILARITY_THRESHOLD=0.7
MAX_CHUNKS=5
```

## 🧪 Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=html

# Run specific test category
pytest -m unit
pytest -m integration
```

## 📊 API Endpoints (To Be Implemented)

### POST /ask
Ask a general question about the book.

**Request:**
```json
{
  "question": "What are the key components of a humanoid robot?",
  "session_id": "optional-uuid"
}
```

**Response:**
```json
{
  "answer": "Humanoid robots consist of...",
  "sources": [
    {
      "file": "/docs/chapter1.md",
      "section": "Introduction",
      "chunk": "Humanoid robots...",
      "similarity": 0.85
    }
  ],
  "session_id": "550e8400-...",
  "latency_ms": 1200
}
```

### POST /ask-selected
Ask about selected text with focused context.

### GET /health
Health check endpoint for monitoring.

## 🚢 Deployment

### Docker Build

```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

### Render Deployment

1. Push code to GitHub
2. Connect repository to Render
3. Set environment variables in Render dashboard
4. Deploy automatically via `render.yaml`

---

## ✅ Deployment Checklist

Use this checklist before deploying to production to ensure all components are properly configured.

### Pre-Deployment Verification

#### 1. Environment Variables
- [ ] `OPENAI_API_KEY` set and valid (test with a simple API call)
- [ ] `QDRANT_URL` set to your Qdrant cluster URL
- [ ] `QDRANT_API_KEY` set and valid
- [ ] `DATABASE_URL` set to Neon Postgres connection string
- [ ] `ENVIRONMENT` set to `production`
- [ ] `ALLOWED_ORIGINS` includes your production domain(s)
- [ ] `RATE_LIMIT_PER_MINUTE` configured (recommended: 10)
- [ ] `SIMILARITY_THRESHOLD` set (default: 0.65)
- [ ] `MAX_CHUNKS` set (default: 5)

**Verify with:**
```bash
python -c "from src.config import settings; print(settings.openai_api_key[:10] + '...')"
```

#### 2. Qdrant Vector Database
- [ ] Collection `book_content` exists
- [ ] Collection has correct vector size (1536D for OpenAI text-embedding-3-small)
- [ ] Collection uses Cosine distance metric
- [ ] Payload indexes created for: `chunk_id`, `file_path`, `section_heading`, `content_hash`
- [ ] At least 800 chunks indexed (verify count)

**Verify with:**
```python
from src.services.retrieval import get_collection_info
info = get_collection_info()
print(f"Chunks indexed: {info['points_count']}")  # Should be 800+
```

#### 3. Neon Postgres Database
- [ ] `chunks_metadata` table exists
- [ ] Table schema matches specification (see SQL above)
- [ ] Indexes created on: `file_path`, `section_heading`, `content_hash`, `created_at`
- [ ] At least 800 rows in `chunks_metadata` table
- [ ] SSL mode enabled in connection string (`?sslmode=require`)

**Verify with:**
```sql
SELECT COUNT(*) FROM chunks_metadata;  -- Should return 800+
SELECT COUNT(*) FROM chunks_metadata WHERE created_at > NOW() - INTERVAL '7 days';
```

#### 4. Content Indexing
- [ ] All book content indexed (run `scripts/reindex_book.py`)
- [ ] No duplicate content hashes (check `content_hash` uniqueness)
- [ ] Chunk token counts within valid range (100-2000 tokens)
- [ ] All file paths reference existing markdown files
- [ ] Section headings properly extracted from markdown

**Verify with:**
```bash
python scripts/reindex_book.py --dry-run  # Should show 800+ chunks to index
```

#### 5. API Health Checks
- [ ] GET `/health` endpoint returns `200 OK`
- [ ] `/health` response includes Qdrant and Postgres status
- [ ] Root `/` endpoint returns API information
- [ ] POST `/api/ask` endpoint accepts valid requests
- [ ] POST `/api/ask-selected` endpoint accepts valid requests

**Verify with:**
```bash
curl http://localhost:8000/health
# Expected: {"status": "healthy", "qdrant": "connected", "postgres": "connected"}

curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
# Expected: JSON response with answer and sources
```

#### 6. CORS Configuration
- [ ] `ALLOWED_ORIGINS` includes production frontend domain
- [ ] CORS allows methods: `GET`, `POST`, `OPTIONS`
- [ ] CORS allows headers: `Content-Type`, `Authorization`
- [ ] CORS credentials enabled if needed

**Verify with:**
```bash
curl -H "Origin: https://your-domain.com" \
     -H "Access-Control-Request-Method: POST" \
     -H "Access-Control-Request-Headers: Content-Type" \
     -X OPTIONS http://localhost:8000/api/ask
# Expected: Access-Control-Allow-Origin header in response
```

#### 7. Rate Limiting
- [ ] Rate limiting middleware enabled
- [ ] Rate limit set to reasonable value (10 requests/minute recommended)
- [ ] Rate limit applies per session ID
- [ ] 429 error returned when limit exceeded

**Test with:**
```bash
# Send 11 requests rapidly
for i in {1..11}; do
  curl -X POST http://localhost:8000/api/ask \
    -H "Content-Type: application/json" \
    -d '{"question": "test"}' &
done
wait
# Expected: 11th request should return 429 Too Many Requests
```

#### 8. Logging and Monitoring
- [ ] Structured JSON logging enabled
- [ ] Log level set appropriately (`INFO` for production)
- [ ] Request IDs generated for all requests (`X-Request-ID` header)
- [ ] Slow query logging enabled (threshold: 2000ms)
- [ ] Error logging includes full stack traces
- [ ] Performance metrics tracked (latency_ms)

**Verify with:**
```bash
# Check logs format
curl http://localhost:8000/health 2>&1 | grep -i "x-request-id"
# Expected: Header present in response
```

#### 9. Error Handling
- [ ] User-friendly error messages configured
- [ ] No technical stack traces exposed to users (in production mode)
- [ ] Proper HTTP status codes returned (400, 422, 429, 500, 503)
- [ ] Validation errors return clear messages
- [ ] Service unavailability handled gracefully

**Test with:**
```bash
# Test validation error
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{"question": ""}'
# Expected: 422 with clear message

# Test invalid SQL injection attempt
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "SELECT * FROM users; DROP TABLE--"}'
# Expected: 400 with "Invalid input: potential SQL injection detected"
```

#### 10. Security
- [ ] Input validation enabled (SQL injection prevention)
- [ ] XSS prevention (HTML tag stripping)
- [ ] Session ID validation (alphanumeric only)
- [ ] Question length limits enforced (max 2000 chars)
- [ ] Selected text length limits enforced (10-5000 chars)
- [ ] Secrets not exposed in logs or error messages

**Verify with:**
```bash
# Test XSS prevention
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "<script>alert(\"xss\")</script>What is ROS?"}'
# Expected: Script tags stripped from question
```

### Deployment Steps

1. **Pre-deployment checks:**
   ```bash
   # Run all verification scripts
   python -m pytest tests/
   python scripts/verify_deployment.py  # If created
   ```

2. **Deploy to staging first:**
   ```bash
   # Test in staging environment
   ENVIRONMENT=staging python -m uvicorn src.main:app --host 0.0.0.0 --port 8000
   ```

3. **Run smoke tests:**
   ```bash
   # Test all critical endpoints
   curl https://staging-api.your-domain.com/health
   curl -X POST https://staging-api.your-domain.com/api/ask -d '{"question": "test"}'
   ```

4. **Monitor initial traffic:**
   - Watch logs for errors
   - Check latency metrics (P95 should be <2s)
   - Verify error rate <5%

5. **Deploy to production:**
   ```bash
   # Use your deployment tool (Render, Vercel, etc.)
   git push origin main  # Triggers auto-deploy on Render
   ```

6. **Post-deployment verification:**
   - Run health check: `curl https://api.your-domain.com/health`
   - Send test query
   - Monitor logs for first 10 minutes
   - Check error reporting service (if configured)

### Rollback Plan

If issues detected after deployment:

1. **Immediate:**
   - Revert to previous deployment via Render dashboard
   - Or: `git revert <commit>` and push

2. **Investigate:**
   - Check structured logs for error patterns
   - Review performance metrics (slow queries)
   - Test locally with production data

3. **Fix and redeploy:**
   - Fix issue in development
   - Test thoroughly in staging
   - Deploy with monitoring

---

## 📝 Development Notes

### Gemini vs OpenAI Comparison

| Feature | OpenAI | Gemini (Current) |
|---------|--------|------------------|
| Embedding Model | text-embedding-3-large (1536D) | text-embedding-004 (768D) |
| Chat Model | gpt-4 / gpt-4-turbo | gemini-1.5-pro-latest |
| Cost (Embeddings) | $0.13/1M tokens | Free tier: 60 RPM |
| Cost (Chat) | $0.01/1K tokens | Free tier: 60 RPM |
| Context Window | 8K / 128K | 1M tokens |

**Key Advantages of Gemini:**
- 🆓 **Free tier** with generous limits
- 🧠 **Larger context window** (1M tokens)
- ⚡ **Faster for long documents**
- 🎯 **Task-specific embeddings** (RETRIEVAL_DOCUMENT vs RETRIEVAL_QUERY)

### Chunking Strategy

- **Size**: 800 tokens (~600 words)
- **Overlap**: 200 tokens (25%)
- **Splitter**: LangChain RecursiveCharacterTextSplitter
- **Markdown-aware**: Preserves headers and structure

### Similarity Threshold

- **Default**: 0.7 (cosine similarity)
- **Rationale**: Filters out irrelevant chunks
- **Tunable**: Via `SIMILARITY_THRESHOLD` env var

## 🤝 Contributing

1. Follow existing code structure
2. Add tests for new features
3. Update documentation
4. Run `black` and `flake8` for code formatting

## 📄 License

MIT License - See project root for details

## 🙋 Support

- **Issues**: https://github.com/asadaligith/AI-Humanoid-Robotics-Book/issues
- **Discussions**: GitHub Discussions
- **Documentation**: See `/specs/001-rag-chatbot/` for detailed design docs

---

**Status**: 🚧 In Development - Phase 2 (Foundational) ~60% Complete

**Next Milestone**: Complete API routes and test MVP chatbot
