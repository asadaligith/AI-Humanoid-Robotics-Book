# Quickstart Guide: RAG Chatbot Implementation

**Feature**: 001-rag-chatbot
**Date**: 2025-12-09
**Audience**: Developers implementing the RAG chatbot

## Overview

This guide provides step-by-step instructions to implement the RAG chatbot from scratch. Follow these phases in order.

---

## Prerequisites

**Required Accounts**:
- OpenAI Platform account (for API key and billing)
- Qdrant Cloud account (free tier)
- Neon Serverless Postgres account (free tier)
- Render/Fly/Railway account (for deployment)
- GitHub account (for repository access)

**Local Development Environment**:
- Python 3.11+ installed
- Node.js 18+ (for Docusaurus development)
- Git configured
- Text editor (VS Code recommended)

**API Keys Needed**:
- `OPENAI_API_KEY`: From OpenAI Platform dashboard
- `QDRANT_URL` and `QDRANT_API_KEY`: From Qdrant Cloud console
- `DATABASE_URL`: From Neon Postgres project settings

---

## Phase 1: Setup Infrastructure (30 minutes)

### Step 1.1: Create Qdrant Collection

1. Sign up at https://cloud.qdrant.io (free tier)
2. Create a new cluster (select closest region)
3. Create collection via Qdrant console or API:
   - Collection name: `book_content`
   - Vector size: 1536
   - Distance metric: Cosine
4. Save `QDRANT_URL` and `QDRANT_API_KEY` for later

### Step 1.2: Setup Neon Postgres

1. Sign up at https://neon.tech (free tier)
2. Create a new project: "rag-chatbot-db"
3. Copy `DATABASE_URL` (PostgreSQL connection string)
4. Run schema creation (see `data-model.md` for SQL)

### Step 1.3: Get OpenAI API Key

1. Sign in to https://platform.openai.com
2. Navigate to API Keys section
3. Create new secret key: "rag-chatbot-key"
4. Save `OPENAI_API_KEY` securely (you won't see it again)
5. Add billing information (required for text-embedding-3-large)

---

## Phase 2: Backend Development (4-6 hours)

### Step 2.1: Project Structure

Create the following directory structure:

```
backend/
├── src/
│   ├── __init__.py
│   ├── main.py                 # FastAPI app entry point
│   ├── config.py               # Environment variables
│   ├── models/
│   │   ├── __init__.py
│   │   ├── requests.py         # Pydantic request models
│   │   └── responses.py        # Pydantic response models
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embeddings.py       # OpenAI embedding service
│   │   ├── retrieval.py        # Qdrant retrieval logic
│   │   ├── agent.py            # OpenAI Agent SDK integration
│   │   └── database.py         # Postgres connection
│   ├── routes/
│   │   ├── __init__.py
│   │   ├── chat.py             # /ask and /ask-selected endpoints
│   │   └── health.py           # /health endpoint
│   └── utils/
│       ├── __init__.py
│       ├── chunking.py         # Text chunking logic
│       └── hashing.py          # SHA256 hashing
├── scripts/
│   └── reindex_book.py         # Indexing script
├── tests/
│   ├── test_api.py
│   ├── test_retrieval.py
│   └── test_agent.py
├── requirements.txt
├── Dockerfile
├── render.yaml
└── .env.example
```

### Step 2.2: Install Dependencies

Create `requirements.txt`:

```
fastapi==0.109.0
uvicorn[standard]==0.27.0
pydantic==2.5.3
pydantic-settings==2.1.0
openai==1.12.0
qdrant-client==1.7.0
psycopg2-binary==2.9.9
sqlalchemy==2.0.25
python-dotenv==1.0.0
tiktoken==0.5.2
langchain==0.1.5
langchain-community==0.0.16
```

Install: `pip install -r requirements.txt`

### Step 2.3: Implement Core Services

**Priority Order**:
1. `config.py` - Load environment variables
2. `database.py` - Postgres connection
3. `embeddings.py` - OpenAI embedding generation
4. `chunking.py` - Text chunking logic
5. `retrieval.py` - Qdrant search
6. `agent.py` - OpenAI Agent integration
7. `routes/chat.py` - API endpoints
8. `routes/health.py` - Health check
9. `main.py` - FastAPI app assembly

**Testing**: Test each service independently before integration.

### Step 2.4: Create Indexing Script

Implement `scripts/reindex_book.py`:
- Read markdown files from `/docs/**`
- Chunk using 800 tokens, 200 overlap
- Generate embeddings
- Store in Qdrant and Postgres
- Handle duplicates via content hash

**Run**: `python scripts/reindex_book.py --source ../docs/`

---

## Phase 3: Frontend Integration (2-3 hours)

### Step 3.1: Create Chatbot Widget

Create `static/js/chatbot-widget.js` in Docusaurus:

**Components**:
1. Floating button (bottom-right corner)
2. Chat panel (slides in from right)
3. Message input field
4. Submit button
5. Messages container (displays Q&A)
6. Selected text handler (listens for text selection)

**Styling**: Use Docusaurus theme variables for consistency.

### Step 3.2: Integrate with Docusaurus

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  // ... existing config
  scripts: [
    '/js/chatbot-widget.js'
  ],
  // ... rest of config
};
```

### Step 3.3: Implement API Communication

Add fetch logic to widget:

```javascript
async function askQuestion(question, sessionId) {
  const response = await fetch('https://rag-chatbot-api.onrender.com/ask', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ question, session_id: sessionId })
  });
  return await response.json();
}
```

**Error Handling**: Show friendly messages for 500/503 errors.

---

## Phase 4: Deployment (1-2 hours)

### Step 4.1: Create Dockerfile

```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY src/ ./src/

EXPOSE 8000

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Step 4.2: Deploy to Render

1. Create `render.yaml`:

```yaml
services:
  - type: web
    name: rag-chatbot-api
    env: python
    buildCommand: "pip install -r requirements.txt"
    startCommand: "uvicorn src.main:app --host 0.0.0.0 --port $PORT"
    envVars:
      - key: OPENAI_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: DATABASE_URL
        sync: false
    healthCheckPath: /health
```

2. Connect GitHub repo to Render
3. Add environment variables in Render dashboard
4. Deploy (automatic on git push)

### Step 4.3: Update CORS Settings

In `main.py`, add Docusaurus domain:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://asadaligith.github.io"],
    allow_methods=["POST", "GET"],
    allow_headers=["Content-Type"],
)
```

---

## Phase 5: Testing & Validation (2-3 hours)

### Step 5.1: Unit Tests

Test each component:
- Chunking: Verify 800-token chunks with overlap
- Embedding: Check vector dimensions (1536)
- Retrieval: Test top-5 retrieval with threshold
- Agent: Verify citations in responses

Run: `pytest tests/`

### Step 5.2: Integration Tests

Test full flow:
1. Ask question → verify answer with citations
2. Ask about selected text → verify context used
3. Multi-turn conversation → verify session tracking
4. Off-topic question → verify refusal message

### Step 5.3: Performance Tests

Measure:
- Response time (target: <2s P95)
- Concurrent requests (10 simultaneous)
- Embedding generation time (<500ms for 1 query)

### Step 5.4: User Acceptance Tests

Manual testing:
- Open chatbot widget
- Ask 10 diverse questions (see test plan in research.md)
- Verify citations are accurate
- Check UI responsiveness

---

## Phase 6: Monitoring & Iteration (Ongoing)

### Step 6.1: Setup Logging

Add structured logging:
- Request/response times
- Retrieval scores
- Error rates
- OpenAI API usage

### Step 6.2: Monitor Costs

Track:
- OpenAI embedding costs (per query)
- OpenAI chat completion costs (per answer)
- Qdrant/Neon/Render usage (should stay in free tier)

**Budget**: Estimate $5-10/month for 1000 queries/month.

### Step 6.3: Iterate on Prompts

If hallucination rate >5%:
- Strengthen system prompt
- Increase similarity threshold (0.7 → 0.75)
- Reduce context window (5 chunks → 3 chunks)

---

## Troubleshooting

### Common Issues

**Issue**: "Qdrant connection refused"
- **Fix**: Check `QDRANT_URL` and `QDRANT_API_KEY` are correct
- **Test**: `curl -H "api-key: $QDRANT_API_KEY" $QDRANT_URL/collections`

**Issue**: "OpenAI rate limit exceeded"
- **Fix**: Add retry logic with exponential backoff
- **Alternative**: Switch to gpt-4-turbo (higher rate limits)

**Issue**: "Response too slow (>2s)"
- **Fix**: Check Qdrant search time (should be <100ms)
- **Optimize**: Reduce retrieval to top-3 chunks

**Issue**: "Chatbot widget not loading"
- **Fix**: Check browser console for CORS errors
- **Fix**: Verify script path in `docusaurus.config.js`

---

## Success Criteria Checklist

Before launching:

- [ ] Backend API returns 200 for `/health`
- [ ] 300+ chunks indexed in Qdrant
- [ ] Postgres metadata table populated
- [ ] Chatbot widget visible on book pages
- [ ] Can ask question and receive cited answer in <2s
- [ ] Off-topic questions are refused
- [ ] Selected text feature works (P2)
- [ ] Multi-turn conversations maintain context (P3)
- [ ] CORS configured for GitHub Pages domain
- [ ] Environment variables secured (not in git)
- [ ] Deployment auto-updates on git push

---

## Next Steps After MVP

**Enhancements**:
1. Add feedback buttons (thumbs up/down)
2. Implement conversation history persistence
3. Add analytics dashboard
4. A/B test different chunk sizes
5. Add "Ask about this diagram" for images
6. Implement semantic caching for common questions

**Scaling**:
- Upgrade to Render Starter plan if traffic increases
- Add Redis for caching frequent queries
- Implement query preprocessing (spell check, autocomplete)

---

## Resources

**Documentation**:
- FastAPI: https://fastapi.tiangolo.com
- OpenAI API: https://platform.openai.com/docs
- Qdrant: https://qdrant.tech/documentation
- Neon Postgres: https://neon.tech/docs
- Docusaurus: https://docusaurus.io

**Community**:
- FastAPI Discord: https://discord.gg/fastapi
- OpenAI Forum: https://community.openai.com
- Qdrant Discord: https://discord.gg/qdrant

**Support**:
- GitHub Issues: [project-repo]/issues
- Email: [contact-email]

---

**Estimated Total Time**: 10-15 hours (spread over 2-3 days)

**Recommended Approach**: Implement MVP (P1 features only) first, then add P2/P3 features based on user feedback.
