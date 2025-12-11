# 🚀 Quick Start Guide - RAG Chatbot Backend

Get your RAG chatbot backend running in **5 simple steps**!

## Prerequisites Check

✅ Python 3.11+ installed
✅ Gemini API key added to `.env.example`
✅ Qdrant URL and API key added to `.env.example`
✅ Neon Postgres credentials ready

---

## Step 1: Setup Environment

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate

# Mac/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

---

## Step 2: Configure Environment Variables

```bash
# Copy .env.example to .env
copy .env.example .env  # Windows
# or
cp .env.example .env    # Mac/Linux

# Edit .env and verify your credentials:
# - GEMINI_API_KEY (already set)
# - QDRANT_URL (already set)
# - QDRANT_API_KEY (already set)
# - DATABASE_URL (add your Neon connection string)
```

**Neon Database URL Format:**
```
postgresql://username:password@ep-xxx-xxx.region.aws.neon.tech/dbname?sslmode=require
```

Get it from: https://console.neon.tech → Your Project → Connection Details

---

## Step 3: Initialize Infrastructure

```bash
# Run setup script (creates Qdrant collection + Postgres tables)
python scripts/setup_infrastructure.py
```

**Expected Output:**
```
🚀 RAG Chatbot Infrastructure Setup
====================================

✅ Gemini API is working!
✅ Collection created successfully!
✅ Tables created successfully!

🎉 All systems ready!
```

**Troubleshooting:**
- If Qdrant fails: Check QDRANT_URL format (must include :6333 port)
- If Postgres fails: Verify DATABASE_URL connection string
- If Gemini fails: Verify API key at https://aistudio.google.com/app/apikey

---

## Step 4: Index Book Content

```bash
# Index all markdown files from /docs/ directory
python scripts/reindex_book.py --source ../docs

# Or do a dry run first to see what would be indexed:
python scripts/reindex_book.py --source ../docs --dry-run
```

**Expected Output:**
```
📂 Reading markdown files from: ../docs
   ✓ /intro.md
   ✓ /chapter1.md
   ...
📊 Found 50 markdown files

🔄 Processing files...
   ✓ Indexed chunk 0: Introduction to robotics...
   ✓ Indexed chunk 1: Key components include...
   ...

📊 Indexing Statistics
Files processed:      50
Total chunks:         300
New chunks indexed:   300
Skipped (unchanged):  0
Errors:               0

✅ Indexing complete!
```

**This will take ~5-10 minutes** depending on:
- Number of markdown files
- Gemini API rate limits (60 requests/min on free tier)
- Network speed

---

## Step 5: Start the API Server

```bash
# Start FastAPI server with auto-reload
uvicorn src.main:app --reload --port 8000

# Or run directly:
python -m src.main
```

**Expected Output:**
```
🚀 RAG Chatbot API Starting
====================================
Environment: development
CORS Origins: http://localhost:3000, https://asadaligith.github.io
Similarity Threshold: 0.7
Max Chunks: 5
====================================

INFO:     Uvicorn running on http://127.0.0.1:8000
INFO:     Application startup complete.
```

---

## ✅ Verify Everything Works

### Test 1: Health Check

```bash
curl http://localhost:8000/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "services": [
    {"name": "qdrant", "healthy": true, "latency_ms": 45},
    {"name": "postgres", "healthy": true, "latency_ms": 23},
    {"name": "gemini", "healthy": true, "latency_ms": 312}
  ],
  "version": "1.0.0"
}
```

### Test 2: Ask a Question

```bash
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key components of a humanoid robot?"
  }'
```

**Expected Response:**
```json
{
  "answer": "Humanoid robots consist of several key components including...",
  "sources": [
    {
      "file": "/docs/chapter1.md",
      "section": "Introduction to Humanoid Robots",
      "chunk": "The main components include sensors, actuators...",
      "similarity": 0.85
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "latency_ms": 1234
}
```

### Test 3: Interactive API Documentation

Visit: http://localhost:8000/docs

- Try out endpoints interactively
- See request/response schemas
- Test with different questions

---

## 🎯 Next Steps

Now that your backend is running:

1. **Test Various Questions:**
   - Ask about different chapters
   - Try technical vs conceptual questions
   - Test hallucination prevention with off-topic questions

2. **Monitor Performance:**
   - Check `/health` endpoint regularly
   - Monitor response latency (target: <2s)
   - Watch Gemini API usage (60 RPM free tier)

3. **Implement Frontend:**
   - Create chatbot widget (vanilla JavaScript)
   - Integrate with Docusaurus
   - Deploy to GitHub Pages

4. **Deploy to Production:**
   - Push code to GitHub
   - Connect Render to your repository
   - Set environment variables in Render dashboard
   - Deploy!

---

## 🐛 Common Issues & Solutions

### Issue: "ModuleNotFoundError: No module named 'src'"

**Solution:**
```bash
# Make sure you're in the backend/ directory
cd backend

# Reinstall dependencies
pip install -r requirements.txt
```

### Issue: "Qdrant connection refused"

**Solution:**
- Check QDRANT_URL includes port :6333
- Verify API key is correct
- Test connection: https://your-cluster.qdrant.io:6333/dashboard

### Issue: "Gemini API quota exceeded"

**Solution:**
- Free tier: 60 requests/minute
- Add delays between requests in reindex script
- Or upgrade to paid tier

### Issue: "No chunks found" when asking questions

**Solution:**
- Verify indexing completed successfully
- Check Qdrant: `curl http://localhost:8000/health`
- Re-run indexing: `python scripts/reindex_book.py --source ../docs`

### Issue: "Database connection failed"

**Solution:**
- Verify DATABASE_URL format
- Check Neon dashboard for database status
- Ensure `?sslmode=require` is in connection string

---

## 📊 Monitoring & Logs

### View API Logs
```bash
# Uvicorn shows all requests
INFO:     127.0.0.1:53951 - "POST /api/ask HTTP/1.1" 200 OK
INFO:     Response time: 1.234s
```

### Check Collection Stats
```python
from src.services.retrieval import get_collection_info
print(get_collection_info())
# {'points_count': 300, 'vectors_count': 300, 'status': 'green'}
```

### Check Database Stats
```python
from src.services.database import get_db, ChunkMetadata
with get_db() as db:
    count = db.query(ChunkMetadata).count()
    print(f"Total chunks: {count}")
```

---

## 🎉 Success!

You now have a fully functional RAG chatbot backend powered by:
- ✨ **Google Gemini** (768D embeddings + 1.5 Pro generation)
- 🔍 **Qdrant** (vector search with cosine similarity)
- 💾 **Neon Postgres** (metadata storage)
- ⚡ **FastAPI** (RESTful API)

**API Endpoints Available:**
- `GET /health` - Health check
- `POST /api/ask` - Ask questions
- `POST /api/ask-selected` - Ask about selected text
- `GET /docs` - Interactive API documentation

**Next:** Implement the frontend chatbot widget! 🚀

---

## 📚 Additional Resources

- **Backend README**: Full documentation in `backend/README.md`
- **API Docs**: http://localhost:8000/docs (when running)
- **Gemini Docs**: https://ai.google.dev/docs
- **Qdrant Docs**: https://qdrant.tech/documentation
- **FastAPI Docs**: https://fastapi.tiangolo.com

Need help? Check `backend/README.md` or open an issue on GitHub!
