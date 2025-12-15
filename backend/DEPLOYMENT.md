# Deployment Checklist - RAG Chatbot Backend

**Last Updated**: 2025-12-15
**Platform**: Render.com (Free Tier)
**Status**: Ready for Deployment

---

## Pre-Deployment Checklist

### 1. Code Verification ✅

- [x] FastAPI application created (`src/main.py`)
- [x] All routes implemented (`/health`, `/api/ask`)
- [x] Environment configuration ready (`src/config.py`)
- [x] Dependencies listed (`requirements.txt`)
- [x] Dockerfile configured
- [x] `.env` in `.gitignore`
- [x] Code committed to GitHub

### 2. External Services Setup

#### Gemini API ✅
- [x] API Key obtained from: https://aistudio.google.com/app/apikey
- [x] Key stored securely (ready for Render env vars)
- [x] Rate limits understood: 60 requests/minute (free tier)

#### Qdrant Vector Database ✅
- [x] Account created at: https://cloud.qdrant.io
- [x] Cluster URL obtained
- [x] API Key generated
- [ ] Collection `book_content` created (see instructions below)

#### Neon Postgres Database ⚠️
- [ ] Account created at: https://neon.tech
- [ ] Database created (name: `robotics_book`)
- [ ] Schema initialized (see SQL below)
- [ ] Connection string obtained

### 3. Configuration Files

- [x] `render.yaml` configured
- [x] `Dockerfile` ready
- [x] `.env.example` provided for reference
- [ ] All environment variables documented

---

## Deployment Steps

### Step 1: Setup Neon Postgres (Required)

1. **Create Account & Database**
   - Go to: https://neon.tech
   - Sign up with GitHub
   - Create new project: "AI Robotics Book"
   - Database name: `robotics_book`

2. **Initialize Database Schema**

   Go to SQL Editor in Neon dashboard and run:

   ```sql
   -- Create chunks metadata table
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

   -- Create indexes for performance
   CREATE INDEX idx_file_path ON chunks_metadata(file_path);
   CREATE INDEX idx_section_heading ON chunks_metadata(section_heading);
   CREATE INDEX idx_content_hash ON chunks_metadata(content_hash);
   CREATE INDEX idx_created_at ON chunks_metadata(created_at);
   ```

3. **Get Connection String**
   - Format: `postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/robotics_book?sslmode=require`
   - Copy this for Render environment variables

### Step 2: Setup Qdrant Collection (Optional - Can be done post-deployment)

**Option A: Using Qdrant Console**
1. Go to: https://cloud.qdrant.io/collections
2. Create collection:
   - Name: `book_content`
   - Vector size: **768** (Gemini text-embedding-004)
   - Distance: Cosine
   - On-disk payload: Enabled (for free tier)

**Option B: Using Python Script (Post-deployment)**
```python
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
client.create_collection(
    collection_name="book_content",
    vectors_config=VectorParams(size=768, distance=Distance.COSINE)
)
```

### Step 3: Deploy to Render.com

1. **Go to Render Dashboard**
   ```
   URL: https://dashboard.render.com
   ```
   - Sign in with GitHub
   - Authorize Render to access your repositories

2. **Create New Web Service**
   - Click: "New +" → "Web Service"
   - Connect repository: `asadaligith/AI-Humanoid-Robotics-Book`
   - Click "Connect"

3. **Configure Service Settings**

   | Setting | Value |
   |---------|-------|
   | **Name** | `rag-chatbot-api` |
   | **Region** | Oregon (or closest region) |
   | **Branch** | `master` |
   | **Root Directory** | `backend` |
   | **Runtime** | Python 3 |
   | **Build Command** | `pip install --upgrade pip && pip install -r requirements.txt` |
   | **Start Command** | `uvicorn src.main:app --host 0.0.0.0 --port $PORT --workers 1` |
   | **Instance Type** | Free (512 MB RAM, 0.1 CPU) |

4. **Add Environment Variables**

   Click "Advanced" → "Add Environment Variable":

   **Required Variables:**
   ```bash
   GEMINI_API_KEY=AIzaSyAJMgXHq9ol_YxmFB2PivcuGMvCRVWic3Y
   QDRANT_URL=https://278d0688-9339-4904-89e7-28e1b4d8d5c2.europe-west3-0.gcp.cloud.qdrant.io:6333
   QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.hE5rcb4c176F-gH0hzOfMskjBnbpEJBzhWwDwl6xl34
   DATABASE_URL=<paste-your-neon-connection-string-here>
   ```

   **Configuration Variables:**
   ```bash
   ENVIRONMENT=production
   ALLOWED_ORIGINS=https://asadaligith.github.io
   RATE_LIMIT_PER_MINUTE=10
   SIMILARITY_THRESHOLD=0.7
   MAX_CHUNKS=5
   ```

5. **Deploy**
   - Click "Create Web Service"
   - Wait for deployment (~3-5 minutes)
   - Watch logs for successful startup

### Step 4: Verify Deployment

1. **Check Service Status**
   - Render Dashboard should show "Live" (green)
   - Note your service URL: `https://rag-chatbot-api.onrender.com`

2. **Test Health Endpoint**
   ```bash
   curl https://rag-chatbot-api.onrender.com/health
   ```

   Expected response:
   ```json
   {
     "status": "healthy",
     "timestamp": "2025-12-15T...",
     "services": {
       "database": "connected",
       "qdrant": "connected"
     }
   }
   ```

3. **Test Root Endpoint**
   ```bash
   curl https://rag-chatbot-api.onrender.com/
   ```

   Expected response:
   ```json
   {
     "name": "RAG Chatbot API",
     "version": "1.0.0",
     "status": "running",
     "health": "/health",
     "endpoints": {
       "ask": "/api/ask",
       "ask_selected": "/api/ask-selected"
     }
   }
   ```

4. **Check Logs**
   - Render Dashboard → Logs
   - Look for: "🚀 RAG Chatbot API Starting"
   - Verify no error messages

---

## Post-Deployment Tasks

### 1. Index Book Content

After successful deployment, populate the vector database:

**Option A: Local Script (Recommended)**
```bash
# From local machine, pointing to production databases
cd backend
python scripts/reindex_book.py --source ../docs/
```

**Option B: Render Shell**
```bash
# From Render Dashboard → Shell
python scripts/reindex_book.py --source ../docs/
```

This will:
- Read all markdown files from `/docs/**`
- Chunk into 800-token segments (200 overlap)
- Generate embeddings with Gemini
- Store in Qdrant + Postgres (~300-500 chunks expected)

**Verification:**
```bash
# Check Postgres for metadata
psql $DATABASE_URL -c "SELECT COUNT(*) FROM chunks_metadata;"

# Check Qdrant for vectors (via API or console)
# Expected: Same count as Postgres
```

### 2. Update Frontend Configuration

Update your frontend to use the production API:

**In your frontend code:**
```javascript
// Before (development)
const API_URL = 'http://localhost:8000';

// After (production)
const API_URL = 'https://rag-chatbot-api.onrender.com';
```

**Environment-based config (recommended):**
```javascript
const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://rag-chatbot-api.onrender.com'
  : 'http://localhost:8000';
```

### 3. Test End-to-End

1. **Test Chat Endpoint**
   ```bash
   curl -X POST https://rag-chatbot-api.onrender.com/api/ask \
     -H "Content-Type: application/json" \
     -d '{
       "question": "What are the key components of a humanoid robot?"
     }'
   ```

2. **Verify Response Quality**
   - Should return answer with citations
   - Check that sources reference actual book content
   - Verify latency is acceptable (~1-3 seconds)

3. **Test Frontend Integration**
   - Open your frontend: https://asadaligith.github.io
   - Ask a question in the chatbot
   - Verify response displays correctly
   - Check browser console for errors

### 4. Monitor Performance

**Key Metrics to Track:**
- Response time (target: <3s for first request after cold start)
- Error rate (target: <1%)
- API usage (stay within Gemini free tier limits)
- Memory usage (should be <400MB on free tier)

**Monitoring Tools:**
- Render Dashboard: Built-in metrics
- Render Logs: Real-time error tracking
- Gemini Console: API usage tracking

---

## Free Tier Considerations

### Render.com Limits

| Resource | Limit | Impact |
|----------|-------|--------|
| **Monthly Hours** | 750 hours | ~31 days if always running |
| **RAM** | 512 MB | Sufficient for this API |
| **Spin Down** | After 15 min inactivity | Cold start on next request |
| **Cold Start Time** | 30-60 seconds | First request delay |
| **Build Minutes** | 500/month | More than enough |

### Optimization Tips

1. **Keep Service Warm**
   - Set up a cron job to ping `/health` every 10 minutes
   - Use free services like UptimeRobot or Cron-job.org

2. **Reduce Cold Starts**
   ```bash
   # Example cron job (every 10 minutes)
   */10 * * * * curl https://rag-chatbot-api.onrender.com/health
   ```

3. **Monitor Gemini Usage**
   - Free tier: 60 requests/minute
   - Track usage in Google AI Studio
   - Implement rate limiting in production

### Cost Scaling (When Ready)

If you outgrow free tier:
- **Render Starter**: $7/month (no spin-down)
- **Gemini Paid**: $0.000125/request
- **Qdrant**: $25/month (1GB → 4GB)
- **Neon**: $19/month (3GB → 10GB)

---

## Troubleshooting

### Common Issues

#### 1. Build Fails
**Symptoms**: Deployment fails during build phase

**Possible Causes:**
- Python version mismatch
- Missing dependencies
- Invalid requirements.txt

**Solutions:**
```bash
# Check Render logs for specific error
# Verify requirements.txt is valid
pip install -r requirements.txt  # Test locally first

# Clear build cache and retry
# Render Dashboard → Manual Deploy → Clear build cache & deploy
```

#### 2. Service Won't Start
**Symptoms**: Build succeeds but service crashes

**Possible Causes:**
- Missing environment variables
- Database connection failure
- Port binding issues

**Solutions:**
```bash
# Check environment variables are set
# Render Dashboard → Environment → Verify all required vars

# Test database connections
# Render Shell → python -c "from src.config import settings; print(settings.database_url)"

# Verify PORT is used correctly (Render provides $PORT automatically)
```

#### 3. Database Connection Errors
**Symptoms**: "Could not connect to Postgres" or "Qdrant unreachable"

**Solutions:**
```bash
# Verify DATABASE_URL format
postgresql://user:password@host/database?sslmode=require

# Check Neon database is active (free tier pauses after inactivity)
# Neon Dashboard → Wake up database

# Verify Qdrant credentials
# Qdrant Console → API Keys → Regenerate if needed
```

#### 4. CORS Errors
**Symptoms**: Frontend can't access API, browser console shows CORS error

**Solutions:**
```bash
# Add frontend URL to ALLOWED_ORIGINS
# Render Dashboard → Environment → Update ALLOWED_ORIGINS
ALLOWED_ORIGINS=https://asadaligith.github.io,https://your-frontend.com

# Redeploy service after changing env vars
```

#### 5. Cold Start Delays
**Symptoms**: First request takes 30-60 seconds after inactivity

**Solutions:**
```bash
# Set up health check pings (free services)
# UptimeRobot: https://uptimerobot.com
# Cron-job.org: https://cron-job.org

# Or upgrade to Render Starter plan ($7/month) for no spin-down
```

### Debug Commands

**Access Render Shell:**
```bash
# Render Dashboard → Your Service → Shell

# Check environment
env | grep -E 'GEMINI|QDRANT|DATABASE'

# Test database connection
python -c "
from src.config import settings
from src.services.database import check_connection
print(check_connection())
"

# Check Qdrant collection
python -c "
from src.services.retrieval import get_collection_info
print(get_collection_info())
"

# View recent logs
tail -n 100 /var/log/render.log
```

---

## Maintenance

### Regular Tasks

**Weekly:**
- [ ] Check error logs in Render Dashboard
- [ ] Monitor Gemini API usage (stay within free tier)
- [ ] Verify frontend is connecting successfully

**Monthly:**
- [ ] Review Render usage (ensure <750 hours)
- [ ] Update dependencies if needed (`pip list --outdated`)
- [ ] Backup Postgres data from Neon

**As Needed:**
- [ ] Re-index book content after docs updates
- [ ] Update environment variables for config changes
- [ ] Clear Render build cache if deployment issues

### Updating Code

1. **Make changes locally**
   ```bash
   cd backend
   # Make your code changes
   pytest  # Run tests
   ```

2. **Commit and push**
   ```bash
   git add .
   git commit -m "Update: description of changes"
   git push origin master
   ```

3. **Auto-deploy**
   - Render automatically deploys on git push (configured in `render.yaml`)
   - Watch deployment in Render Dashboard
   - Verify after deployment completes

### Rolling Back

**If deployment fails:**
```bash
# Render Dashboard → Your Service → Manual Deploy
# Select previous successful commit
# Click "Deploy"
```

---

## Security Best Practices

### Credentials Management

- [x] All API keys stored as Render environment variables (not in code)
- [x] `.env` in `.gitignore` (never commit secrets)
- [ ] Rotate API keys periodically (quarterly recommended)
- [ ] Use Render's secret management (variables are encrypted)

### API Security

- [x] CORS configured (only allow specific origins)
- [x] Rate limiting enabled (10 requests/minute per session)
- [ ] Monitor for unusual traffic patterns
- [ ] Set up alerts for failed requests

### Database Security

- [x] Neon enforces SSL (`sslmode=require`)
- [x] Qdrant uses API key authentication
- [ ] Regularly backup Postgres data
- [ ] Monitor database access logs

---

## Success Criteria

Deployment is successful when:

- [ ] Service shows "Live" status in Render Dashboard
- [ ] Health endpoint returns 200 OK
- [ ] Root endpoint returns API information
- [ ] Database connections verified (Postgres + Qdrant)
- [ ] Book content indexed (verified count in both databases)
- [ ] Frontend can successfully query the API
- [ ] Response times are acceptable (<3s)
- [ ] No errors in Render logs during normal operation
- [ ] CORS works correctly from frontend domain
- [ ] Rate limiting functions as expected

---

## Quick Reference

### Important URLs

| Service | URL |
|---------|-----|
| **Production API** | https://rag-chatbot-api.onrender.com |
| **API Health** | https://rag-chatbot-api.onrender.com/health |
| **API Docs** | https://rag-chatbot-api.onrender.com/docs (dev only) |
| **Render Dashboard** | https://dashboard.render.com |
| **Neon Console** | https://console.neon.tech |
| **Qdrant Console** | https://cloud.qdrant.io |
| **Gemini Console** | https://aistudio.google.com |

### Environment Variables Reference

```bash
# Core Services
GEMINI_API_KEY=<your-key>
QDRANT_URL=<your-url>
QDRANT_API_KEY=<your-key>
DATABASE_URL=<your-connection-string>

# Application Config
ENVIRONMENT=production
ALLOWED_ORIGINS=https://asadaligith.github.io
RATE_LIMIT_PER_MINUTE=10
SIMILARITY_THRESHOLD=0.7
MAX_CHUNKS=5
```

### Support Resources

- **Render Docs**: https://render.com/docs
- **Gemini Docs**: https://ai.google.dev/docs
- **Qdrant Docs**: https://qdrant.tech/documentation
- **Neon Docs**: https://neon.tech/docs
- **FastAPI Docs**: https://fastapi.tiangolo.com

---

## Next Steps After Deployment

1. [ ] Complete book content indexing
2. [ ] Test all API endpoints thoroughly
3. [ ] Integrate with frontend
4. [ ] Set up monitoring/uptime checks
5. [ ] Document API for frontend developers
6. [ ] Plan for scaling if usage grows

---

**Deployment Date**: _____________
**Deployed By**: _____________
**Production URL**: https://rag-chatbot-api.onrender.com
**Status**: ⏳ Ready to Deploy

---

*For issues or questions, refer to backend/README.md or create an issue in the GitHub repository.*
