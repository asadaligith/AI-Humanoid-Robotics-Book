# RAG Chatbot Deployment Guide

## Overview

This guide covers deploying the RAG chatbot to production using Docker, cloud services, and CI/CD automation.

## Deployment Options

### Option 1: Cloud Platform (Recommended)

- **API**: Railway, Render, or Fly.io
- **Database**: Neon PostgreSQL (serverless)
- **Vector DB**: Qdrant Cloud
- **CDN**: Cloudflare (for static site)

### Option 2: Self-Hosted

- **Server**: AWS EC2, DigitalOcean, or VPS
- **Database**: Self-hosted PostgreSQL
- **Vector DB**: Self-hosted Qdrant
- **Reverse Proxy**: Nginx

### Option 3: Serverless

- **Functions**: AWS Lambda or Cloudflare Workers
- **API Gateway**: AWS API Gateway
- **Database**: Neon PostgreSQL
- **Vector DB**: Qdrant Cloud

## Production Deployment (Railway)

### Step 1: Prepare Application

Create `railway.json`:

```json
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS",
    "buildCommand": "pip install -r requirements.txt"
  },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 10
  }
}
```

### Step 2: Environment Variables

Set in Railway dashboard:

```bash
ANTHROPIC_API_KEY=sk-ant-...
OPENAI_API_KEY=sk-...
QDRANT_URL=https://xxx.qdrant.io:6333
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...
APP_ENV=production
LOG_LEVEL=INFO
ALLOWED_ORIGINS=https://yourdomain.com
```

### Step 3: Deploy

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Initialize project
railway init

# Deploy
railway up

# Add custom domain
railway domain
```

## Docker Production Deployment

### Multi-Stage Dockerfile

```dockerfile
# Build stage
FROM python:3.11-slim as builder

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    postgresql-client \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip install --user --no-cache-dir -r requirements.txt

# Runtime stage
FROM python:3.11-slim

WORKDIR /app

# Copy Python packages from builder
COPY --from=builder /root/.local /root/.local

# Copy application
COPY app/ ./app/

# Create non-root user
RUN useradd -m -u 1000 appuser && \
    chown -R appuser:appuser /app

USER appuser

# Update PATH
ENV PATH=/root/.local/bin:$PATH

# Expose port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
  CMD python -c "import requests; requests.get('http://localhost:8000/health')"

# Run application
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "4"]
```

### Docker Compose for Production

```yaml
version: '3.8'

services:
  chatbot-api:
    build:
      context: .
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
    environment:
      - ANTHROPIC_API_KEY=${ANTHROPIC_API_KEY}
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - DATABASE_URL=${DATABASE_URL}
      - REDIS_URL=redis://redis:6379/0
      - APP_ENV=production
    depends_on:
      - redis
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3

  redis:
    image: redis:7-alpine
    volumes:
      - redis_data:/data
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "redis-cli", "ping"]
      interval: 30s
      timeout: 10s
      retries: 3

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf:ro
      - ./ssl:/etc/nginx/ssl:ro
    depends_on:
      - chatbot-api
    restart: unless-stopped

volumes:
  redis_data:
```

### Nginx Configuration

```nginx
# nginx.conf
upstream chatbot_api {
    server chatbot-api:8000;
}

server {
    listen 80;
    server_name api.yourdomain.com;

    # Redirect to HTTPS
    return 301 https://$server_name$request_uri;
}

server {
    listen 443 ssl http2;
    server_name api.yourdomain.com;

    # SSL certificates
    ssl_certificate /etc/nginx/ssl/fullchain.pem;
    ssl_certificate_key /etc/nginx/ssl/privkey.pem;

    # SSL configuration
    ssl_protocols TLSv1.2 TLSv1.3;
    ssl_ciphers HIGH:!aNULL:!MD5;
    ssl_prefer_server_ciphers on;

    # Security headers
    add_header X-Frame-Options "SAMEORIGIN" always;
    add_header X-Content-Type-Options "nosniff" always;
    add_header X-XSS-Protection "1; mode=block" always;
    add_header Strict-Transport-Security "max-age=31536000; includeSubDomains" always;

    # CORS headers
    add_header Access-Control-Allow-Origin "https://yourdomain.com" always;
    add_header Access-Control-Allow-Methods "GET, POST, OPTIONS" always;
    add_header Access-Control-Allow-Headers "Content-Type, Authorization" always;

    # Rate limiting
    limit_req_zone $binary_remote_addr zone=api_limit:10m rate=10r/m;

    location / {
        limit_req zone=api_limit burst=20 nodelay;

        proxy_pass http://chatbot_api;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        # Timeouts
        proxy_connect_timeout 60s;
        proxy_send_timeout 60s;
        proxy_read_timeout 60s;
    }

    location /health {
        proxy_pass http://chatbot_api/health;
        access_log off;
    }
}
```

## CI/CD with GitHub Actions

```yaml
# .github/workflows/deploy.yml
name: Deploy to Production

on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  deploy:
    runs-on: ubuntu-latest

    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: |
          cd chatbot
          pip install -r requirements.txt
          pip install pytest pytest-asyncio

      - name: Run tests
        run: |
          cd chatbot
          pytest tests/

      - name: Build Docker image
        run: |
          cd chatbot
          docker build -t chatbot-api:latest .

      - name: Login to Docker Hub
        uses: docker/login-action@v2
        with:
          username: ${{ secrets.DOCKER_USERNAME }}
          password: ${{ secrets.DOCKER_PASSWORD }}

      - name: Push to Docker Hub
        run: |
          docker tag chatbot-api:latest ${{ secrets.DOCKER_USERNAME }}/chatbot-api:latest
          docker push ${{ secrets.DOCKER_USERNAME }}/chatbot-api:latest

      - name: Deploy to Railway
        run: |
          npm install -g @railway/cli
          railway up --service chatbot-api
        env:
          RAILWAY_TOKEN: ${{ secrets.RAILWAY_TOKEN }}

      - name: Run database migrations
        run: |
          python chatbot/scripts/migrate_database.py
        env:
          DATABASE_URL: ${{ secrets.DATABASE_URL }}

      - name: Ingest latest documentation
        run: |
          python chatbot/scripts/ingest_docs.py --docs-path docs
        env:
          QDRANT_URL: ${{ secrets.QDRANT_URL }}
          QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
          OPENAI_API_KEY: ${{ secrets.OPENAI_API_KEY }}
          DATABASE_URL: ${{ secrets.DATABASE_URL }}
```

## Monitoring & Logging

### Application Logging

```python
# app/logging_config.py
import logging
from logging.handlers import RotatingFileHandler
import sys

def setup_logging(app_env="production"):
    """Configure application logging."""

    # Create logger
    logger = logging.getLogger("chatbot")
    logger.setLevel(logging.INFO if app_env == "production" else logging.DEBUG)

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)

    # File handler (rotating)
    file_handler = RotatingFileHandler(
        "logs/chatbot.log",
        maxBytes=10 * 1024 * 1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(logging.DEBUG)

    # Formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)
    file_handler.setFormatter(formatter)

    # Add handlers
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    return logger
```

### Sentry Integration

```python
import sentry_sdk
from sentry_sdk.integrations.fastapi import FastApiIntegration

sentry_sdk.init(
    dsn=os.getenv("SENTRY_DSN"),
    integrations=[FastApiIntegration()],
    traces_sample_rate=0.1,
    environment=os.getenv("APP_ENV", "production")
)
```

### Prometheus Metrics

```python
from prometheus_client import Counter, Histogram, Gauge, generate_latest
from fastapi import Response

# Metrics
request_count = Counter('chatbot_requests_total', 'Total requests')
request_duration = Histogram('chatbot_request_duration_seconds', 'Request duration')
active_sessions = Gauge('chatbot_active_sessions', 'Active sessions')

@app.get("/metrics")
async def metrics():
    """Prometheus metrics endpoint."""
    return Response(content=generate_latest(), media_type="text/plain")
```

## Scaling Strategies

### Horizontal Scaling

```yaml
# docker-compose.scale.yml
services:
  chatbot-api:
    deploy:
      replicas: 4
      resources:
        limits:
          cpus: '1'
          memory: 2G
        reservations:
          cpus: '0.5'
          memory: 1G
```

### Load Balancing

```nginx
upstream chatbot_api {
    least_conn;
    server chatbot-api-1:8000 weight=1;
    server chatbot-api-2:8000 weight=1;
    server chatbot-api-3:8000 weight=1;
    server chatbot-api-4:8000 weight=1;
}
```

### Caching Layer

```python
import redis
from functools import wraps
import json
import hashlib

redis_client = redis.Redis.from_url(os.getenv("REDIS_URL"))

def cache_query(ttl=3600):
    """Cache query results in Redis."""
    def decorator(func):
        @wraps(func)
        async def wrapper(query: str, *args, **kwargs):
            # Create cache key
            cache_key = f"query:{hashlib.sha256(query.encode()).hexdigest()}"

            # Check cache
            cached = redis_client.get(cache_key)
            if cached:
                return json.loads(cached)

            # Execute query
            result = await func(query, *args, **kwargs)

            # Cache result
            redis_client.setex(cache_key, ttl, json.dumps(result))

            return result
        return wrapper
    return decorator
```

## Security Hardening

### API Key Rotation

```python
# Rotate API keys periodically
import boto3

def rotate_api_keys():
    """Rotate API keys stored in AWS Secrets Manager."""
    secrets = boto3.client('secretsmanager')

    # Generate new keys
    # Update in Secrets Manager
    # Update in application config
    # Revoke old keys
```

### Input Sanitization

```python
from fastapi import HTTPException
import re

def sanitize_query(query: str) -> str:
    """Sanitize user query."""
    # Remove control characters
    query = re.sub(r'[\x00-\x1f\x7f-\x9f]', '', query)

    # Limit length
    if len(query) > 1000:
        raise HTTPException(status_code=400, detail="Query too long")

    # Check for injection attempts
    dangerous_patterns = [
        r'<script',
        r'javascript:',
        r'onerror=',
        r'onclick='
    ]

    for pattern in dangerous_patterns:
        if re.search(pattern, query, re.IGNORECASE):
            raise HTTPException(status_code=400, detail="Invalid query")

    return query.strip()
```

## Backup & Recovery

### Database Backups

```bash
# Automated daily backups
#!/bin/bash
# backup.sh

DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR="/backups"
DATABASE_URL="postgresql://..."

# Backup PostgreSQL
pg_dump "$DATABASE_URL" | gzip > "$BACKUP_DIR/db_$DATE.sql.gz"

# Upload to S3
aws s3 cp "$BACKUP_DIR/db_$DATE.sql.gz" s3://my-backups/chatbot/

# Keep only last 30 days
find "$BACKUP_DIR" -name "db_*.sql.gz" -mtime +30 -delete
```

### Vector Database Backup

```python
# Export Qdrant snapshot
from qdrant_client import QdrantClient

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Create snapshot
client.create_snapshot(collection_name="course_docs")

# Download snapshot
# Upload to cloud storage
```

## Cost Optimization

1. **Use Connection Pooling**: Reduce database connections
2. **Cache Frequent Queries**: Redis for hot data
3. **Batch Embeddings**: Group API calls
4. **Right-Size Instances**: Monitor and adjust resources
5. **Use Spot Instances**: For non-critical workloads

## Troubleshooting

### High Latency

```bash
# Check database query performance
EXPLAIN ANALYZE SELECT * FROM documents WHERE module = '01';

# Add missing indexes
CREATE INDEX idx_documents_module ON documents(module);

# Monitor API response times
tail -f logs/chatbot.log | grep "response_time"
```

### Memory Leaks

```python
# Use memory profiler
from memory_profiler import profile

@profile
def process_query(query):
    # Your code here
    pass
```

### Database Connection Issues

```python
# Implement connection retry
from tenacity import retry, stop_after_attempt, wait_fixed

@retry(stop=stop_after_attempt(3), wait=wait_fixed(2))
def get_db_connection():
    return psycopg2.connect(DATABASE_URL)
```

## References

- [Railway Documentation](https://docs.railway.app/)
- [Docker Best Practices](https://docs.docker.com/develop/dev-best-practices/)
- [Nginx Configuration](https://www.nginx.com/resources/wiki/)
- [FastAPI Deployment](https://fastapi.tiangolo.com/deployment/)
