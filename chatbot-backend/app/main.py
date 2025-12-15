from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import chat, health
from app.config.settings import Settings

settings = Settings()

app = FastAPI(title="Context-Based RAG Chatbot API", version="1.0.0")

# SHORT-TERM HACKATHON FIX: Allow ALL origins
# This will stop the 400 error on OPTIONS requests from your deployed frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],           # Allows any domain (including your Netlify/Vercel Docusaurus site)
    allow_credentials=True,
    allow_methods=["*"],           # Allows GET, POST, OPTIONS, etc.
    allow_headers=["*"],           # Allows Content-Type and any other headers
)

# Include your API routers
app.include_router(chat.router, prefix="/chat", tags=["chat"])
app.include_router(health.router, prefix="/health", tags=["health"])

# For local development
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="0.0.0.0", port=8001, reload=True)
