from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import chat, health
from app.config.settings import Settings

settings = Settings()
app = FastAPI(title="Context-Based RAG Chatbot API", version="1.0.0")

# Add CORS middleware to allow requests from frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000", 
        "https://hackathon1-robotics-book-production-a9bd.up.railway.app",
        ],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, prefix="/chat", tags=["chat"])
app.include_router(health.router, prefix="/health", tags=["health"])

if __name__ == "__main__":
    import uvicorn
<<<<<<< HEAD
    uvicorn.run("app.main:app", host="0.0.0.0", port=8001, reload=True)
=======
    uvicorn.run("app.main:app", host="0.0.0.0", port=8001, reload=True)
>>>>>>> a63f5c1 (Fix CORS for localhost and Railway frontend)
