try:
    from pydantic_settings import BaseSettings
    from pydantic import Field
except ImportError:
    from pydantic import BaseSettings, Field
from typing import Optional
from pathlib import Path

class Settings(BaseSettings):
    """
    Application settings loaded from environment variables
    """
    # API Configuration
    api_host: str = Field(default="0.0.0.0")
    api_port: int = Field(default=8000)

    # OpenAI/Gemini Configuration
    gemini_api_key: Optional[str] = Field(default=None)
    gemini_model: str = Field(default="gemini-pro")  # Default model

    # Database Configuration
    database_url: Optional[str] = Field(default=None)
    neon_db_url: Optional[str] = Field(default=None)

    # Qdrant Configuration (optional)
    qdrant_url: Optional[str] = Field(default=None)

    # Content Configuration
    docs_path: str = Field(default="E:/my-AI-Robotics-Book/docs")
    max_context_tokens: int = Field(default=3000)  # Maximum tokens for context
    max_question_length: int = Field(default=500)  # Maximum length of questions
    max_response_length: int = Field(default=10000)  # Maximum length of responses

    # Performance Configuration
    max_concurrent_users: int = Field(default=100)
    response_timeout: int = Field(default=30)  # seconds
    cache_ttl: int = Field(default=3600)  # seconds

    # Security Configuration
    rate_limit_requests: int = Field(default=100)  # per minute
    rate_limit_window: int = Field(default=60)  # seconds

    # AI Behavior Configuration
    educational_tone: bool = Field(default=True)
    hallucination_prevention: bool = Field(default=True)
    external_knowledge_prohibited: bool = Field(default=True)

    model_config = {
        "env_file": ".env",
        "env_file_encoding": "utf-8",
        "case_sensitive": True,
        "extra": "ignore"  # Ignore extra fields that might be in the env file
    }


# Create a global settings instance
from dotenv import load_dotenv
import os
from pathlib import Path

# Load environment variables from .env file
env_path = Path(__file__).parent.parent / ".env"
if env_path.exists():
    load_dotenv(env_path)

settings = Settings()