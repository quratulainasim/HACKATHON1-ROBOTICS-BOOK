import os
from pathlib import Path
from dotenv import load_dotenv

# Check if .env file exists
env_file_path = Path("E:/my-AI-Robotics-Book/chatbot-backend/.env")
print(f".env file exists: {env_file_path.exists()}")

if env_file_path.exists():
    print("Loading .env file...")
    load_dotenv(env_file_path)
    print(f"After loading - QDRANT_URL: {os.getenv('QDRANT_URL')}")
    print(f"After loading - GEMINI_API_KEY: {os.getenv('GEMINI_API_KEY')[:10]}..." if os.getenv('GEMINI_API_KEY') else "None")
else:
    print("File not found!")

# Now test Pydantic settings
try:
    from pydantic_settings import BaseSettings
    from pydantic import Field
except ImportError:
    from pydantic import BaseSettings, Field

from typing import Optional

class TestSettings(BaseSettings):
    qdrant_url: Optional[str] = Field(default=None)
    gemini_api_key: Optional[str] = Field(default=None)

    class Config:
        env_file = ".env"
        env_file_encoding = 'utf-8'

test_settings = TestSettings()
print(f"\nPydantic TestSettings - qdrant_url: {test_settings.qdrant_url}")
print(f"Pydantic TestSettings - gemini_api_key: {test_settings.gemini_api_key[:10] if test_settings.gemini_api_key else 'None'}...")