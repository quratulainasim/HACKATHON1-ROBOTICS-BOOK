# Test which BaseSettings we're getting
try:
    from pydantic_settings import BaseSettings
    print("Using pydantic_settings.BaseSettings")
    print(f"BaseSettings module: {BaseSettings.__module__}")
except ImportError:
    print("pydantic_settings not available, using fallback")
    try:
        from pydantic import BaseSettings
        print(f"Using pydantic.BaseSettings")
        print(f"BaseSettings module: {BaseSettings.__module__}")
    except ImportError:
        print("Neither pydantic nor pydantic_settings available")

# Test if _env_file parameter is supported
from pydantic import Field
from typing import Optional
from pathlib import Path

class TestSettings(BaseSettings):
    qdrant_url: Optional[str] = Field(default=None)

    model_config = {
        "case_sensitive": True,
        "extra": "ignore"
    }

try:
    # Try to create with _env_file parameter
    test_settings = TestSettings(_env_file=Path("E:/my-AI-Robotics-Book/chatbot-backend/.env"))
    print(f"Successfully created with _env_file. Qdrant URL: {test_settings.qdrant_url}")
except TypeError as e:
    print(f"Error with _env_file parameter: {e}")

    # Try loading env file manually and then creating settings
    import os
    from dotenv import load_dotenv
    load_dotenv("E:/my-AI-Robotics-Book/chatbot-backend/.env")
    test_settings = TestSettings()
    print(f"Created after manual load. Qdrant URL: {test_settings.qdrant_url}")