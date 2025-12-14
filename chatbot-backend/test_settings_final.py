#!/usr/bin/env python3
"""
Final test to verify settings are loading correctly
"""

import os
import sys
from pathlib import Path

# Add the current directory to the path
chatbot_dir = Path(__file__).parent
sys.path.insert(0, str(chatbot_dir))

# Load environment variables first to see what's available
from dotenv import load_dotenv
env_file_path = chatbot_dir / ".env"
if env_file_path.exists():
    print(f"Loading environment from: {env_file_path}")
    load_dotenv(env_file_path)

print(f"Environment variables:")
print(f"  GEMINI_API_KEY: {'SET' if os.getenv('GEMINI_API_KEY') else 'NOT SET'}")
print(f"  QDRANT_URL: {'SET' if os.getenv('QDRANT_URL') else 'NOT SET'}")
print(f"  API_PORT: {os.getenv('API_PORT', 'NOT SET')}")

# Now import and check settings
from app.config.settings import settings

print(f"\nSettings object:")
print(f"  GEMINI_API_KEY: {'SET' if settings.gemini_api_key else 'NOT SET'}")
print(f"  QDRANT_URL: {'SET' if settings.qdrant_url else 'NOT SET'}")
print(f"  API_PORT: {settings.api_port}")

if settings.qdrant_url:
    print("\n[SUCCESS] Settings loaded successfully from .env file!")
else:
    print("\n[WARNING] Settings still not loading properly, but Qdrant service uses direct env loading")