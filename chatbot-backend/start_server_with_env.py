#!/usr/bin/env python3
"""
Script to start the server with proper environment loading
"""

import os
import sys
from pathlib import Path

# Add the current directory to the path
chatbot_dir = Path(__file__).parent
sys.path.insert(0, str(chatbot_dir))

# Load environment variables first
from dotenv import load_dotenv
env_file_path = chatbot_dir / ".env"
if env_file_path.exists():
    print(f"Loading environment from: {env_file_path}")
    load_dotenv(env_file_path)
else:
    print("Warning: .env file not found")

# Now import and check settings
from app.config.settings import settings
print(f"Settings loaded:")
print(f"  GEMINI_API_KEY available: {'Yes' if settings.gemini_api_key else 'No'}")
print(f"  QDRANT_URL available: {'Yes' if settings.qdrant_url else 'No'}")
print(f"  API Port: {settings.api_port}")

# Import and run the app
import uvicorn
from app.main import app

# Use a different port to avoid conflicts
port = 8005
print(f"Starting server on port {port}")

if __name__ == "__main__":
    uvicorn.run(app, host='0.0.0.0', port=port)