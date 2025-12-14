import os
import sys
from pathlib import Path

# Add the chatbot-backend directory to the Python path
chatbot_dir = Path("E:/my-AI-Robotics-Book/chatbot-backend")
os.chdir(chatbot_dir)
sys.path.insert(0, str(chatbot_dir))

# Change to the chatbot directory so that relative paths work correctly
os.chdir(chatbot_dir)

print(f"Current working directory: {os.getcwd()}")
print(f".env file exists in current dir: {(chatbot_dir / '.env').exists()}")

# Now try to import and use the settings
from app.config.settings import settings

print(f"Settings qdrant_url: {settings.qdrant_url}")
print(f"Settings gemini_api_key (first 10 chars): {settings.gemini_api_key[:10] if settings.gemini_api_key else 'None'}")