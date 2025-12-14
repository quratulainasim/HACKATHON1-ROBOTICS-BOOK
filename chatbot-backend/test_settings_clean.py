import subprocess
import sys

# Run a completely separate Python process to test settings
result = subprocess.run([
    sys.executable, '-c', '''
import os
from pathlib import Path

# Change to the correct directory
chatbot_dir = Path("E:/my-AI-Robotics-Book/chatbot-backend")
os.chdir(chatbot_dir)

print(f"Working in: {os.getcwd()}")

# Import and test settings
from app.config.settings import settings
print(f"Qdrant URL: {settings.qdrant_url}")
print(f"Gemini API Key (first 10): {settings.gemini_api_key[:10] if settings.gemini_api_key else None}")
'''
], capture_output=True, text=True)

print("STDOUT:")
print(result.stdout)
print("STDERR:")
print(result.stderr)
print(f"Return code: {result.returncode}")