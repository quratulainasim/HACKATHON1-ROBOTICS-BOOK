import os
from pathlib import Path

# Change to the correct directory
chatbot_dir = Path("E:/my-AI-Robotics-Book/chatbot-backend")
os.chdir(chatbot_dir)

print(f"Current working directory: {os.getcwd()}")
print(f"Looking for .env file at: {chatbot_dir / '.env'}")
print(f".env file exists: {(chatbot_dir / '.env').exists()}")

# Read the .env file content directly to verify it
env_file_path = chatbot_dir / ".env"
if env_file_path.exists():
    with open(env_file_path, 'r') as f:
        content = f.read()
        print(f".env file content preview:\n{content[:500]}...")

# Load environment variables manually
from dotenv import load_dotenv
print("\nLoading .env file...")
load_dotenv(env_file_path)
print(f"Environment variable QDRANT_URL after loading: {os.getenv('QDRANT_URL')}")
print(f"Environment variable GEMINI_API_KEY after loading (first 10): {os.getenv('GEMINI_API_KEY')[:10] if os.getenv('GEMINI_API_KEY') else None}")

# Now import and create settings
print("\nImporting Settings class...")
from app.config.settings import Settings

print("Creating Settings instance...")
settings = Settings()

print(f"Settings qdrant_url: {settings.qdrant_url}")
print(f"Settings gemini_api_key (first 10): {settings.gemini_api_key[:10] if settings.gemini_api_key else None}")

# Also print the actual settings instance values
print(f"Settings object __dict__: {settings.__dict__}")