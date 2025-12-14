import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

print("Environment variables:")
print(f"GEMINI_API_KEY: {os.getenv('GEMINI_API_KEY')}")
print(f"QDRANT_URL: {os.getenv('QDRANT_URL')}")
print(f"QDRANT_API_KEY: {os.getenv('QDRANT_API_KEY')}")

# Now import settings
from app.config.settings import Settings
settings = Settings()

print("\nSettings object:")
print(f"settings.gemini_api_key: {settings.gemini_api_key}")
print(f"settings.qdrant_url: {settings.qdrant_url}")