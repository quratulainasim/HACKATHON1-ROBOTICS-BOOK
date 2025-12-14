import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

# Force reload of the settings module
import importlib
if 'app.config.settings' in sys.modules:
    importlib.reload(sys.modules['app.config.settings'])

from app.config.settings import settings

print("Settings object after reload:")
print(f"settings.gemini_api_key: {settings.gemini_api_key}")
print(f"settings.qdrant_url: {settings.qdrant_url}")