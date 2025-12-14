#!/usr/bin/env python3
"""
Script to create RAG collections in cloud Qdrant from the docs/ folder content
This script indexes all markdown files from the docs/ folder into cloud Qdrant
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

# Load environment variables BEFORE importing settings
from dotenv import load_dotenv
load_dotenv()

# Now import settings (after environment is loaded)
from app.config.settings import Settings
settings = Settings()

from app.services.qdrant_service import QdrantService

def main():
    import os
    qdrant_url = os.getenv("QDRANT_URL")
    print("Starting RAG creation process for cloud Qdrant...")
    print(f"Reading content from: {settings.docs_path}")
    print(f"Using Qdrant URL: {qdrant_url}")

    try:
        # Create Qdrant service instance (will use cloud Qdrant due to env var)
        qdrant_service = QdrantService()

        # Initialize and populate the collection
        print("Initializing Qdrant collection...")
        qdrant_service.initialize_collection()

        print("Indexing docs content to cloud Qdrant...")
        qdrant_service.index_docs_content()

        print("RAG creation process completed successfully!")

        # Verify the collection
        count = qdrant_service.get_collection_count()
        print(f"Total documents indexed: {count}")

    except Exception as e:
        print(f"Error during RAG creation: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()