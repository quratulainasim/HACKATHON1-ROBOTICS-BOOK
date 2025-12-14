#!/usr/bin/env python3
"""
Script to create RAG collections in Qdrant from the docs/ folder content
This script indexes all markdown files from the docs/ folder into Qdrant
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from app.services.qdrant_service import QdrantService
from app.config.settings import settings

def main():
    print("Starting RAG creation process...")
    print(f"Reading content from: {settings.docs_path}")

    try:
        # Create Qdrant service instance
        qdrant_service = QdrantService()

        # Initialize and populate the collection
        print("Initializing Qdrant collection...")
        qdrant_service.initialize_collection()

        print("Indexing docs content to Qdrant...")
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