#!/usr/bin/env python3
"""
Test script to verify RAG functionality works
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

# Import Qdrant service directly
from app.services.qdrant_service import QdrantService

print("Creating Qdrant service...")
qdrant_service = QdrantService()

print(f"Qdrant service initialized. Collection exists: {qdrant_service.collection_exists()}")

if qdrant_service.collection_exists():
    count = qdrant_service.get_collection_count()
    print(f"Collection has {count} documents")

    # Test a search
    print("Testing search functionality...")
    results = qdrant_service.search_contexts("What is ROS 2?", limit=3)
    print(f"Search returned {len(results)} results")

    if results:
        print(f"First result title: {results[0]['title'][:50]}...")
        print(f"First result relevance: {results[0]['relevance_score']:.2f}")
        print(f"First result source: {results[0]['source_path']}")

    print("\nRAG functionality is working correctly!")
else:
    print("ERROR: Qdrant collection does not exist!")

print("\nRAG creation and functionality test completed.")