#!/usr/bin/env python3
"""
Script to verify that RAG collections were created successfully in Qdrant
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from app.services.qdrant_service import QdrantService

def main():
    print("Verifying RAG collections in Qdrant...")

    try:
        # Create Qdrant service instance
        qdrant_service = QdrantService()

        # Check if collection exists
        if qdrant_service.collection_exists():
            print("[OK] Qdrant collection 'book_content' exists")
        else:
            print("[ERROR] Qdrant collection 'book_content' does not exist")
            return False

        # Get collection count
        count = qdrant_service.get_collection_count()
        print(f"[OK] Total documents in collection: {count}")

        if count > 0:
            print("[OK] Qdrant collections have been successfully created and populated!")

            # Test a sample search to ensure functionality
            print("\nTesting sample search...")
            sample_results = qdrant_service.search_contexts("What is ROS 2?", limit=3)
            print(f"[OK] Sample search returned {len(sample_results)} results")

            if sample_results:
                print("[OK] Sample search was successful - RAG system is ready!")
                print(f"  First result: {sample_results[0]['title'][:50]}...")
            else:
                print("[WARN] Sample search returned no results, but collections exist")

            return True
        else:
            print("[ERROR] Qdrant collection exists but is empty")
            return False

    except Exception as e:
        print(f"[ERROR] Error during verification: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = main()
    if success:
        print("\n[OK] RAG verification completed successfully!")
        sys.exit(0)
    else:
        print("\n[ERROR] RAG verification failed!")
        sys.exit(1)