from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
from app.config.settings import settings
from app.models.context import Context
from app.utils.content_reader import ContentReader
from app.utils.text_processor import extract_headings, calculate_relevance_score, chunk_text
import hashlib
import logging

logger = logging.getLogger(__name__)

class QdrantService:
    """
    Service to handle Qdrant vector database operations
    Creates and manages collections for the RAG system
    """

    def __init__(self):
        # Initialize Qdrant client
        import os
        from dotenv import load_dotenv
        # Try to load .env file to ensure environment variables are available
        env_file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), ".env")
        if os.path.exists(env_file_path):
            load_dotenv(env_file_path)

        # Check if we have Qdrant URL in environment variables directly
        qdrant_url = os.getenv("QDRANT_URL")

        if qdrant_url:
            # Use cloud Qdrant instance with API key
            qdrant_api_key = os.getenv("QDRANT_API_KEY")
            if qdrant_api_key:
                self.client = QdrantClient(
                    url=qdrant_url,
                    api_key=qdrant_api_key
                )
            else:
                self.client = QdrantClient(url=qdrant_url)
        else:
            # Use persistent local storage for local development
            # Create a local directory for Qdrant data
            qdrant_data_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "qdrant_data")
            os.makedirs(qdrant_data_path, exist_ok=True)
            self.client = QdrantClient(path=qdrant_data_path)  # Local persistent storage

        self.collection_name = "book_content"

    def initialize_collection(self):
        """
        Initialize the Qdrant collection for book content
        Creates the collection if it doesn't exist
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with vector configuration
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=128,  # Size must match the vector dimensions we're inserting (128 from _create_vector_from_hash)
                        distance=models.Distance.COSINE
                    ),
                    # Add payload schema for filtering
                    optimizers_config=models.OptimizersConfigDiff(
                        memmap_threshold=20000,
                        indexing_threshold=20000
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {str(e)}")
            raise

    def index_docs_content(self):
        """
        Index all content from docs/ folder into Qdrant
        This is the RAG creation process
        """
        try:
            logger.info("Starting RAG creation process - indexing docs content to Qdrant")

            # Initialize the collection if needed
            self.initialize_collection()

            # Read all content from docs/
            content_reader = ContentReader(settings.docs_path)
            all_content = content_reader.read_all_content()

            logger.info(f"Found {len(all_content)} documents to index")

            # Prepare points for Qdrant
            points = []
            for i, content_block in enumerate(all_content):
                # Create a simple vector representation (we'll use a hash-based approach for deterministic matching)
                content_hash = hashlib.md5(content_block['content'].encode()).hexdigest()
                vector = self._create_vector_from_hash(content_hash)

                # Create point for Qdrant
                point = models.PointStruct(
                    id=i,
                    vector=vector,
                    payload={
                        "path": content_block['path'],
                        "title": content_block['title'],
                        "content": content_block['content'],
                        "last_modified": content_block['last_modified'],
                        "headings": extract_headings(content_block['content'])
                    }
                )
                points.append(point)

                # Batch insert every 100 points to avoid memory issues
                if len(points) >= 100:
                    self.client.upsert(
                        collection_name=self.collection_name,
                        points=points
                    )
                    logger.info(f"Indexed {len(points)} documents so far...")
                    points = []

            # Insert remaining points
            if points:
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )
                logger.info(f"Indexed final batch of {len(points)} documents")

            logger.info(f"Successfully indexed {len(all_content)} documents to Qdrant collection {self.collection_name}")

            # Verify the collection has been created with content
            count = self.client.count(collection_name=self.collection_name)
            logger.info(f"Total points in collection: {count.count}")

        except Exception as e:
            logger.error(f"Error indexing docs content: {str(e)}")
            raise

    def search_contexts(self, query: str, limit: int = 10) -> List[Dict]:
        """
        Search for relevant contexts based on the query
        """
        try:
            # Create vector from query for similarity search
            query_hash = hashlib.md5(query.encode()).hexdigest()
            query_vector = self._create_vector_from_hash(query_hash)

            # Search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                with_payload=True
            )

            # Process results
            contexts = []
            for result in search_results:
                payload = result.payload
                # Calculate relevance based on content similarity
                relevance_score = calculate_relevance_score(payload['content'], query)

                context_data = {
                    'content': payload['content'],
                    'source_path': payload['path'],
                    'title': payload['title'],
                    'relevance_score': relevance_score,
                    'metadata': {
                        'headings': payload.get('headings', []),
                        'last_modified': payload.get('last_modified', '')
                    }
                }
                contexts.append(context_data)

            # Sort by relevance score
            contexts.sort(key=lambda x: x['relevance_score'], reverse=True)

            return contexts

        except Exception as e:
            logger.error(f"Error searching contexts: {str(e)}")
            return []

    def _create_vector_from_hash(self, hash_str: str) -> List[float]:
        """
        Create a simple vector representation from a hash string
        This is a basic approach for deterministic similarity
        """
        # Convert hash to a vector of floats
        vector = []
        for i in range(0, len(hash_str), 2):
            hex_pair = hash_str[i:i+2] if i+2 <= len(hash_str) else hash_str[i:]
            # Convert hex to int, normalize to range [0, 1]
            value = int(hex_pair, 16) / 255.0
            vector.append(value)

        # Pad or truncate to a fixed size (e.g., 128 dimensions)
        while len(vector) < 128:
            vector.append(0.0)

        return vector[:128]  # Return exactly 128 dimensions

    def collection_exists(self) -> bool:
        """
        Check if the collection exists
        """
        try:
            collections = self.client.get_collections()
            return any(col.name == self.collection_name for col in collections.collections)
        except:
            return False

    def get_collection_count(self) -> int:
        """
        Get the number of points in the collection
        """
        try:
            count = self.client.count(collection_name=self.collection_name)
            return count.count
        except:
            return 0