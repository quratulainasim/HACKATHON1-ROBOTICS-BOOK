from typing import List, Dict, Optional
from app.models.context import Context, SourceType
from app.models.book_content import BookContent
from app.utils.content_reader import ContentReader
from app.utils.text_processor import extract_headings, calculate_relevance_score, chunk_text
from app.config.settings import settings
from app.services.qdrant_service import QdrantService
from uuid import uuid4
import os

class ContextExtractor:
    """
    Service for deterministic context extraction from book content
    Based on requirements for deterministic extraction without embeddings
    """

    def __init__(self):
        self.content_reader = ContentReader(settings.docs_path)
        self.qdrant_service = QdrantService()

    def extract_context_for_full_book(self, query: str, max_tokens: int = None) -> List[Context]:
        """
        Extract context from the full book based on the query
        Uses deterministic rules: page- and heading-based context with neighboring sections
        Now uses Qdrant for efficient retrieval
        """
        if max_tokens is None:
            max_tokens = settings.max_context_tokens

        # First, ensure Qdrant collection exists and is populated
        if not self.qdrant_service.collection_exists():
            print("Qdrant collection does not exist. Creating and indexing content...")
            self.qdrant_service.index_docs_content()

        # Search for relevant contexts in Qdrant
        qdrant_contexts = self.qdrant_service.search_contexts(query, limit=20)

        contexts = []
        total_tokens = 0

        # Process contexts from Qdrant
        for ctx_data in qdrant_contexts:
            relevance_score = ctx_data['relevance_score']

            # Only include content that has some relevance
            if relevance_score > 0.1:  # Threshold to filter out irrelevant content
                content_to_process = ctx_data['content']

                # Check if content exceeds the Context model limit and chunk if needed
                if len(content_to_process) > 13000:  # Max length from Context validation
                    # Split the content into smaller chunks
                    chunks = chunk_text(content_to_process, 12000)  # Use slightly less than max for safety
                    for i, chunk in enumerate(chunks):
                        chunk_relevance = relevance_score  # Keep same relevance for all chunks

                        # Adjust relevance slightly for subsequent chunks to prefer earlier ones
                        if i > 0:
                            chunk_relevance = relevance_score * (1.0 - (i * 0.1))  # Reduce relevance for later chunks

                        chunk_context = Context.create_context(
                            source_type=SourceType.section,
                            content=chunk,
                            source_path=ctx_data['source_path'],
                            relevance_score=max(0.0, chunk_relevance),  # Ensure non-negative
                            metadata={
                                'title': ctx_data['title'],
                                'headings': extract_headings(chunk),
                                'last_modified': ctx_data['metadata']['last_modified'],
                                'chunk_info': f'chunk_{i+1}_of_{len(chunks)}'
                            }
                        )

                        # Estimate token count (roughly 4 chars per token)
                        estimated_tokens = len(chunk) // 4

                        if total_tokens + estimated_tokens <= max_tokens:
                            contexts.append(chunk_context)
                            total_tokens += estimated_tokens
                        else:
                            break  # Stop if we exceed the max tokens
                else:
                    # Content is within limits, create context directly
                    context = Context.create_context(
                        source_type=SourceType.chapter,  # or section based on structure
                        content=content_to_process,
                        source_path=ctx_data['source_path'],
                        relevance_score=relevance_score,
                        metadata={
                            'title': ctx_data['title'],
                            'headings': ctx_data['metadata']['headings'],
                            'last_modified': ctx_data['metadata']['last_modified']
                        }
                    )

                    # Estimate token count (roughly 4 chars per token)
                    estimated_tokens = len(content_to_process) // 4

                    if total_tokens + estimated_tokens <= max_tokens:
                        contexts.append(context)
                        total_tokens += estimated_tokens
                    else:
                        # If adding this context would exceed the limit,
                        # we need to chunk it or stop adding more
                        remaining_tokens = max_tokens - total_tokens
                        if remaining_tokens > 100:  # Only add if there's meaningful space left
                            # Chunk the content to fit remaining tokens
                            chunk_size = remaining_tokens * 4  # Convert back to characters
                            chunks = chunk_text(content_to_process, chunk_size)

                            if chunks:
                                chunk_context = Context.create_context(
                                    source_type=SourceType.section,
                                    content=chunks[0],  # Take the first chunk that fits
                                    source_path=ctx_data['source_path'],
                                    relevance_score=relevance_score,
                                    metadata={
                                        'title': ctx_data['title'],
                                        'headings': extract_headings(chunks[0]),
                                        'last_modified': ctx_data['metadata']['last_modified'],
                                        'chunk_info': 'first_chunk'
                                    }
                                )
                                contexts.append(chunk_context)
                                break  # Stop after adding the chunk since we're at limit
                        else:
                            break  # No more space to add content

        # Sort contexts by relevance score in descending order
        contexts.sort(key=lambda x: x.relevance_score, reverse=True)

        return contexts

    def extract_context_for_selected_text(self, selected_text: str, query: str) -> List[Context]:
        """
        Extract context based only on the selected text
        """
        # Create context directly from the selected text
        context = Context.create_context(
            source_type=SourceType.selected_text,
            content=selected_text,
            source_path="selected-text",
            relevance_score=1.0,  # Selected text is 100% relevant by definition
            metadata={
                'query': query,
                'creation_method': 'selected_text'
            }
        )

        return [context]

    def extract_context_by_path(self, path: str, query: str = None) -> Optional[Context]:
        """
        Extract context from a specific file path
        """
        try:
            content_block = self.content_reader.read_content_by_path(path)

            relevance_score = 1.0  # If specific path is requested, assume high relevance
            if query:
                relevance_score = calculate_relevance_score(content_block['content'], query)

            context = Context.create_context(
                source_type=SourceType.section,
                content=content_block['content'],
                source_path=path,
                relevance_score=relevance_score,
                metadata={
                    'title': content_block['title'],
                    'headings': extract_headings(content_block['content']),
                    'last_modified': content_block['last_modified']
                }
            )

            return context
        except FileNotFoundError:
            return None

    def extract_context_by_heading(self, heading: str, query: str = None) -> List[Context]:
        """
        Extract context based on a specific heading
        """
        all_content = self.content_reader.read_all_content()
        contexts = []

        for content_block in all_content:
            headings = extract_headings(content_block['content'])
            # Check if the requested heading is in the content's headings
            if any(heading.lower() in h.lower() for h in headings):
                relevance_score = 1.0
                if query:
                    relevance_score = calculate_relevance_score(content_block['content'], query)

                context = Context.create_context(
                    source_type=SourceType.section,
                    content=content_block['content'],
                    source_path=content_block['path'],
                    relevance_score=relevance_score,
                    metadata={
                        'title': content_block['title'],
                        'headings': headings,
                        'target_heading': heading,
                        'last_modified': content_block['last_modified']
                    }
                )

                contexts.append(context)

        return contexts