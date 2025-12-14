import re
from typing import List
from bs4 import BeautifulSoup

def extract_headings(content: str) -> List[str]:
    """
    Extract headings from markdown content
    Returns a list of headings found in the content
    """
    headings = []

    # Extract markdown headings (lines starting with #)
    lines = content.split('\n')
    for line in lines:
        line = line.strip()
        # Match markdown headings: #, ##, ###, etc.
        if line.startswith('#'):
            heading = line.lstrip('#').strip()
            if heading:  # Only add non-empty headings
                headings.append(heading)

    return headings

def extract_text_content(markdown_content: str) -> str:
    """
    Extract plain text content from markdown by removing markdown syntax
    """
    # Remove markdown headings but keep the text
    content = re.sub(r'^#+\s*', '', markdown_content, flags=re.MULTILINE)

    # Remove bold and italic markers
    content = re.sub(r'\*\*(.*?)\*\*', r'\1', content)
    content = re.sub(r'\*(.*?)\*', r'\1', content)
    content = re.sub(r'__(.*?)__', r'\1', content)
    content = re.sub(r'_(.*?)_', r'\1', content)

    # Remove links [text](url) -> text
    content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)

    # Remove images ![alt](url) -> alt
    content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', r'\1', content)

    # Remove inline code
    content = re.sub(r'`([^`]+)`', r'\1', content)

    # Remove code blocks
    content = re.sub(r'```.*?\n(.*?)```', '', content, flags=re.DOTALL)

    # Remove blockquotes
    content = re.sub(r'^>\s*', '', content, flags=re.MULTILINE)

    # Remove lists markers
    content = re.sub(r'^\s*[\*\-\+]\s+', '', content, flags=re.MULTILINE)
    content = re.sub(r'^\s*\d+\.\s+', '', content, flags=re.MULTILINE)

    return content.strip()

def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace and normalizing
    """
    # Replace multiple whitespace with single space
    text = re.sub(r'\s+', ' ', text)

    # Remove leading/trailing whitespace
    text = text.strip()

    return text

def chunk_text(text: str, max_chunk_size: int = 2000) -> List[str]:
    """
    Split text into chunks of specified maximum size
    Respects sentence boundaries when possible
    """
    if len(text) <= max_chunk_size:
        return [text]

    chunks = []
    sentences = re.split(r'[.!?]+', text)

    current_chunk = ""
    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue

        if len(current_chunk + " " + sentence) <= max_chunk_size:
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = sentence

    # Add the last chunk if it exists
    if current_chunk:
        chunks.append(current_chunk.strip())

    # If any chunk is still too large, split by character count
    final_chunks = []
    for chunk in chunks:
        if len(chunk) <= max_chunk_size:
            final_chunks.append(chunk)
        else:
            # Split by character count as a fallback
            for i in range(0, len(chunk), max_chunk_size):
                final_chunks.append(chunk[i:i + max_chunk_size])

    return final_chunks

def calculate_relevance_score(text: str, query: str) -> float:
    """
    Calculate a basic relevance score between text and query
    This is a simple keyword matching approach
    """
    if not text or not query:
        return 0.0

    text_lower = text.lower()
    query_lower = query.lower()

    # Count matching words
    query_words = set(query_lower.split())
    text_words = set(text_lower.split())

    matching_words = query_words.intersection(text_words)

    if not query_words:
        return 0.0

    # Calculate score as ratio of matching words to total query words
    score = len(matching_words) / len(query_words)

    # Boost score if query appears as phrase in text
    if query_lower in text_lower:
        score = min(1.0, score * 1.5)

    return min(1.0, score)