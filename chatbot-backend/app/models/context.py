from pydantic import BaseModel, validator
from typing import Optional, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4
from enum import Enum

class SourceType(str, Enum):
    full_book = "full-book"
    selected_text = "selected-text"
    chapter = "chapter"
    section = "section"

class Context(BaseModel):
    """
    Represents relevant book content extracted for response generation
    Based on data-model.md specification
    """
    id: UUID
    source_type: SourceType
    content: str
    source_path: str
    relevance_score: float
    metadata: Dict[str, Any]

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat(),
            UUID: lambda v: str(v)
        }

    @validator('source_type')
    def validate_source_type(cls, v):
        if v not in [stype.value for stype in SourceType]:
            raise ValueError(f'Source type must be one of: {list(SourceType.__members__.keys())}')
        return v

    @validator('content')
    def validate_content_length(cls, v):
        # Ensure content is not too short (minimum meaningful context)
        if len(v) < 10:
            raise ValueError('Context content must be at least 10 characters')
        # Ensure content doesn't exceed 80% of model token limit (approximate)
        # Assuming ~4 chars per token, 80% of 4096 tokens = ~13,000 chars
        if len(v) > 13000:
            raise ValueError('Context content exceeds maximum length (80% of model token limit)')
        return v

    @validator('source_path')
    def validate_source_path(cls, v):
        # Ensure the source path exists in docs/ folder
        # This would be validated at runtime in a real implementation
        return v

    @validator('relevance_score')
    def validate_relevance_score(cls, v):
        if not 0 <= v <= 1:
            raise ValueError('Relevance score must be between 0 and 1')
        return v

    @classmethod
    def create_context(cls, source_type: SourceType, content: str, source_path: str,
                      relevance_score: float, metadata: Dict[str, Any] = None):
        """Create a Context instance with validation"""
        return cls(
            id=uuid4(),
            source_type=source_type,
            content=content,
            source_path=source_path,
            relevance_score=relevance_score,
            metadata=metadata or {}
        )