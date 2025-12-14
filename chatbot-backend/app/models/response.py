from pydantic import BaseModel, validator
from typing import List, Optional
from datetime import datetime
from uuid import UUID, uuid4
from enum import Enum

class QuestionMode(str, Enum):
    full_book = "full-book"
    selected_text = "selected-text"

class Response(BaseModel):
    """
    Represents an AI-generated response to a question
    Based on data-model.md specification
    """
    id: UUID
    question_id: str
    content: str
    confidence: float
    groundedness_score: float
    timestamp: datetime
    mode: QuestionMode
    sources: List[str] = []

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat(),
            UUID: lambda v: str(v)
        }

    @validator('confidence')
    def validate_confidence(cls, v):
        if not 0 <= v <= 1:
            raise ValueError('Confidence must be between 0 and 1')
        return v

    @validator('groundedness_score')
    def validate_groundedness_score(cls, v):
        if not 0 <= v <= 1:
            raise ValueError('Groundedness score must be between 0 and 1')
        return v

    @validator('content')
    def validate_content_length(cls, v):
        # Check if content exceeds model token limits (approximate character count)
        if len(v) > 10000:  # Rough estimate for token limit
            raise ValueError('Response content exceeds maximum length')
        return v

    @classmethod
    def create_response(cls, question_id: str, content: str, confidence: float,
                       groundedness_score: float, mode: QuestionMode,
                       sources: List[str] = None):
        """Create a Response instance with validation"""
        return cls(
            id=uuid4(),
            question_id=question_id,
            content=content,
            confidence=confidence,
            groundedness_score=groundedness_score,
            timestamp=datetime.now(),
            mode=mode,
            sources=sources or []
        )