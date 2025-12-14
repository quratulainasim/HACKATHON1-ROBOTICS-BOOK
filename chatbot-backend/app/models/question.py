from pydantic import BaseModel, validator
from typing import Optional
from datetime import datetime
from uuid import UUID, uuid4
from enum import Enum

class QuestionMode(str, Enum):
    full_book = "full-book"
    selected_text = "selected-text"

class Question(BaseModel):
    """
    Represents a natural language question from a user
    Based on data-model.md specification
    """
    id: UUID
    content: str
    mode: QuestionMode
    selected_text: Optional[str] = None
    timestamp: datetime
    user_id: Optional[str] = None

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat(),
            UUID: lambda v: str(v)
        }

    @validator('content')
    def validate_content_length(cls, v):
        if len(v) < 5:
            raise ValueError('Question content must be at least 5 characters')
        if len(v) > 500:
            raise ValueError('Question content must not exceed 500 characters')
        return v

    @validator('mode')
    def validate_mode(cls, v):
        if v not in [mode.value for mode in QuestionMode]:
            raise ValueError(f'Mode must be one of: {list(QuestionMode.__members__.keys())}')
        return v

    @validator('selected_text')
    def validate_selected_text(cls, v, values):
        if values.get('mode') == 'selected-text' and not v:
            raise ValueError('selected_text is required when mode is selected-text')
        return v

    @classmethod
    def create_question(cls, content: str, mode: QuestionMode, selected_text: Optional[str] = None, user_id: Optional[str] = None):
        """Create a Question instance with validation"""
        return cls(
            id=uuid4(),
            content=content,
            mode=mode,
            selected_text=selected_text,
            timestamp=datetime.now(),
            user_id=user_id
        )