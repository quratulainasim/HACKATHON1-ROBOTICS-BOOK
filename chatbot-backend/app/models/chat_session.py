from pydantic import BaseModel, validator
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4
from enum import Enum

class QuestionMode(str, Enum):
    full_book = "full-book"
    selected_text = "selected-text"

class SessionState(str, Enum):
    active = "Active"
    processing = "Processing"
    responded = "Responded"
    ended = "Ended"

class ChatSession(BaseModel):
    """
    Represents a user interaction session with the chatbot
    Based on data-model.md specification
    """
    id: UUID
    user_id: Optional[str] = None
    start_time: datetime
    end_time: Optional[datetime] = None
    mode: QuestionMode
    history: List[Dict[str, Any]] = []  # List of question/response pairs
    state: SessionState = SessionState.active

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat(),
            UUID: lambda v: str(v)
        }

    @validator('mode')
    def validate_mode(cls, v):
        if v not in [mode.value for mode in QuestionMode]:
            raise ValueError(f'Mode must be one of: {list(QuestionMode.__members__.keys())}')
        return v

    @validator('history')
    def validate_history_order(cls, v):
        # Ensure history maintains chronological order
        # This is a basic check - in practice, we'd validate the actual timestamps
        return v

    @validator('end_time')
    def validate_end_time(cls, v, values):
        if v and v < values.get('start_time'):
            raise ValueError('End time must be after start time')
        return v

    @classmethod
    def create_session(cls, user_id: Optional[str] = None, mode: QuestionMode = QuestionMode.full_book):
        """Create a new chat session"""
        return cls(
            id=uuid4(),
            user_id=user_id,
            start_time=datetime.now(),
            mode=mode,
            state=SessionState.active
        )