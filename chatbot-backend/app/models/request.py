from pydantic import BaseModel
from typing import Optional

class FullBookQueryRequest(BaseModel):
    """
    Request model for asking questions using full book context
    Based on the OpenAPI contract
    """
    question: str
    user_id: Optional[str] = None

    class Config:
        schema_extra = {
            "example": {
                "question": "Explain how ROS 2 handles message passing between nodes",
                "user_id": "user-12345"
            }
        }

class SelectedTextQueryRequest(BaseModel):
    """
    Request model for asking questions using selected text context
    Based on the OpenAPI contract
    """
    question: str
    selected_text: str
    user_id: Optional[str] = None

    class Config:
        schema_extra = {
            "example": {
                "question": "What does this mean?",
                "selected_text": "ROS 2 uses a publish-subscribe model for message passing between nodes...",
                "user_id": "user-12345"
            }
        }