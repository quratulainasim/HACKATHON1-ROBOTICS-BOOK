from pydantic import BaseModel
from typing import List, Optional

class QueryResponse(BaseModel):
    """
    Response model for chat queries
    Based on the OpenAPI contract
    """
    id: str
    answer: str
    sources: List[str]
    confidence: float
    groundedness_score: float

    class Config:
        schema_extra = {
            "example": {
                "id": "resp-abc123",
                "answer": "ROS 2 handles message passing through a publish-subscribe model where nodes publish messages to topics and other nodes subscribe to those topics...",
                "sources": ["/docs/module-1-ros2/nodes-topics-services.md"],
                "confidence": 0.95,
                "groundedness_score": 0.98
            }
        }

class ErrorResponse(BaseModel):
    """
    Response model for errors
    Based on the OpenAPI contract
    """
    error: str
    message: str

    class Config:
        schema_extra = {
            "example": {
                "error": "QUERY_TOO_LONG",
                "message": "Question exceeds maximum length of 500 characters"
            }
        }

class HealthResponse(BaseModel):
    """
    Response model for health checks
    Based on the OpenAPI contract
    """
    status: str
    timestamp: str

    class Config:
        schema_extra = {
            "example": {
                "status": "healthy",
                "timestamp": "2025-12-13T10:30:00Z"
            }
        }