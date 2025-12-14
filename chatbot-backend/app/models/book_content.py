from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from uuid import UUID, uuid4

class BookContent(BaseModel):
    """
    Represents a content block from the docs/ folder
    Based on data-model.md specification
    """
    id: UUID
    path: str
    title: str
    content: str
    headings: List[str]
    tags: List[str]
    last_modified: datetime

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat(),
            UUID: lambda v: str(v)
        }

    @classmethod
    def create_from_file(cls, file_path: str, content: str, title: str = ""):
        """Create a BookContent instance from a file path and content"""
        from app.utils.text_processor import extract_headings

        headings = extract_headings(content)
        tags = []  # Extract tags from content or file metadata

        return cls(
            id=uuid4(),
            path=file_path,
            title=title or file_path.split("/")[-1],
            content=content,
            headings=headings,
            tags=tags,
            last_modified=datetime.now()
        )