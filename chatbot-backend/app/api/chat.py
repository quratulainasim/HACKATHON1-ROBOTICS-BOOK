from fastapi import APIRouter, HTTPException, status
from typing import Optional
from app.models.request import FullBookQueryRequest, SelectedTextQueryRequest
from app.models.response_models import QueryResponse, ErrorResponse
from app.services.chat_service import ChatService
from app.config.settings import settings
import uuid
from fastapi.responses import StreamingResponse
import json

router = APIRouter()
chat_service = ChatService()

@router.post("/query", response_model=QueryResponse)
async def ask_full_book_question(request: FullBookQueryRequest):
    """
    Endpoint to ask a question using full book context
    Based on the OpenAPI contract: POST /chat/query
    """
    try:
        # Validate the question
        if not chat_service.validate_question(request.question):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": "INVALID_QUESTION",
                    "message": "Question must be between 5 and 500 characters"
                }
            )

        # Process the query using the chat service
        response = chat_service.process_full_book_query(
            question_text=request.question,
            user_id=request.user_id
        )

        # Return the response in the required format
        return QueryResponse(
            id=str(response.id),
            answer=response.content,
            sources=response.sources,
            confidence=response.confidence,
            groundedness_score=response.groundedness_score
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Handle any other errors
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "INTERNAL_ERROR",
                "message": f"An error occurred while processing your request: {str(e)}"
            }
        )

@router.post("/selected-text", response_model=QueryResponse)
async def ask_selected_text_question(request: SelectedTextQueryRequest):
    """
    Endpoint to ask a question using selected text context
    Based on the OpenAPI contract: POST /chat/selected-text
    """
    try:
        # Validate the question
        if not chat_service.validate_question(request.question):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": "INVALID_QUESTION",
                    "message": "Question must be between 5 and 500 characters"
                }
            )

        # Validate the selected text
        if not chat_service.validate_selected_text(request.selected_text):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": "INVALID_SELECTED_TEXT",
                    "message": "Selected text must be at least 10 characters"
                }
            )

        # Process the query using the chat service
        response = chat_service.process_selected_text_query(
            question_text=request.question,
            selected_text=request.selected_text,
            user_id=request.user_id
        )

        # Return the response in the required format
        return QueryResponse(
            id=str(response.id),
            answer=response.content,
            sources=response.sources,
            confidence=response.confidence,
            groundedness_score=response.groundedness_score
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Handle any other errors
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "INTERNAL_ERROR",
                "message": f"An error occurred while processing your request: {str(e)}"
            }
        )


@router.post("/query-stream")
async def ask_full_book_question_stream(request: FullBookQueryRequest):
    """
    Streaming endpoint to ask a question using full book context
    Provides real-time response as it's being generated
    """
    async def generate_stream():
        try:
            # Validate the question
            if not chat_service.validate_question(request.question):
                yield json.dumps({
                    "error": "INVALID_QUESTION",
                    "message": "Question must be between 5 and 500 characters"
                }) + "\n"
                return

            # Process the query using the chat service
            response = chat_service.process_full_book_query(
                question_text=request.question,
                user_id=request.user_id
            )

            # Yield the response in chunks for streaming
            answer = response.content
            chunk_size = 20  # characters per chunk

            for i in range(0, len(answer), chunk_size):
                chunk = answer[i:i + chunk_size]
                yield json.dumps({
                    "id": str(response.id),
                    "chunk": chunk,
                    "sources": response.sources if i == 0 else [],  # Only send sources with first chunk
                    "confidence": response.confidence if i == 0 else None,
                    "groundedness_score": response.groundedness_score if i == 0 else None
                }) + "\n"

        except Exception as e:
            yield json.dumps({
                "error": "INTERNAL_ERROR",
                "message": f"An error occurred while processing your request: {str(e)}"
            }) + "\n"

    return StreamingResponse(generate_stream(), media_type="application/x-ndjson")


@router.post("/selected-text-stream")
async def ask_selected_text_question_stream(request: SelectedTextQueryRequest):
    """
    Streaming endpoint to ask a question using selected text context
    Provides real-time response as it's being generated
    """
    async def generate_stream():
        try:
            # Validate the question
            if not chat_service.validate_question(request.question):
                yield json.dumps({
                    "error": "INVALID_QUESTION",
                    "message": "Question must be between 5 and 500 characters"
                }) + "\n"
                return

            # Validate the selected text
            if not chat_service.validate_selected_text(request.selected_text):
                yield json.dumps({
                    "error": "INVALID_SELECTED_TEXT",
                    "message": "Selected text must be at least 10 characters"
                }) + "\n"
                return

            # Process the query using the chat service
            response = chat_service.process_selected_text_query(
                question_text=request.question,
                selected_text=request.selected_text,
                user_id=request.user_id
            )

            # Yield the response in chunks for streaming
            answer = response.content
            chunk_size = 20  # characters per chunk

            for i in range(0, len(answer), chunk_size):
                chunk = answer[i:i + chunk_size]
                yield json.dumps({
                    "id": str(response.id),
                    "chunk": chunk,
                    "sources": response.sources if i == 0 else [],  # Only send sources with first chunk
                    "confidence": response.confidence if i == 0 else None,
                    "groundedness_score": response.groundedness_score if i == 0 else None
                }) + "\n"

        except Exception as e:
            yield json.dumps({
                "error": "INTERNAL_ERROR",
                "message": f"An error occurred while processing your request: {str(e)}"
            }) + "\n"

    return StreamingResponse(generate_stream(), media_type="application/x-ndjson")