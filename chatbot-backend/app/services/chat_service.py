from typing import Optional, List
from app.models.question import Question, QuestionMode
from app.models.response import Response
from app.models.chat_session import ChatSession
from app.models.context import Context
from app.services.context_extractor import ContextExtractor
from app.services.ai_service import AIService
from app.config.settings import settings
from datetime import datetime
from uuid import uuid4

class ChatService:
    """
    Core service for handling chat interactions
    Orchestrates context extraction, AI processing, and response generation
    """

    def __init__(self):
        self.context_extractor = ContextExtractor()
        self.ai_service = AIService()

    def process_full_book_query(self, question_text: str, user_id: Optional[str] = None) -> Response:
        """
        Process a query using the full book context
        """
        # Create question object
        question = Question.create_question(
            content=question_text,
            mode=QuestionMode.full_book,
            user_id=user_id
        )

        # Extract context from the full book based on the question
        contexts = self.context_extractor.extract_context_for_full_book(
            query=question_text
        )

        if not contexts:
            # If no relevant context found, return appropriate response
            return Response.create_response(
                question_id=str(question.id),
                content="I couldn't find any relevant information in the book to answer your question. Please try rephrasing or check the relevant chapters.",
                confidence=0.1,
                groundedness_score=0.0,
                mode=QuestionMode.full_book,
                sources=[]
            )

        # Generate response using AI service
        ai_response = self.ai_service.generate_response(question_text, contexts)

        # Create response object
        response = Response.create_response(
            question_id=str(question.id),
            content=ai_response['content'],
            confidence=ai_response['confidence'],
            groundedness_score=ai_response['groundedness_score'],
            mode=QuestionMode.full_book,
            sources=ai_response['sources']
        )

        return response

    def process_selected_text_query(self, question_text: str, selected_text: str, user_id: Optional[str] = None) -> Response:
        """
        Process a query using only the selected text as context
        """
        # Create question object
        question = Question.create_question(
            content=question_text,
            mode=QuestionMode.selected_text,
            selected_text=selected_text,
            user_id=user_id
        )

        # Extract context based only on the selected text
        contexts = self.context_extractor.extract_context_for_selected_text(
            selected_text=selected_text,
            query=question_text
        )

        # Generate response using AI service
        ai_response = self.ai_service.generate_response(question_text, contexts)

        # Create response object
        response = Response.create_response(
            question_id=str(question.id),
            content=ai_response['content'],
            confidence=ai_response['confidence'],
            groundedness_score=ai_response['groundedness_score'],
            mode=QuestionMode.selected_text,
            sources=ai_response['sources']
        )

        return response

    def create_session(self, user_id: Optional[str] = None, initial_mode: QuestionMode = QuestionMode.full_book) -> ChatSession:
        """
        Create a new chat session
        """
        return ChatSession.create_session(user_id, initial_mode)

    def validate_question(self, question: str) -> bool:
        """
        Validate a question before processing
        """
        if not question or len(question.strip()) < 5:
            return False
        if len(question) > settings.max_question_length:
            return False
        return True

    def validate_selected_text(self, selected_text: str) -> bool:
        """
        Validate selected text before processing
        """
        if not selected_text or len(selected_text.strip()) < 10:
            return False
        return True

    def update_session_history(self, session: ChatSession, question: Question, response: Response) -> ChatSession:
        """
        Update session history with the new question-response pair
        """
        session.history.append({
            'question_id': str(question.id),
            'response_id': str(response.id),
            'question_content': question.content,
            'response_content': response.content,
            'timestamp': datetime.now().isoformat()
        })
        return session

    def switch_session_mode(self, session: ChatSession, new_mode: QuestionMode) -> ChatSession:
        """
        Switch the mode of an existing session
        """
        session.mode = new_mode
        return session