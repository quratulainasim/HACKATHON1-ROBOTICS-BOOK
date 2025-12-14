from typing import List, Dict, Any
from openai import OpenAI
from app.models.context import Context
from app.config.settings import settings
from app.utils.text_processor import clean_text
import re

class AIService:
    """
    Service to interact with OpenAI-compatible Gemini API for generating responses
    Based on requirements for using OpenAI Agents SDK (Gemini model)
    """

    def __init__(self):
        # Initialize the OpenAI client with the API key from settings
        # Using Google's Gemini API through OpenAI-compatible endpoint
        import os
        from dotenv import load_dotenv

        # Try to load API key from settings first, then from environment directly
        api_key = settings.gemini_api_key
        if not api_key:
            # Load environment variables to ensure they're available
            env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), ".env")
            if os.path.exists(env_path):
                load_dotenv(env_path)
            api_key = os.getenv("GEMINI_API_KEY")

        if api_key:
            self.client = OpenAI(
                api_key=api_key,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/"  # Google's OpenAI-compatible endpoint
            )
            model_name_env = os.getenv("GEMINI_MODEL")
            self.model_name = model_name_env or settings.gemini_model or "gemini-2.5-flash"
        else:
            print("Warning: GEMINI_API_KEY not set.")
            self.client = None
            self.model_name = "gemini-pro"

    def generate_response(self, question: str, contexts: List[Context]) -> Dict[str, Any]:
        """
        Generate a response based on the question and provided contexts
        """
        if not contexts:
            return {
                'content': "I couldn't find any relevant information in the book to answer your question.",
                'confidence': 0.0,
                'groundedness_score': 0.0,
                'sources': []
            }

        # Combine all contexts into a single context string
        context_str = ""
        sources = []
        for ctx in contexts:
            context_str += f"\n\nFrom {ctx.source_path}:\n{ctx.content}"
            if ctx.source_path not in sources:
                sources.append(ctx.source_path)

        # Create the prompt for the AI
        prompt = self._create_prompt(question, context_str)

        try:
            if self.client:
                # Use the actual OpenAI-compatible Gemini API
                response = self.client.chat.completions.create(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": "You are an educational assistant for the Physical AI Robotics Book. Answer based only on the provided context."},
                        {"role": "user", "content": prompt}
                    ],
                    temperature=0.3,  # Lower temperature for more factual responses
                    max_tokens=1000
                )

                if response and response.choices and response.choices[0].message:
                    response_content = response.choices[0].message.content

                    # Calculate confidence based on source availability and response quality
                    confidence = min(0.95, 0.3 + len(sources) * 0.2)

                    # Calculate groundedness score
                    groundedness_score = self.validate_response_groundedness(response_content, context_str if context_str else prompt)

                    return {
                        'content': response_content,
                        'confidence': confidence,
                        'groundedness_score': groundedness_score,
                        'sources': sources
                    }
                else:
                    # Fallback if no response from API
                    return self._fallback_response(sources)
            else:
                # Fallback to mock implementation if no API key
                return self._mock_generate_response_with_context(question, context_str, sources)
        except Exception as e:
            print(f"Error calling AI service: {str(e)}")
            return {
                'content': "Sorry, I encountered an error while processing your question. Please try again.",
                'confidence': 0.1,
                'groundedness_score': 0.0,
                'sources': []
            }

    def _create_prompt(self, question: str, context: str) -> str:
        """
        Create a prompt for the AI model with the question and context
        """
        prompt = f"""You are an educational assistant for the Physical AI Robotics Book.
Your task is to answer questions based ONLY on the provided context.
Do not use any external knowledge or information not present in the context.
If the answer is not available in the context, clearly state that.

Context:
{context}

Question: {question}

Answer (be concise, educational, and stick strictly to the provided context):
"""

        return prompt.strip()

    def _fallback_response(self, sources: List[str]) -> Dict[str, Any]:
        """
        Fallback response when API is not available
        """
        return {
            'content': "I'm currently unable to generate a response. Please check that your API key is valid and that you have internet connectivity.",
            'confidence': 0.1,
            'groundedness_score': 0.0,
            'sources': sources
        }

    def _mock_generate_response_with_context(self, question: str, context_str: str, sources: List[str]) -> Dict[str, Any]:
        """
        Mock implementation that still processes context when API is not available
        """
        question_lower = question.lower()
        context_lower = context_str.lower()

        # Find sentences in context that are most relevant to the question
        sentences = context_str.split('.')
        relevant_sentences = []

        for sentence in sentences:
            sentence_clean = clean_text(sentence)
            if any(word in sentence_lower for word in question_lower.split()[:3]):
                relevant_sentences.append(sentence_clean)

        if relevant_sentences:
            response = ". ".join(relevant_sentences[:3])  # Take up to 3 relevant sentences
            response = response.strip()
            if not response.endswith('.'):
                response += '.'
        else:
            response = "Based on the provided book content, I couldn't find specific information to answer your question. Please check the relevant chapters in the Physical AI Robotics Book."

        return {
            'content': response,
            'confidence': 0.3,  # Lower confidence for mock response
            'groundedness_score': 0.7,  # Moderate grounding since it's based on context
            'sources': sources
        }

    def validate_response_groundedness(self, response: str, context: str) -> float:
        """
        Validate how well the response is grounded in the provided context
        Returns a score between 0 and 1
        """
        if not response or not context:
            return 0.0

        # Normalize the texts
        response_lower = response.lower()
        context_lower = context.lower()

        # Extract key terms from context
        context_words = set(word.strip('.,;:!?()[]{}"\'') for word in context_lower.split() if len(word.strip('.,;:!?()[]{}"\'')) > 2)
        response_words = set(word.strip('.,;:!?()[]{}"\'') for word in response_lower.split() if len(word.strip('.,;:!?()[]{}"\'')) > 2)

        # Calculate overlap
        if not context_words:
            return 0.0

        # Calculate Jaccard similarity for better accuracy
        intersection = len(context_words.intersection(response_words))
        union = len(context_words.union(response_words))

        if union == 0:
            return 0.0

        jaccard_score = intersection / union

        # Also consider the percentage of response words that appear in context
        if response_words:
            response_coverage = intersection / len(response_words)
        else:
            response_coverage = 1.0  # If no words in response, it's perfectly grounded (joke, but we'll use jaccard)

        # Combine scores for a more robust groundedness measure
        combined_score = (jaccard_score * 0.7 + response_coverage * 0.3)

        # Ensure score is between 0 and 1
        return min(1.0, max(0.0, combined_score))

    def check_for_external_knowledge(self, response: str, context: str) -> bool:
        """
        Check if the response contains information not present in the context
        Returns True if external knowledge is detected (which violates requirements)
        """
        # This is a simplified check - in a real implementation, we'd use more sophisticated methods
        # For now, we'll check if the response contains specific claims not supported by context

        # Convert to lowercase for comparison
        response_lower = response.lower()
        context_lower = context.lower()

        # Look for dates, specific numbers, or technical terms that might indicate external knowledge


        # Extract potential facts from response that should be in context
        # This is a basic implementation - a full implementation would require NLP
        response_sentences = re.split(r'[.!?]+', response_lower)
        context_sentences = re.split(r'[.!?]+', context_lower)

        # Check if response sentences have strong similarity to context sentences
        for resp_sent in response_sentences:
            resp_sent = resp_sent.strip()
            if len(resp_sent) < 10:  # Skip very short sentences
                continue

            # Check if this sentence or a similar one exists in context
            found_match = False
            for ctx_sent in context_sentences:
                ctx_sent = ctx_sent.strip()
                if len(ctx_sent) < 10:
                    continue

                # Simple similarity check
                if self._calculate_sentence_similarity(resp_sent, ctx_sent) > 0.3:
                    found_match = True
                    break

            if not found_match and len(resp_sent) > 20:  # Longer sentences should have matches
                return True  # Potential external knowledge detected

        return False

    def _calculate_sentence_similarity(self, sent1: str, sent2: str) -> float:
        """
        Calculate similarity between two sentences using word overlap
        """
        words1 = set(sent1.split())
        words2 = set(sent2.split())

        if not words1 and not words2:
            return 1.0
        if not words1 or not words2:
            return 0.0

        intersection = len(words1.intersection(words2))
        union = len(words1.union(words2))

        return intersection / union if union > 0 else 0.0