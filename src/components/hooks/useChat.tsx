import { useState } from 'react';

interface ChatResponse {
  id: string;
  answer: string;
  sources: string[];
  confidence: number;
  groundedness_score: number;
}

export const useChat = () => {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // SHORT-TERM FIX: Hardcoded production backend URL
  // This points directly to your deployed Railway app
  const chatServerUrl = 'https://hackathon1-robotics-book-production-a9bd.up.railway.app';

  const sendMessage = async (
    question: string,
    selectedText?: string
  ): Promise<ChatResponse> => {
    setIsLoading(true);
    setError(null);

    try {
      let response;

      if (selectedText) {
        // Selected text mode
        response = await fetch(`${chatServerUrl}/chat/selected-text`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question,
            selected_text: selectedText,
          }),
        });
      } else {
        // Full book query mode
        response = await fetch(`${chatServerUrl}/chat/query`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question,
          }),
        });
      }

      if (!response.ok) {
        let errorData;
        try {
          errorData = await response.json();
        } catch {
          errorData = { message: 'Server error' };
        }
        throw new Error(errorData.message || `HTTP ${response.status}: Failed to get response`);
      }

      const data: ChatResponse = await response.json();
      return data;
    } catch (err) {
      console.error('Chat error:', err);
      const errorMessage = err instanceof Error ? err.message : 'An unknown error occurred';
      setError(errorMessage);
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  return {
    sendMessage,
    isLoading,
    error,
  };
};

export default useChat;
