import { useState } from 'react';
import { useAppConfig } from './useAppConfig';

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
  const config = useAppConfig();

  const sendMessage = async (
    question: string,
    selectedText?: string
  ): Promise<ChatResponse> => {
    setIsLoading(true);
    setError(null);

    try {
      const chatServerUrl = config.chatServerUrl;

      let response;
      if (selectedText) {
        // Use selected text mode with correct API endpoint
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
        // Use full book mode with correct API endpoint
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
        const errorData = await response.json();
        throw new Error(errorData.message || 'Failed to get response');
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