import React, { useState, useEffect, useRef } from 'react';
import { useChat } from '../hooks/useChat';
import './ChatWidget.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

const ChatWidget: React.FC = () => {
  const { sendMessage, isLoading, error } = useChat();
  const [inputValue, setInputValue] = useState('');
  const [messages, setMessages] = useState<Message[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [mode, setMode] = useState<'full-book' | 'selected-text'>('full-book');
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Get selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const text = window.getSelection()?.toString().trim();
      if (text) {
        setSelectedText(text);
        // If chat is open and in selected-text mode, automatically populate the input
        if (isOpen && mode === 'selected-text' && !inputValue.trim()) {
          setInputValue(text);
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, [isOpen, mode, inputValue]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim()) return;

    // Add user message to UI immediately
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    const userQuestion = inputValue;
    setInputValue('');

    try {
      // Send message based on mode
      let response;
      if (mode === 'selected-text' && selectedText) {
        response = await sendMessage(userQuestion, selectedText);
      } else {
        response = await sendMessage(userQuestion);
      }

      // Add AI response to UI
      const aiMessage: Message = {
        id: response.id || Date.now().toString(),
        content: response.answer || response.content || "I'm sorry, I couldn't process that question.",
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (err) {
      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an error. Please try again.',
        role: 'assistant',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    }
  };

  const toggleChat = () => {
    const newOpenState = !isOpen;
    setIsOpen(newOpenState);
    // If opening chat in selected-text mode and there's selected text, populate the input
    if (newOpenState && mode === 'selected-text' && selectedText && !inputValue.trim()) {
      setInputValue(selectedText);
    }
  };

  const switchMode = () => {
    const newMode = mode === 'full-book' ? 'selected-text' : 'full-book';
    setMode(newMode);
    // If switching to selected-text mode and there's selected text, populate the input
    if (newMode === 'selected-text' && selectedText && !inputValue.trim()) {
      setInputValue(selectedText);
    }
  };

  return (
    <div className="chat-widget">
      {!isOpen && (
        <button className="chat-toggle-btn" onClick={toggleChat} style={{display: 'flex', alignItems: 'center', justifyContent: 'center'}}>
          💬
        </button>
      )}
      {isOpen && (
        <div className="chat-container">
          <div className="chat-header">
            <h3>Book Assistant</h3>
            <div className="chat-controls">
              <span className={`mode-indicator ${mode}`}>
                {mode === 'full-book' ? '📚 Full Book' : '✏️ Selected Text'}
              </span>
              <button onClick={switchMode} className="mode-switch-btn" title="Switch mode">
                {mode === 'full-book' ? '✏️' : '📚'}
              </button>
              <button onClick={toggleChat} className="close-btn">✕</button>
            </div>
          </div>

          {mode === 'selected-text' && selectedText && (
            <div className="selected-text-preview">
              <small>Context: "{selectedText.substring(0, 50)}..."</small>
            </div>
          )}

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.role}`}
              >
                <div className="message-content">
                  {message.content}
                </div>
                <small className="message-timestamp">
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </small>
              </div>
            ))}
            {isLoading && (
              <div className="message assistant">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {error && (
            <div className="error-message">
              {error}
            </div>
          )}

          <form onSubmit={handleSubmit} className="chat-input-form">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder={`Ask about the book... ${mode === 'selected-text' ? '(using selected text)' : '(full book context)'}`}
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading || !inputValue.trim()}>
              {isLoading ? 'Sending...' : '→'}
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;