import React from 'react';

interface InputAreaProps {
  value: string;
  onChange: (value: string) => void;
  onSubmit: (e: React.FormEvent) => void;
  isLoading: boolean;
  placeholder?: string;
}

const InputArea: React.FC<InputAreaProps> = ({
  value,
  onChange,
  onSubmit,
  isLoading,
  placeholder = "Type your message..."
}) => {
  return (
    <form onSubmit={onSubmit} className="chat-input-form">
      <input
        type="text"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        placeholder={placeholder}
        disabled={isLoading}
      />
      <button type="submit" disabled={isLoading || !value.trim()}>
        {isLoading ? 'Sending...' : '→'}
      </button>
    </form>
  );
};

export default InputArea;