import React from 'react';

interface MessageProps {
  content: string;
  role: 'user' | 'assistant';
  timestamp?: Date;
}

const Message: React.FC<MessageProps> = ({ content, role, timestamp }) => {
  return (
    <div className={`message ${role}`}>
      <div className="message-content">
        {content}
      </div>
      {timestamp && (
        <small className="message-timestamp">
          {timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </small>
      )}
    </div>
  );
};

export default Message;