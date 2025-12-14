import React from 'react';
import ChatWidget from './components/ChatWidget/ChatWidget';

// Root component that provides chat widget only (auth context provided at layout level)
export default function Root({ children }: { children: React.ReactNode }) {
  console.log('Root component rendering, wrapping with ChatWidget only');
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}