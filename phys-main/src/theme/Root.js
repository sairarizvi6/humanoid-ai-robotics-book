// src/theme/Root.js
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

// This component wraps the entire Docusaurus app
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}