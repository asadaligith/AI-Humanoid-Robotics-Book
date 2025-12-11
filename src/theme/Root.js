import React from 'react';
import ChatBot from '../components/ChatBot';

/**
 * Root component wrapper for Docusaurus
 * This wraps all pages and adds global components like the ChatBot
 */
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}
