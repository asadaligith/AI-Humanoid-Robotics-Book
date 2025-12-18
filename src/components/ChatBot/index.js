import React, { useState, useRef, useEffect } from 'react';
import ChatMessage from './ChatMessage';
import { askQuestion } from './apiService';
import styles from './styles.module.css';

/**
 * ChatBot Widget Component
 * A floating chatbot widget for the AI Humanoid Robotics Book
 */
export default function ChatBot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: 'Hello! I\'m your AI assistant for the Humanoid Robotics Book. Ask me anything about ROS 2, Gazebo, Isaac Sim, or Physical AI!',
    },
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [sessionId, setSessionId] = useState(null);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    setError(null);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      role: 'user',
      content: inputValue.trim(),
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Call backend API with session ID
      const response = await askQuestion(userMessage.content, sessionId);

      // Store session ID from response for conversation continuity
      if (response.session_id && !sessionId) {
        setSessionId(response.session_id);
      }

      // Add assistant response to chat
      const assistantMessage = {
        role: 'assistant',
        content: response.answer,
        sources: response.sources || [],
        usage: response.usage,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      setError(err.message || 'Failed to get response. Please try again.');
      console.error('Chat error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const clearChat = () => {
    setMessages([
      {
        role: 'assistant',
        content: 'Chat cleared! How can I help you?',
      },
    ]);
    setSessionId(null);  // Reset session for new conversation
    setError(null);
  };

  return (
    <>
      {/* Chat Button */}
      <button
        className={`${styles.chatButton} ${isOpen ? styles.chatButtonActive : ''}`}
        onClick={toggleChat}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.chatTitle}>
              <span className={styles.chatIcon}>🤖</span>
              <span>AI Robotics Assistant</span>
            </div>
            <button
              className={styles.clearButton}
              onClick={clearChat}
              title="Clear chat"
            >
              🗑️
            </button>
          </div>

          {/* Messages */}
          <div className={styles.chatMessages}>
            {messages.map((message, index) => (
              <ChatMessage key={index} message={message} />
            ))}
            {isLoading && (
              <div className={styles.loadingMessage}>
                <div className={styles.loadingDots}>
                  <span>●</span>
                  <span>●</span>
                  <span>●</span>
                </div>
                <span className={styles.loadingText}>AI is thinking...</span>
              </div>
            )}
            {error && (
              <div className={styles.errorMessage}>
                ⚠️ {error}
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form className={styles.chatForm} onSubmit={handleSubmit}>
            <input
              ref={inputRef}
              type="text"
              className={styles.chatInput}
              placeholder="Ask about robotics, ROS 2, Gazebo..."
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={!inputValue.trim() || isLoading}
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </form>

          {/* Footer */}
          <div className={styles.chatFooter}>
            Powered by OpenAI GPT-4o-mini & RAG
          </div>
        </div>
      )}
    </>
  );
}
