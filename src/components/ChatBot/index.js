import React, { useState, useRef, useEffect } from 'react';
import ChatMessage from './ChatMessage';
import { askQuestion, askAboutSelectedText } from './apiService';
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
  const [sessionId, setSessionId] = useState(() => {
    // Load session ID from sessionStorage on mount
    if (typeof window !== 'undefined') {
      return sessionStorage.getItem('chatbot_session_id') || null;
    }
    return null;
  });
  const [selectedText, setSelectedText] = useState('');
  const [showSelectionButton, setShowSelectionButton] = useState(false);
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });
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

  // Text selection handler
  useEffect(() => {
    const handleSelection = (e) => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      // Check if selection is valid (10-5000 chars, outside chatbot)
      if (text.length >= 10 && text.length <= 5000) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Don't show button for text selected inside chatbot
        const chatWindow = document.querySelector(`.${styles.chatWindow}`);
        if (chatWindow && chatWindow.contains(range.commonAncestorContainer)) {
          setShowSelectionButton(false);
          return;
        }

        setSelectedText(text);
        setSelectionPosition({ x: rect.right, y: rect.bottom });
        setShowSelectionButton(true);
      } else {
        setShowSelectionButton(false);
        setSelectedText('');
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('selectionchange', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('selectionchange', handleSelection);
    };
  }, []);

  // Persist session ID to sessionStorage whenever it changes
  useEffect(() => {
    if (typeof window !== 'undefined' && sessionId) {
      sessionStorage.setItem('chatbot_session_id', sessionId);
    }
  }, [sessionId]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    setError(null);
  };

  const handleSubmit = async (e, isSelectedTextMode = false) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      role: 'user',
      content: isSelectedTextMode
        ? `[About selected text: "${selectedText.substring(0, 100)}..."] ${inputValue.trim()}`
        : inputValue.trim(),
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputValue.trim();
    const currentSelectedText = selectedText;
    setInputValue('');
    setIsLoading(true);
    setError(null);

    // Clear selection state if used
    if (isSelectedTextMode) {
      setSelectedText('');
      setShowSelectionButton(false);
    }

    try {
      let response;

      // Use appropriate endpoint based on mode
      if (isSelectedTextMode && currentSelectedText) {
        response = await askAboutSelectedText(currentInput, currentSelectedText, sessionId);
      } else {
        response = await askQuestion(currentInput, sessionId);
      }

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
    setSelectedText('');
    setShowSelectionButton(false);

    // Clear sessionStorage
    if (typeof window !== 'undefined') {
      sessionStorage.removeItem('chatbot_session_id');
    }
  };

  const handleAskAboutSelection = () => {
    if (!selectedText) return;

    // Open chat if closed
    if (!isOpen) {
      setIsOpen(true);
    }

    // Set placeholder question or let user type their own
    setInputValue('');

    // Show indicator that selection is active
    setMessages(prev => [
      ...prev,
      {
        role: 'system',
        content: `📝 Selected text: "${selectedText.substring(0, 150)}${selectedText.length > 150 ? '...' : ''}"\n\nAsk me a question about this selection!`,
      }
    ]);

    // Focus input
    setTimeout(() => {
      if (inputRef.current) {
        inputRef.current.focus();
      }
    }, 100);
  };

  return (
    <>
      {/* Selection Button */}
      {showSelectionButton && !isOpen && (
        <button
          className={styles.selectionButton}
          style={{
            position: 'fixed',
            left: `${selectionPosition.x}px`,
            top: `${selectionPosition.y + 5}px`,
          }}
          onClick={handleAskAboutSelection}
          aria-label="Ask about this text"
        >
          💡 Ask about this
        </button>
      )}

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
          <form className={styles.chatForm} onSubmit={(e) => handleSubmit(e, !!selectedText)}>
            <input
              ref={inputRef}
              type="text"
              className={styles.chatInput}
              placeholder={selectedText ? "Ask about the selected text..." : "Ask about robotics, ROS 2, Gazebo..."}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={isLoading}
            />
            {selectedText && (
              <button
                type="button"
                className={styles.clearSelectionButton}
                onClick={() => {
                  setSelectedText('');
                  setShowSelectionButton(false);
                }}
                title="Clear selection"
              >
                ✕
              </button>
            )}
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
