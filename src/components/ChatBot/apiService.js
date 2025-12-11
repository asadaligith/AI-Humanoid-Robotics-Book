/**
 * API Service for communicating with the RAG Chatbot backend
 */

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://rag-chatbot-api.onrender.com'  // Update with your actual Render URL after deployment
  : 'http://localhost:8000';

/**
 * Ask a question to the chatbot
 * @param {string} question - The user's question
 * @param {Array} conversationHistory - Previous conversation messages
 * @returns {Promise<Object>} The chatbot response
 */
export async function askQuestion(question, conversationHistory = []) {
  try {
    const response = await fetch(`${API_BASE_URL}/api/ask`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        question,
        conversation_history: conversationHistory,
      }),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error asking question:', error);
    throw error;
  }
}

/**
 * Check backend health status
 * @returns {Promise<Object>} Health status response
 */
export async function checkHealth() {
  try {
    const response = await fetch(`${API_BASE_URL}/health`);
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    return await response.json();
  } catch (error) {
    console.error('Error checking health:', error);
    throw error;
  }
}
