/**
 * API Service for communicating with the RAG Chatbot backend
 */

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://rag-chatbot-api-ilmt.onrender.com'  // Production backend URL
  : 'http://localhost:8000';

/**
 * Ask a question to the chatbot
 * @param {string} question - The user's question
 * @param {string} sessionId - Optional session ID for conversation tracking
 * @returns {Promise<Object>} The chatbot response
 */
export async function askQuestion(question, sessionId = null) {
  try {
    const requestBody = { question };
    if (sessionId) {
      requestBody.session_id = sessionId;
    }

    const response = await fetch(`${API_BASE_URL}/api/ask`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
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
