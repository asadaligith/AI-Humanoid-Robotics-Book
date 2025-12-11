import React from 'react';
import styles from './styles.module.css';

/**
 * ChatMessage Component
 * Displays a single message in the chat (user or assistant)
 */
export default function ChatMessage({ message }) {
  const isUser = message.role === 'user';

  return (
    <div className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}>
      <div className={styles.messageHeader}>
        <strong>{isUser ? 'You' : 'AI Assistant'}</strong>
      </div>
      <div className={styles.messageContent}>
        {message.content}
      </div>
      {/* Display source citations if available (from assistant) */}
      {!isUser && message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <div className={styles.sourcesHeader}>Sources:</div>
          <ul className={styles.sourcesList}>
            {message.sources.map((source, idx) => {
              // Safety checks to prevent crashes
              const filePath = source?.file_path || 'Unknown';
              const sectionHeading = source?.section_heading || filePath;
              const score = source?.score || 0;
              const docPath = filePath.replace('.md', '');

              return (
                <li key={idx} className={styles.sourceItem}>
                  <a
                    href={`/docs${docPath}`}
                    target="_blank"
                    rel="noopener noreferrer"
                    className={styles.sourceLink}
                  >
                    {sectionHeading}
                  </a>
                  <span className={styles.sourceScore}>
                    {' '}(relevance: {Math.round(score * 100)}%)
                  </span>
                </li>
              );
            })}
          </ul>
        </div>
      )}
    </div>
  );
}
