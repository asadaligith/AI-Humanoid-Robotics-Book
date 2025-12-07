import React from 'react';
import styles from './ChapterTemplate.module.css';

/**
 * ChapterTemplate - Base template for all module chapters
 *
 * Provides consistent structure and styling for educational content including:
 * - Learning objectives
 * - Prerequisites
 * - Content sections
 * - Checkpoints
 * - Exercises
 * - Summary
 *
 * @example
 * ```tsx
 * import ChapterTemplate from '@site/src/components/ChapterTemplate';
 *
 * <ChapterTemplate
 *   moduleNumber={1}
 *   chapterNumber={3}
 *   title="Creating Your First ROS 2 Node"
 *   objectives={[
 *     "Understand the structure of a ROS 2 Python node",
 *     "Implement a simple publisher-subscriber pattern",
 *     "Build and run nodes using colcon"
 *   ]}
 *   prerequisites={[
 *     "ROS 2 Humble installed",
 *     "Understanding of Python classes"
 *   ]}
 *   estimatedTime="45 minutes"
 * >
 *   {children}
 * </ChapterTemplate>
 * ```
 */

export interface ChapterTemplateProps {
  /** Module number (1-5) */
  moduleNumber: number;

  /** Chapter number within module */
  chapterNumber: number;

  /** Chapter title */
  title: string;

  /** Learning objectives (bullet points) */
  objectives: string[];

  /** Prerequisites (bullet points) */
  prerequisites: string[];

  /** Estimated completion time */
  estimatedTime: string;

  /** Difficulty level */
  difficulty?: 'beginner' | 'intermediate' | 'advanced';

  /** Chapter content */
  children: React.ReactNode;

  /** Optional checkpoint sections */
  checkpoints?: CheckpointSection[];

  /** Optional exercises */
  exercises?: Exercise[];
}

export interface CheckpointSection {
  id: string;
  title: string;
  description: string;
  verificationSteps: string[];
  expectedOutput?: string;
}

export interface Exercise {
  id: string;
  title: string;
  difficulty: 'easy' | 'medium' | 'hard';
  description: string;
  hints?: string[];
  solution?: string;
}

const ChapterTemplate: React.FC<ChapterTemplateProps> = ({
  moduleNumber,
  chapterNumber,
  title,
  objectives,
  prerequisites,
  estimatedTime,
  difficulty = 'intermediate',
  children,
  checkpoints = [],
  exercises = []
}) => {
  const difficultyColors = {
    beginner: '#28a745',
    intermediate: '#ffc107',
    advanced: '#dc3545'
  };

  return (
    <div className={styles.chapterTemplate}>
      {/* Chapter Header */}
      <div className={styles.chapterHeader}>
        <div className={styles.chapterMeta}>
          <span className={styles.moduleTag}>Module {moduleNumber}</span>
          <span className={styles.chapterTag}>Chapter {chapterNumber}</span>
          <span
            className={styles.difficultyBadge}
            style={{ backgroundColor: difficultyColors[difficulty] }}
          >
            {difficulty.charAt(0).toUpperCase() + difficulty.slice(1)}
          </span>
        </div>
        <h1 className={styles.chapterTitle}>{title}</h1>
        <div className={styles.timeEstimate}>
          ⏱️ Estimated time: {estimatedTime}
        </div>
      </div>

      {/* Learning Objectives */}
      <section className={styles.objectivesSection}>
        <h2>📚 Learning Objectives</h2>
        <p>By the end of this chapter, you will be able to:</p>
        <ul className={styles.objectivesList}>
          {objectives.map((objective, index) => (
            <li key={index}>{objective}</li>
          ))}
        </ul>
      </section>

      {/* Prerequisites */}
      <section className={styles.prerequisitesSection}>
        <h2>✅ Prerequisites</h2>
        <p>Before starting, ensure you have:</p>
        <ul className={styles.prerequisitesList}>
          {prerequisites.map((prereq, index) => (
            <li key={index}>{prereq}</li>
          ))}
        </ul>
      </section>

      {/* Main Content */}
      <div className={styles.mainContent}>
        {children}
      </div>

      {/* Checkpoints */}
      {checkpoints.length > 0 && (
        <section className={styles.checkpointsSection}>
          <h2>🎯 Checkpoints</h2>
          {checkpoints.map((checkpoint, index) => (
            <div key={checkpoint.id} className={styles.checkpoint}>
              <h3>
                ✅ Checkpoint {moduleNumber}.{chapterNumber}.{index + 1}: {checkpoint.title}
              </h3>
              <p>{checkpoint.description}</p>

              <h4>Verification Steps:</h4>
              <ol>
                {checkpoint.verificationSteps.map((step, stepIndex) => (
                  <li key={stepIndex}>{step}</li>
                ))}
              </ol>

              {checkpoint.expectedOutput && (
                <div className={styles.expectedOutput}>
                  <h4>Expected Output:</h4>
                  <pre>
                    <code>{checkpoint.expectedOutput}</code>
                  </pre>
                </div>
              )}
            </div>
          ))}
        </section>
      )}

      {/* Exercises */}
      {exercises.length > 0 && (
        <section className={styles.exercisesSection}>
          <h2>💪 Practice Exercises</h2>
          {exercises.map((exercise, index) => (
            <div key={exercise.id} className={styles.exercise}>
              <div className={styles.exerciseHeader}>
                <h3>Exercise {index + 1}: {exercise.title}</h3>
                <span className={`${styles.exerciseDifficulty} ${styles[exercise.difficulty]}`}>
                  {exercise.difficulty}
                </span>
              </div>
              <p>{exercise.description}</p>

              {exercise.hints && exercise.hints.length > 0 && (
                <details className={styles.hints}>
                  <summary>💡 Hints</summary>
                  <ul>
                    {exercise.hints.map((hint, hintIndex) => (
                      <li key={hintIndex}>{hint}</li>
                    ))}
                  </ul>
                </details>
              )}

              {exercise.solution && (
                <details className={styles.solution}>
                  <summary>🔓 Solution</summary>
                  <div className={styles.solutionContent}>
                    <p>{exercise.solution}</p>
                  </div>
                </details>
              )}
            </div>
          ))}
        </section>
      )}

      {/* Chapter Navigation */}
      <div className={styles.chapterNavigation}>
        <div className={styles.navButtons}>
          <button className={styles.prevButton}>← Previous Chapter</button>
          <button className={styles.nextButton}>Next Chapter →</button>
        </div>
      </div>
    </div>
  );
};

export default ChapterTemplate;
