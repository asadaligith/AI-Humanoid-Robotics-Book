import React, { useState } from 'react';
import styles from './Context7Citation.module.css';

/**
 * Context7Citation - React component for displaying Context7-linked citations
 *
 * Renders citations in APA 7th edition format with interactive tooltips showing
 * full bibliographic information. Citations link to Context7 metadata for
 * version tracking and deeper exploration.
 *
 * @example
 * ```tsx
 * import Context7Citation from '@site/src/components/Context7Citation';
 *
 * // Inline citation
 * <p>
 *   ROS 2 uses DDS for communication <Context7Citation id="CTX7-ROS2-001" />.
 * </p>
 *
 * // With custom text
 * <Context7Citation
 *   id="CTX7-GAZEBO-003"
 *   text="Open Robotics (2023)"
 * />
 *
 * // Multiple citations
 * <p>
 *   Several frameworks support robotics simulation{' '}
 *   <Context7Citation ids={["CTX7-GAZEBO-003", "CTX7-ISAAC-012"]} />.
 * </p>
 * ```
 */

export interface Citation {
  id: string;
  type: 'research_paper' | 'documentation' | 'vendor_guide' | 'book';
  authors: string[];
  year: number;
  title: string;
  url: string;
  doi?: string;
  publisher?: string;
  journal?: string;
  module?: string;
}

export interface Context7CitationProps {
  /** Single citation ID (e.g., "CTX7-ROS2-001") */
  id?: string;

  /** Multiple citation IDs for grouped citations */
  ids?: string[];

  /** Custom display text (overrides default "[ID]" format) */
  text?: string;

  /** Show full citation inline instead of just link */
  inline?: boolean;

  /** Custom CSS class */
  className?: string;
}

/**
 * Citation database - loaded from Context7 metadata
 * In production, this would be imported from .context7/metadata/citations.json
 * For now, using mock data for demonstration
 */
const citationDatabase: Record<string, Citation> = {
  'CTX7-ROS2-001': {
    id: 'CTX7-ROS2-001',
    type: 'documentation',
    authors: ['Open Robotics'],
    year: 2023,
    title: 'ROS 2 Documentation: Humble',
    url: 'https://docs.ros.org/en/humble/',
    module: 'module-01'
  },
  'CTX7-GAZEBO-003': {
    id: 'CTX7-GAZEBO-003',
    type: 'documentation',
    authors: ['Open Robotics'],
    year: 2023,
    title: 'ROS 2 Integration with Gazebo',
    url: 'https://gazebosim.org/docs/fortress/ros2_integration',
    module: 'module-02'
  },
  'CTX7-ISAAC-012': {
    id: 'CTX7-ISAAC-012',
    type: 'documentation',
    authors: ['NVIDIA Corporation'],
    year: 2023,
    title: 'Isaac Sim Documentation',
    url: 'https://docs.omniverse.nvidia.com/isaacsim/latest/',
    module: 'module-02'
  },
  'CTX7-CLAUDE-001': {
    id: 'CTX7-CLAUDE-001',
    type: 'documentation',
    authors: ['Anthropic'],
    year: 2024,
    title: 'Claude API Documentation',
    url: 'https://docs.anthropic.com/',
    module: 'module-04'
  },
  'CTX7-WHISPER-001': {
    id: 'CTX7-WHISPER-001',
    type: 'research_paper',
    authors: ['Radford, A.', 'Kim, J. W.', 'Xu, T.', 'Brockman, G.', 'McLeavey, C.', 'Sutskever, I.'],
    year: 2022,
    title: 'Robust Speech Recognition via Large-Scale Weak Supervision',
    url: 'https://arxiv.org/abs/2212.04356',
    doi: '10.48550/arXiv.2212.04356',
    journal: 'arXiv preprint',
    module: 'module-04'
  }
};

/**
 * Format authors in APA style
 */
function formatAuthorsAPA(authors: string[]): string {
  if (authors.length === 1) {
    return authors[0];
  } else if (authors.length === 2) {
    return `${authors[0]}, & ${authors[1]}`;
  } else if (authors.length <= 19) {
    const allButLast = authors.slice(0, -1).join(', ');
    return `${allButLast}, & ${authors[authors.length - 1]}`;
  } else {
    // 20 or more authors: first 19, then ellipsis, then last
    const first19 = authors.slice(0, 19).join(', ');
    const last = authors[authors.length - 1];
    return `${first19}, ... ${last}`;
  }
}

/**
 * Format single citation in APA 7th edition
 */
function formatCitationAPA(citation: Citation): string {
  const authors = formatAuthorsAPA(citation.authors);
  const year = citation.year;
  const title = citation.title;

  let formatted = `${authors}. (${year}). ${title}.`;

  if (citation.journal) {
    formatted += ` ${citation.journal}.`;
  }

  if (citation.doi) {
    formatted += ` https://doi.org/${citation.doi}`;
  } else {
    formatted += ` ${citation.url}`;
  }

  return formatted;
}

const Context7Citation: React.FC<Context7CitationProps> = ({
  id,
  ids,
  text,
  inline = false,
  className = ''
}) => {
  const [tooltipVisible, setTooltipVisible] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });

  // Determine which citations to display
  const citationIds = ids || (id ? [id] : []);

  if (citationIds.length === 0) {
    console.warn('Context7Citation: No citation ID(s) provided');
    return <span className={styles.error}>[Citation Error]</span>;
  }

  // Fetch citation data
  const citations = citationIds
    .map(citId => citationDatabase[citId])
    .filter(Boolean);

  if (citations.length === 0) {
    console.warn(`Context7Citation: No citations found for IDs: ${citationIds.join(', ')}`);
    return <span className={styles.error}>[Citation Not Found]</span>;
  }

  // Handle mouse enter for tooltip
  const handleMouseEnter = (event: React.MouseEvent) => {
    const rect = event.currentTarget.getBoundingClientRect();
    setTooltipPosition({
      x: rect.left + window.scrollX,
      y: rect.bottom + window.scrollY + 5
    });
    setTooltipVisible(true);
  };

  const handleMouseLeave = () => {
    setTooltipVisible(false);
  };

  // Render inline full citation
  if (inline && citations.length === 1) {
    const citation = citations[0];
    return (
      <span className={`${styles.inlineCitation} ${className}`}>
        <strong>[{citation.id}]</strong> {formatCitationAPA(citation)}
      </span>
    );
  }

  // Render linked citation(s)
  const displayText = text || citations.map(c => `[${c.id}]`).join(', ');

  return (
    <>
      <span
        className={`${styles.citationLink} ${className}`}
        onMouseEnter={handleMouseEnter}
        onMouseLeave={handleMouseLeave}
      >
        <a
          href={`/docs/appendices/citations#${citations[0].id.toLowerCase()}`}
          className={styles.link}
          onClick={(e) => {
            // Open citation in new tab if external URL
            if (citations[0].url) {
              e.preventDefault();
              window.open(citations[0].url, '_blank', 'noopener,noreferrer');
            }
          }}
        >
          <strong>{displayText}</strong>
        </a>
      </span>

      {/* Tooltip */}
      {tooltipVisible && (
        <div
          className={styles.tooltip}
          style={{
            position: 'absolute',
            left: `${tooltipPosition.x}px`,
            top: `${tooltipPosition.y}px`,
            zIndex: 1000
          }}
        >
          {citations.map((citation, index) => (
            <div key={citation.id} className={styles.tooltipCitation}>
              {citations.length > 1 && (
                <div className={styles.tooltipId}>[{citation.id}]</div>
              )}
              <div className={styles.tooltipContent}>
                {formatCitationAPA(citation)}
              </div>
              {index < citations.length - 1 && <hr className={styles.tooltipDivider} />}
            </div>
          ))}
          <div className={styles.tooltipFooter}>
            Click to view full citation or open source
          </div>
        </div>
      )}
    </>
  );
};

export default Context7Citation;

/**
 * Helper component for rendering citation lists
 * Used in appendices/citations.md page
 */
export const CitationList: React.FC<{ moduleId?: string }> = ({ moduleId }) => {
  const citations = Object.values(citationDatabase).filter(
    c => !moduleId || c.module === moduleId
  );

  return (
    <div className={styles.citationList}>
      {citations.map(citation => (
        <div key={citation.id} className={styles.citationListItem} id={citation.id.toLowerCase()}>
          <strong>[{citation.id}]</strong> {formatCitationAPA(citation)}
        </div>
      ))}
    </div>
  );
};
