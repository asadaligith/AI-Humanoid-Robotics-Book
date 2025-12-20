import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const features = [
    {
      title: '🤖 Hands-On Learning',
      description: (
        <>
          Build real autonomous humanoid robots using ROS 2, Gazebo, and NVIDIA Isaac Sim.
          From basics to advanced Physical AI deployment on Jetson hardware.
        </>
      ),
    },
    {
      title: '🎯 Industry-Ready Skills',
      description: (
        <>
          Master voice-controlled manipulation, navigation, computer vision, and LLM integration.
          Learn the exact workflows used in robotics companies worldwide.
        </>
      ),
    },
    {
      title: '🚀 Capstone Project',
      description: (
        <>
          Complete a production-grade autonomous butler robot with 60%+ success rate,
          voice commands, object detection, and pick-and-place manipulation.
        </>
      ),
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <div key={idx} className={clsx('col col--4')}>
              <div className="text--center padding-horiz--md">
                <h3>{feature.title}</h3>
                <p>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageModules() {
  const modules = [
    {
      title: 'Module 01: ROS 2 Fundamentals',
      description: 'Nodes, topics, services, actions, and core robotics concepts',
      status: '📝 Spec Ready',
      statusType: 'spec-ready',
      specLink: 'https://github.com/asadaligith/AI-Humanoid-Robotics-Book/tree/main/specs/001-ros2-nervous-system',
      chapters: 7,
      progress: 0,
    },
    {
      title: 'Module 02: Gazebo Simulation',
      description: 'Physics simulation, URDF modeling, and sensor integration',
      status: '📝 Spec Ready',
      statusType: 'spec-ready',
      specLink: 'https://github.com/asadaligith/AI-Humanoid-Robotics-Book/tree/main/specs/002-digital-twin-gazebo-unity',
      chapters: 6,
      progress: 0,
    },
    {
      title: 'Module 03: Computer Vision & Perception',
      description: 'Camera integration, YOLO object detection, and depth processing',
      status: '📝 Spec Ready',
      statusType: 'spec-ready',
      specLink: 'https://github.com/asadaligith/AI-Humanoid-Robotics-Book/tree/main/specs/003-ai-robot-brain-isaac',
      chapters: 5,
      progress: 0,
    },
    {
      title: 'Module 04: Navigation & Manipulation',
      description: 'Nav2 stack, MoveIt 2, inverse kinematics, and motion planning',
      status: '📝 Spec Ready',
      statusType: 'spec-ready',
      specLink: 'https://github.com/asadaligith/AI-Humanoid-Robotics-Book/tree/main/specs/004-vla-vision-language-action',
      chapters: 4,
      progress: 0,
    },
    {
      title: 'Module 05: Autonomous Humanoid Capstone',
      description: 'Voice-controlled butler robot with LLM, navigation, perception, and manipulation',
      status: '✅ Available Now',
      statusType: 'complete',
      link: '/docs/modules/module-05-capstone/',
      chapters: 10,
      progress: 100,
    },
  ];

  return (
    <section className={styles.modules}>
      <div className="container">
        <h2 className="text--center margin-bottom--lg">📚 Course Modules</h2>
        <div className="row">
          {modules.map((module, idx) => (
            <div key={idx} className="col col--12 margin-bottom--md">
              <div className={clsx('card', styles.moduleCard)}>
                <div className="card__header">
                  <h3>{module.title}</h3>
                  <span className={clsx('badge',
                    module.statusType === 'complete' ? 'badge--success' :
                    module.statusType === 'spec-ready' ? 'badge--primary' :
                    'badge--secondary')}>
                    {module.status}
                  </span>
                </div>
                <div className="card__body">
                  <p>{module.description}</p>
                  {module.chapters && (
                    <p className="text--sm" style={{marginTop: '8px', color: 'var(--ifm-color-emphasis-600)'}}>
                      📚 {module.chapters} chapters planned
                    </p>
                  )}
                  {module.progress !== undefined && (
                    <div style={{marginTop: '12px'}}>
                      <div style={{
                        width: '100%',
                        height: '6px',
                        backgroundColor: 'var(--ifm-color-emphasis-200)',
                        borderRadius: '3px',
                        overflow: 'hidden'
                      }}>
                        <div style={{
                          width: `${module.progress}%`,
                          height: '100%',
                          background: module.progress === 100
                            ? 'linear-gradient(90deg, var(--ifm-color-success-light), var(--ifm-color-success))'
                            : 'linear-gradient(90deg, var(--ifm-color-primary-light), var(--ifm-color-primary))',
                          transition: 'width 0.3s ease'
                        }} />
                      </div>
                      <p className="text--sm" style={{marginTop: '4px', textAlign: 'right', color: 'var(--ifm-color-emphasis-600)'}}>
                        {module.progress}% Complete
                      </p>
                    </div>
                  )}
                </div>
                <div className="card__footer">
                  {module.link ? (
                    <Link className="button button--primary button--block" to={module.link}>
                      Start Module →
                    </Link>
                  ) : module.specLink ? (
                    <>
                      <a
                        href={module.specLink}
                        className="button button--secondary button--block"
                        target="_blank"
                        rel="noopener noreferrer"
                        style={{marginBottom: '8px'}}>
                        📋 View Specification →
                      </a>
                      <div className="text--center text--sm" style={{color: 'var(--ifm-color-emphasis-600)'}}>
                        Documentation in development
                      </div>
                    </>
                  ) : null}
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Master Physical AI: From ROS 2 to Autonomous Humanoid Robots">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <HomepageModules />
      </main>
    </Layout>
  );
}
