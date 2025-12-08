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
      status: 'Coming Soon',
    },
    {
      title: 'Module 02: Gazebo Simulation',
      description: 'Physics simulation, URDF modeling, and sensor integration',
      status: 'Coming Soon',
    },
    {
      title: 'Module 03: Computer Vision & Perception',
      description: 'Camera integration, YOLO object detection, and depth processing',
      status: 'Coming Soon',
    },
    {
      title: 'Module 04: Navigation & Manipulation',
      description: 'Nav2 stack, MoveIt 2, inverse kinematics, and motion planning',
      status: 'Coming Soon',
    },
    {
      title: 'Module 05: Autonomous Humanoid Capstone',
      description: 'Voice-controlled butler robot with LLM, navigation, perception, and manipulation',
      status: '✅ Available Now',
      link: '/docs/modules/module-05-capstone/',
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
                  <span className={clsx('badge', module.status === '✅ Available Now' ? 'badge--success' : 'badge--secondary')}>
                    {module.status}
                  </span>
                </div>
                <div className="card__body">
                  <p>{module.description}</p>
                </div>
                {module.link && (
                  <div className="card__footer">
                    <Link className="button button--primary button--block" to={module.link}>
                      Start Module →
                    </Link>
                  </div>
                )}
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
