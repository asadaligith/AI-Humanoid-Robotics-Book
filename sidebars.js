/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main tutorial sidebar
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '🏠 Getting Started',
    },
    'preface',
    'how-to-use',
    'hardware-options',
    'conventions',
    {
      type: 'category',
      label: '🚀 Getting Started',
      collapsed: false,
      items: [
        'getting-started/prerequisites',
        'getting-started/installation',
      ],
    },
    {
      type: 'category',
      label: '📚 Course Modules',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: '📘 Module 01: ROS 2 Fundamentals',
          collapsed: true,
          link: {
            type: 'doc',
            id: 'modules/module-01-ros2-fundamentals/index',
          },
          items: [
            {
              type: 'doc',
              id: 'modules/module-01-ros2-fundamentals/chapter-01-introduction',
              label: '1️⃣ Introduction & Installation',
            },
            {
              type: 'doc',
              id: 'modules/module-01-ros2-fundamentals/chapter-02-pubsub',
              label: '2️⃣ Publisher-Subscriber Pattern',
            },
            {
              type: 'doc',
              id: 'modules/module-01-ros2-fundamentals/chapter-03-packages',
              label: '3️⃣ ROS 2 Packages',
            },
            {
              type: 'doc',
              id: 'modules/module-01-ros2-fundamentals/chapter-04-services',
              label: '4️⃣ Services',
            },
            {
              type: 'doc',
              id: 'modules/module-01-ros2-fundamentals/chapter-05-actions',
              label: '5️⃣ Actions',
            },
            {
              type: 'doc',
              id: 'modules/module-01-ros2-fundamentals/chapter-06-urdf-tf',
              label: '6️⃣ URDF & TF2',
            },
            {
              type: 'doc',
              id: 'modules/module-01-ros2-fundamentals/chapter-07-summary',
              label: '7️⃣ Module Summary',
            },
          ],
        },
        {
          type: 'category',
          label: '📗 Module 02: Digital Twin - Gazebo & Unity',
          collapsed: true,
          link: {
            type: 'doc',
            id: 'modules/module-02-digital-twin-gazebo-unity/index',
          },
          items: [
            {
              type: 'doc',
              id: 'modules/module-02-digital-twin-gazebo-unity/chapter-01-introduction',
              label: '1️⃣ Introduction to Digital Twins',
            },
            {
              type: 'doc',
              id: 'modules/module-02-digital-twin-gazebo-unity/chapter-02-gazebo-basics',
              label: '2️⃣ Gazebo Basics',
            },
            {
              type: 'doc',
              id: 'modules/module-02-digital-twin-gazebo-unity/chapter-03-urdf-gazebo',
              label: '3️⃣ URDF in Gazebo',
            },
            {
              type: 'doc',
              id: 'modules/module-02-digital-twin-gazebo-unity/chapter-04-sensors',
              label: '4️⃣ Virtual Sensors',
            },
            {
              type: 'doc',
              id: 'modules/module-02-digital-twin-gazebo-unity/chapter-05-unity-ros',
              label: '5️⃣ Unity-ROS Integration',
            },
            {
              type: 'doc',
              id: 'modules/module-02-digital-twin-gazebo-unity/chapter-06-physics-tuning',
              label: '6️⃣ Physics Tuning',
            },
            {
              type: 'doc',
              id: 'modules/module-02-digital-twin-gazebo-unity/chapter-07-summary',
              label: '7️⃣ Module Summary',
            },
          ],
        },
        {
          type: 'category',
          label: '📙 Module 03: AI-Robot Brain - Isaac/Nav2',
          collapsed: true,
          link: {
            type: 'doc',
            id: 'modules/module-03-ai-robot-brain-isaac/index',
          },
          items: [
            {
              type: 'doc',
              id: 'modules/module-03-ai-robot-brain-isaac/chapter-01-introduction',
              label: '1️⃣ Introduction to AI Perception',
            },
            {
              type: 'doc',
              id: 'modules/module-03-ai-robot-brain-isaac/chapter-02-isaac-setup',
              label: '2️⃣ Isaac Sim Setup',
            },
            {
              type: 'doc',
              id: 'modules/module-03-ai-robot-brain-isaac/chapter-03-synthetic-data',
              label: '3️⃣ Synthetic Data Generation',
            },
            {
              type: 'doc',
              id: 'modules/module-03-ai-robot-brain-isaac/chapter-04-vslam',
              label: '4️⃣ Visual SLAM',
            },
            {
              type: 'doc',
              id: 'modules/module-03-ai-robot-brain-isaac/chapter-05-perception',
              label: '5️⃣ Perception Pipeline',
            },
            {
              type: 'doc',
              id: 'modules/module-03-ai-robot-brain-isaac/chapter-06-nav2',
              label: '6️⃣ Nav2 Navigation',
            },
            {
              type: 'doc',
              id: 'modules/module-03-ai-robot-brain-isaac/chapter-07-summary',
              label: '7️⃣ Module Summary',
            },
          ],
        },
        {
          type: 'category',
          label: '📕 Module 04: Vision-Language-Action',
          collapsed: true,
          link: {
            type: 'doc',
            id: 'modules/module-04-vla-vision-language-action/index',
          },
          items: [
            {
              type: 'doc',
              id: 'modules/module-04-vla-vision-language-action/chapter-01-introduction',
              label: '1️⃣ Introduction to VLA',
            },
            {
              type: 'doc',
              id: 'modules/module-04-vla-vision-language-action/chapter-02-whisper',
              label: '2️⃣ Speech Recognition (Whisper)',
            },
            {
              type: 'doc',
              id: 'modules/module-04-vla-vision-language-action/chapter-03-llm-planning',
              label: '3️⃣ LLM Task Planning',
            },
            {
              type: 'doc',
              id: 'modules/module-04-vla-vision-language-action/chapter-04-ros-executor',
              label: '4️⃣ ROS Action Executor',
            },
            {
              type: 'doc',
              id: 'modules/module-04-vla-vision-language-action/chapter-05-integration',
              label: '5️⃣ End-to-End Integration',
            },
            {
              type: 'doc',
              id: 'modules/module-04-vla-vision-language-action/chapter-06-jetson-deploy',
              label: '6️⃣ Jetson Deployment',
            },
            {
              type: 'doc',
              id: 'modules/module-04-vla-vision-language-action/chapter-07-summary',
              label: '7️⃣ Module Summary',
            },
          ],
        },
        {
          type: 'category',
          label: '🤖 Module 05: Autonomous Humanoid Capstone',
          collapsed: false,
          link: {
            type: 'doc',
            id: 'modules/module-05-capstone/index',
          },
          items: [
            {
              type: 'doc',
              id: 'modules/module-05-capstone/chapter-01-architecture',
              label: '1️⃣ System Architecture',
            },
            {
              type: 'doc',
              id: 'modules/module-05-capstone/chapter-02-voice-llm',
              label: '2️⃣ Voice & LLM Integration',
            },
            {
              type: 'doc',
              id: 'modules/module-05-capstone/chapter-03-navigation-perception',
              label: '3️⃣ Navigation & Perception',
            },
            {
              type: 'doc',
              id: 'modules/module-05-capstone/chapter-04-manipulation',
              label: '4️⃣ Manipulation',
            },
            {
              type: 'doc',
              id: 'modules/module-05-capstone/chapter-05-simulation-deployment',
              label: '5️⃣ Simulation & Deployment',
            },
            {
              type: 'doc',
              id: 'modules/module-05-capstone/chapter-06-jetson-deployment',
              label: '6️⃣ Jetson Hardware Deployment',
            },
            {
              type: 'category',
              label: '🧪 Testing & Validation',
              collapsed: true,
              items: [
                'modules/module-05-capstone/testing-methodology',
                'modules/module-05-capstone/benchmarking',
              ],
            },
            {
              type: 'doc',
              id: 'modules/module-05-capstone/troubleshooting',
              label: '🔧 Troubleshooting',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: '🔧 Troubleshooting',
      collapsed: true,
      items: [
        'troubleshooting/common-issues',
      ],
    },
    {
      type: 'category',
      label: '🤖 RAG Chatbot',
      collapsed: true,
      items: [
        'rag-chatbot/architecture',
        'rag-chatbot/setup',
        'rag-chatbot/deployment',
      ],
    },
    {
      type: 'category',
      label: '🔗 Integrations',
      collapsed: true,
      items: [
        'integrations/api-reference',
      ],
    },
    {
      type: 'category',
      label: '📖 Reference',
      collapsed: true,
      items: [
        'quality-gates',
        'review-process',
      ],
    },
  ],
};

module.exports = sidebars;
