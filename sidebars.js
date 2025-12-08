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
      label: '📚 Course Modules',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: '📘 Module 01: ROS 2 Fundamentals',
          collapsed: true,
          link: {
            type: 'generated-index',
            title: 'Module 01: ROS 2 Fundamentals',
            description: 'Learn the core concepts of ROS 2: nodes, topics, services, actions, and parameters. Build your first robot applications.',
            slug: '/modules/module-01',
          },
          items: [
            // Content coming soon
          ],
        },
        {
          type: 'category',
          label: '📗 Module 02: Gazebo Simulation',
          collapsed: true,
          link: {
            type: 'generated-index',
            title: 'Module 02: Gazebo Simulation',
            description: 'Master physics simulation, URDF modeling, sensor integration, and world building with Gazebo Fortress.',
            slug: '/modules/module-02',
          },
          items: [
            // Content coming soon
          ],
        },
        {
          type: 'category',
          label: '📙 Module 03: Computer Vision & Perception',
          collapsed: true,
          link: {
            type: 'generated-index',
            title: 'Module 03: Computer Vision & Perception',
            description: 'Implement camera integration, YOLO object detection, depth processing, and point cloud analysis.',
            slug: '/modules/module-03',
          },
          items: [
            // Content coming soon
          ],
        },
        {
          type: 'category',
          label: '📕 Module 04: Navigation & Manipulation',
          collapsed: true,
          link: {
            type: 'generated-index',
            title: 'Module 04: Navigation & Manipulation',
            description: 'Build autonomous navigation with Nav2, and robotic manipulation with MoveIt 2, inverse kinematics, and motion planning.',
            slug: '/modules/module-04',
          },
          items: [
            // Content coming soon
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
      label: '📖 References',
      collapsed: true,
      items: [
        // Appendices and references will be added here
      ],
    },
  ],
};

module.exports = sidebars;
