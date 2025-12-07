// @ts-check
// Note: type annotations allow type checking and IDE autocompletion

const {themes} = require('prism-react-renderer');
const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI Humanoid Robotics Book',
  tagline: 'Master Physical AI: From ROS 2 to Autonomous Humanoid Robots',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://asadaligith.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/AI-Humanoid-Robotics-Book/',

  // GitHub pages deployment config.
  organizationName: 'asadaligith', // Usually your GitHub org/user name.
  projectName: 'AI-Humanoid-Robotics-Book', // Usually your repo name.
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          editUrl:
            'https://github.com/asadaligith/AI-Humanoid-Robotics-Book/tree/main/',
          remarkPlugins: [],
          rehypePlugins: [],
        },
        blog: false, // Disable blog for now
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'AI Humanoid Robotics',
        logo: {
          alt: 'AI Humanoid Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/asadaligith/AI-Humanoid-Robotics-Book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: ROS 2',
                to: '/docs/intro',
              },
              {
                label: 'Module 5: Capstone',
                to: '/docs/modules/module-05-capstone/',
              },
            ],
          },
          {
            title: 'Technologies',
            items: [
              {
                label: 'ROS 2',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'Gazebo',
                href: 'https://gazebosim.org/docs',
              },
              {
                label: 'Isaac Sim',
                href: 'https://developer.nvidia.com/isaac-sim',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/asadaligith/AI-Humanoid-Robotics-Book',
              },
              {
                label: 'GIAIC',
                href: 'https://www.piaic.org/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} GIAIC Hackathon Q4. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'yaml'],
      },
    }),
};

module.exports = config;
