/**
 * Context7 Import Plugin for Docusaurus
 *
 * This plugin processes Context7 citation placeholders and injects
 * citation metadata into Docusaurus builds.
 *
 * Usage:
 * - Add [Context7: ID-12345] in markdown files
 * - Plugin converts to proper APA citations during build
 * - Citations pulled from .context7/metadata/citations.json
 */

const fs = require('fs');
const path = require('path');

function context7ImportPlugin(context, options) {
  return {
    name: 'docusaurus-plugin-context7-import',

    async loadContent() {
      // Load Context7 citation metadata
      const citationsPath = path.join(
        context.siteDir,
        '.context7',
        'metadata',
        'citations.json'
      );

      if (!fs.existsSync(citationsPath)) {
        console.warn('[Context7] No citations.json found. Skipping citation injection.');
        return {};
      }

      const citations = JSON.parse(fs.readFileSync(citationsPath, 'utf8'));
      console.log(`[Context7] Loaded ${Object.keys(citations).length} citations`);

      return { citations };
    },

    async contentLoaded({ content, actions }) {
      const { setGlobalData } = actions;

      // Make citations available globally to all components
      setGlobalData({
        citations: content.citations || {},
      });
    },

    configureWebpack(config, isServer) {
      return {
        resolve: {
          alias: {
            '@context7': path.resolve(context.siteDir, '.context7'),
          },
        },
      };
    },

    injectHtmlTags() {
      return {
        headTags: [
          {
            tagName: 'meta',
            attributes: {
              name: 'context7-version',
              content: '1.0.0',
            },
          },
        ],
      };
    },
  };
}

module.exports = context7ImportPlugin;
