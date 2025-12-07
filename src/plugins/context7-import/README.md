# Context7 Import Plugin

Custom Docusaurus plugin for importing and injecting Context7 citation metadata into the book build process.

## Features

- **Automatic Citation Loading**: Reads `.context7/metadata/citations.json` during build
- **Global Citation Access**: Makes citations available to all React components via `useGlobalData`
- **APA Formatting**: Integrates with `Context7Citation` component for proper citation rendering
- **Build-Time Processing**: No runtime dependencies for citation lookups

## Usage

### 1. Configure in `docusaurus.config.js`

```javascript
module.exports = {
  plugins: [
    require.resolve('./src/plugins/context7-import'),
  ],
};
```

### 2. Add Citations in Markdown

```markdown
ROS 2 uses DDS for communication [Context7: CTX7-001].
```

### 3. Use Context7Citation Component

```mdx
import Context7Citation from '@site/src/components/Context7Citation';

<Context7Citation id="CTX7-001" />
```

## Citation Metadata Format

Expected structure in `.context7/metadata/citations.json`:

```json
{
  "CTX7-001": {
    "id": "CTX7-001",
    "type": "research_paper",
    "title": "Exploring the performance of ROS2",
    "authors": ["Maruyama, Y.", "Kato, S.", "Azumi, T."],
    "year": 2016,
    "journal": "Proceedings of the 13th International Conference on Embedded Software",
    "pages": "1-10",
    "doi": "10.1145/2968478.2968502",
    "url": "https://doi.org/10.1145/2968478.2968502"
  }
}
```

## Integration with Citation Script

This plugin works with `scripts/citations/context7_to_apa.py` which:

1. Reads citations from Context7 metadata
2. Converts to APA 7th Edition format
3. Generates `docs/appendices/citations.md` with full bibliography

## Development

### Adding New Citation Fields

Edit `index.js` and update the `loadContent()` function to parse additional metadata fields.

### Testing

Run Docusaurus build to verify plugin loads correctly:

```bash
npm run build
```

Check console output for:
```
[Context7] Loaded X citations
```

## Troubleshooting

**Issue**: "No citations.json found"
**Solution**: Ensure `.context7/metadata/citations.json` exists and contains valid JSON

**Issue**: Citations not rendering
**Solution**: Verify Context7Citation component is imported and ID matches metadata

**Issue**: Build fails with plugin error
**Solution**: Check Node.js version (18+) and Docusaurus compatibility (3.x)

## License

MIT License - See repository root for full license text.
