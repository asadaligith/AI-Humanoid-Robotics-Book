# Diagram Conversion Scripts

This directory contains scripts for converting Mermaid diagram source files (`.mmd`) to PNG images for documentation.

## Quick Start

### Option 1: Automated Script (Recommended)

**Prerequisites**: Install Mermaid CLI
```bash
npm install -g @mermaid-js/mermaid-cli
```

**Convert all diagrams**:
```bash
# Linux/Mac
./convert_mermaid_to_png.sh

# Windows PowerShell
.\convert_mermaid_to_png.ps1
```

### Option 2: Docker (No Installation Required)

```bash
# Navigate to diagram directory
cd assets/diagrams/architecture

# Convert individual diagram
docker run --rm -v ${PWD}:/data minlag/mermaid-cli \
  -i /data/nominal-flow.mmd \
  -o /data/nominal-flow.png \
  -b transparent \
  -t dark \
  -w 1200

# Repeat for all 3 diagrams
```

### Option 3: Online Editor (Manual)

1. Open [Mermaid Live Editor](https://mermaid.live/)
2. Copy contents of `.mmd` file
3. Paste into editor
4. Click "Actions" → "PNG" to download
5. Save as `<diagram-name>.png` in `assets/diagrams/architecture/`

**Diagrams to convert**:
- `nominal-flow.mmd` → `nominal-flow.png`
- `navigation-failure.mmd` → `navigation-failure.png`
- `object-not-found.mmd` → `object-not-found.png`

## Diagram Specifications

All diagrams are generated with:
- **Format**: PNG
- **Background**: Transparent
- **Theme**: Dark
- **Width**: 1200px
- **Resolution**: High DPI (300 DPI for print quality)

## Diagram Inventory

### Sequence Diagrams (3 diagrams)

1. **nominal-flow.mmd** - Successful fetch-and-deliver task
   - User → Voice → LLM → Integration → Navigation → Perception → Manipulation
   - Shows complete task execution flow
   - Expected duration: ~60 seconds

2. **navigation-failure.mmd** - Navigation failure with retry logic
   - Demonstrates 3 retry attempts with costmap clearing
   - Shows transition to LLM replanning after failures
   - Illustrates edge case handling

3. **object-not-found.mmd** - Perception failure with clarification
   - Object detection fails after 3 retries
   - System requests user clarification
   - Shows human-in-the-loop recovery

## Verification

After conversion, verify PNG files exist:

```bash
ls -lh assets/diagrams/architecture/*.png
```

Expected output:
```
nominal-flow.png         (~150 KB)
navigation-failure.png   (~120 KB)
object-not-found.png     (~100 KB)
```

## Troubleshooting

### Error: "mmdc: command not found"

**Solution**: Install Mermaid CLI
```bash
npm install -g @mermaid-js/mermaid-cli
```

### Error: "Puppeteer error: Could not find Chrome"

**Solution**: Install Chromium
```bash
# Ubuntu/Debian
sudo apt-get install chromium-browser

# macOS
brew install chromium

# Or let Puppeteer download it
npx puppeteer browsers install chrome
```

### Error: "EACCES: permission denied"

**Solution**: Run with appropriate permissions
```bash
# Linux/Mac
chmod +x convert_mermaid_to_png.sh
sudo ./convert_mermaid_to_png.sh

# Windows PowerShell (Run as Administrator)
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
.\convert_mermaid_to_png.ps1
```

### Generated PNG is blank/corrupted

**Solution**: Check Mermaid syntax
```bash
# Validate Mermaid syntax
mmdc -i assets/diagrams/architecture/nominal-flow.mmd --validate

# If validation fails, open in Mermaid Live Editor to debug
```

## Alternative Tools

### 1. VS Code Extension

Install **Mermaid Preview** extension:
1. Open `.mmd` file in VS Code
2. Press `Ctrl+Shift+P` (Windows) or `Cmd+Shift+P` (Mac)
3. Select "Mermaid: Export Diagram"
4. Choose PNG format

### 2. Python Script

```python
# install: pip install mermaid-py
from mermaid import Mermaid

mermaid = Mermaid()
mermaid.render_from_file(
    'assets/diagrams/architecture/nominal-flow.mmd',
    'assets/diagrams/architecture/nominal-flow.png'
)
```

### 3. GitHub Actions (Automated CI/CD)

```yaml
# .github/workflows/diagrams.yml
name: Generate Diagrams
on: [push]
jobs:
  generate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
      - run: npm install -g @mermaid-js/mermaid-cli
      - run: ./scripts/diagrams/convert_mermaid_to_png.sh
      - uses: actions/upload-artifact@v3
        with:
          name: diagrams
          path: assets/diagrams/architecture/*.png
```

## Integration with Documentation

Once PNG files are generated, they are automatically referenced in documentation:

- `chapter-01-architecture.md` → Links to all 3 diagrams
- `state-machine.md` → References execution flow diagrams

Docusaurus will render PNG images with:
```markdown
![Nominal Flow](../../../assets/diagrams/architecture/nominal-flow.png)
```

## Maintenance

**When to regenerate diagrams**:
- After modifying `.mmd` source files
- When updating state machine logic
- Before deploying to production
- As part of release process

**Automated regeneration** (recommended):
Add to pre-commit hook:
```bash
# .git/hooks/pre-commit
#!/bin/bash
./scripts/diagrams/convert_mermaid_to_png.sh
git add assets/diagrams/architecture/*.png
```

## References

- [Mermaid Documentation](https://mermaid.js.org/)
- [Mermaid CLI GitHub](https://github.com/mermaid-js/mermaid-cli)
- [Mermaid Live Editor](https://mermaid.live/)
- [Docusaurus Diagrams](https://docusaurus.io/docs/markdown-features/diagrams)
