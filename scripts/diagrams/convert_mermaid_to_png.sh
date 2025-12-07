#!/bin/bash
#
# Convert Mermaid Diagrams to PNG
# Requires: @mermaid-js/mermaid-cli (mmdc)
#
# Installation:
#   npm install -g @mermaid-js/mermaid-cli
#
# Usage:
#   ./convert_mermaid_to_png.sh
#
# Output:
#   PNG files created in same directory as .mmd files

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "🎨 Mermaid to PNG Converter"
echo "=========================="
echo ""

# Check if mmdc is installed
if ! command -v mmdc &> /dev/null; then
    echo -e "${RED}❌ Error: mmdc (Mermaid CLI) not found${NC}"
    echo ""
    echo "Install with:"
    echo "  npm install -g @mermaid-js/mermaid-cli"
    echo ""
    echo "Or use Docker:"
    echo "  docker run --rm -v \$(pwd):/data minlag/mermaid-cli -i /data/input.mmd -o /data/output.png"
    exit 1
fi

echo -e "${GREEN}✓ mmdc found: $(mmdc --version)${NC}"
echo ""

# Base directory
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIAGRAM_DIR="$BASE_DIR/assets/diagrams/architecture"

# Convert each Mermaid file
echo "Converting diagrams..."
echo ""

# Array of diagrams to convert
DIAGRAMS=(
    "nominal-flow"
    "navigation-failure"
    "object-not-found"
)

SUCCESS_COUNT=0
FAIL_COUNT=0

for diagram in "${DIAGRAMS[@]}"; do
    INPUT="$DIAGRAM_DIR/$diagram.mmd"
    OUTPUT="$DIAGRAM_DIR/$diagram.png"

    if [ ! -f "$INPUT" ]; then
        echo -e "${YELLOW}⚠  Warning: $INPUT not found, skipping${NC}"
        ((FAIL_COUNT++))
        continue
    fi

    echo -n "  Converting $diagram.mmd → $diagram.png ... "

    # Convert with mmdc
    if mmdc -i "$INPUT" -o "$OUTPUT" -b transparent -t dark -w 1200 2>/dev/null; then
        echo -e "${GREEN}✓${NC}"
        ((SUCCESS_COUNT++))
    else
        echo -e "${RED}✗${NC}"
        ((FAIL_COUNT++))
    fi
done

echo ""
echo "=========================="
echo "Summary:"
echo -e "  ${GREEN}✓ Success: $SUCCESS_COUNT${NC}"
if [ $FAIL_COUNT -gt 0 ]; then
    echo -e "  ${RED}✗ Failed: $FAIL_COUNT${NC}"
fi
echo ""

if [ $SUCCESS_COUNT -gt 0 ]; then
    echo "Output directory: $DIAGRAM_DIR"
    echo ""
    echo "Generated PNGs:"
    ls -lh "$DIAGRAM_DIR"/*.png 2>/dev/null || echo "  (no PNG files found)"
fi

exit 0
