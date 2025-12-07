#!/usr/bin/env python3
"""Debug word counting for testing-methodology.md"""

from pathlib import Path
import re

def debug_count(file_path: Path):
    content = file_path.read_text(encoding='utf-8')

    print(f"0. Original: {len(content.split())} words\n")

    # Step 1: Remove frontmatter
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL | re.MULTILINE)
    print(f"1. After frontmatter removal: {len(content.split())} words")

    # Step 2: Remove code blocks
    content = re.sub(r'```[^`]*```', '', content, flags=re.DOTALL)
    print(f"2. After code block removal: {len(content.split())} words")

    # Step 3: Remove inline code
    content = re.sub(r'(?<!`)`(?!`)[^`\n]+`(?!`)', '', content)
    print(f"3. After inline code removal: {len(content.split())} words")

    # Step 4: Remove URLs
    content = re.sub(r'https?://\S+', '', content)
    print(f"4. After URL removal: {len(content.split())} words")

    # Step 5: Remove markdown link syntax but keep text
    content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)
    print(f"5. After link syntax removal: {len(content.split())} words")

    # Step 6: Remove markdown image syntax
    content = re.sub(r'!\[([^\]]*)\]\([^\)]+\)', '', content)
    print(f"6. After image removal: {len(content.split())} words")

    # Step 7: Remove markdown headings symbols
    content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)
    print(f"7. After heading symbols removal: {len(content.split())} words")

    # Step 8: Remove markdown emphasis markers
    content = re.sub(r'\*\*([^\*]+)\*\*', r'\1', content)  # Bold
    print(f"8a. After **bold** removal: {len(content.split())} words")

    content = re.sub(r'__([^_]+)__', r'\1', content)       # Bold alt
    print(f"8b. After __bold__ removal: {len(content.split())} words")

    content = re.sub(r'\*([^\*]+)\*', r'\1', content)      # Italic
    print(f"8c. After *italic* removal: {len(content.split())} words")

    content = re.sub(r'_([^_]+)_', r'\1', content)         # Italic alt
    print(f"8d. After _italic_ removal: {len(content.split())} words")

    content = re.sub(r'~~([^~]+)~~', r'\1', content)       # Strikethrough
    print(f"8e. After ~~strike~~ removal: {len(content.split())} words")

    # Step 9: Remove HTML tags
    content = re.sub(r'<[^>]+>', '', content)
    print(f"9. After HTML tag removal: {len(content.split())} words")

    # Step 10: Remove horizontal rules
    content = re.sub(r'^[\-\*]{3,}$', '', content, flags=re.MULTILINE)
    print(f"10. After horizontal rule removal: {len(content.split())} words")

    # Step 11: Remove list markers
    content = re.sub(r'^\s*[-\*\+]\s+', '', content, flags=re.MULTILINE)
    print(f"11a. After bullet list removal: {len(content.split())} words")

    content = re.sub(r'^\s*\d+\.\s+', '', content, flags=re.MULTILINE)
    print(f"11b. After numbered list removal: {len(content.split())} words")

    # Step 12: Remove table formatting
    content = re.sub(r'\|', ' ', content)
    print(f"12. After table pipe removal: {len(content.split())} words")

    print(f"\nFinal word count: {len([w for w in content.split() if w.strip()])}")

if __name__ == '__main__':
    base_dir = Path(__file__).parent.parent.parent
    test_file = base_dir / 'docs' / 'modules' / 'module-05-capstone' / 'testing-methodology.md'
    debug_count(test_file)
