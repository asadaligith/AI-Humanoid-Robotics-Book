#!/usr/bin/env python3
"""
Word Count Validator for AI Humanoid Robotics Book
Validates that all documentation chapters meet word count targets.

Usage:
    python validate_word_counts.py
    python validate_word_counts.py --verbose
    python validate_word_counts.py --json

Exit codes:
    0: All validations passed
    1: One or more validations failed
"""

import os
import re
import sys
import json
import argparse
from pathlib import Path
from typing import Dict, Tuple, List


# Word count targets (min, max) for each document
WORD_COUNT_TARGETS = {
    "docs/modules/module-05-capstone/chapter-01-architecture.md": (350, 400),
    "docs/modules/module-05-capstone/chapter-02-voice-llm.md": (300, 350),
    "docs/modules/module-05-capstone/chapter-03-navigation-perception.md": (300, 350),
    "docs/modules/module-05-capstone/chapter-04-manipulation.md": (250, 300),
    "docs/modules/module-05-capstone/chapter-05-simulation-deployment.md": (400, 450),
    "docs/modules/module-05-capstone/chapter-06-jetson-deployment.md": (350, 400),
    "docs/modules/module-05-capstone/testing-methodology.md": (400, 500),
    "docs/modules/module-05-capstone/benchmarking.md": (350, 400),
    "docs/modules/module-05-capstone/troubleshooting.md": (450, 550),
}


def get_base_dir() -> Path:
    """Get the base directory of the project."""
    script_dir = Path(__file__).parent
    return script_dir.parent.parent


def count_words(file_path: Path) -> int:
    """
    Count words in markdown file, excluding:
    - YAML frontmatter
    - Code blocks (fenced with ```)
    - Inline code
    - URLs
    - Markdown syntax characters
    """
    if not file_path.exists():
        return 0

    content = file_path.read_text(encoding='utf-8')

    # Remove YAML frontmatter (between --- markers)
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL | re.MULTILINE)

    # Remove code blocks (``` ... ```) - non-greedy match
    content = re.sub(r'```[^`]*```', '', content, flags=re.DOTALL)

    # Remove inline code (`...`) - match single backticks not part of triple backticks
    content = re.sub(r'(?<!`)`(?!`)[^`\n]+`(?!`)', '', content)

    # Remove URLs
    content = re.sub(r'https?://\S+', '', content)

    # Remove markdown link syntax but keep text
    content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)

    # Remove markdown image syntax
    content = re.sub(r'!\[([^\]]*)\]\([^\)]+\)', '', content)

    # Remove markdown headings symbols (but keep the text)
    content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)

    # Remove markdown emphasis markers (but keep the text)
    # Replace **bold** with bold, *italic* with italic, etc.
    content = re.sub(r'\*\*([^\*]+)\*\*', r'\1', content)  # Bold
    content = re.sub(r'__([^_]+)__', r'\1', content)       # Bold alt
    content = re.sub(r'\*([^\*]+)\*', r'\1', content)      # Italic
    content = re.sub(r'_([^_]+)_', r'\1', content)         # Italic alt
    content = re.sub(r'~~([^~]+)~~', r'\1', content)       # Strikethrough

    # Remove HTML tags (must start with letter after <, to avoid matching < > operators)
    content = re.sub(r'<[a-zA-Z][^>]*>', '', content)

    # Remove horizontal rules (---, ***)
    content = re.sub(r'^[\-\*]{3,}$', '', content, flags=re.MULTILINE)

    # Remove list markers (-, *, 1., etc.)
    content = re.sub(r'^\s*[-\*\+]\s+', '', content, flags=re.MULTILINE)
    content = re.sub(r'^\s*\d+\.\s+', '', content, flags=re.MULTILINE)

    # Remove table formatting (|)
    content = re.sub(r'\|', ' ', content)

    # Split by whitespace and count non-empty tokens
    words = [word for word in content.split() if word.strip() and len(word) > 0]

    return len(words)


def validate_file(file_path: Path, min_words: int, max_words: int) -> Tuple[bool, int, str]:
    """
    Validate word count for a single file.

    Returns:
        (passed, word_count, status_message)
    """
    word_count = count_words(file_path)

    if not file_path.exists():
        return False, 0, f"File not found"

    if word_count < min_words:
        delta = min_words - word_count
        return False, word_count, f"Below target by {delta} words"
    elif word_count > max_words:
        delta = word_count - max_words
        return False, word_count, f"Above target by {delta} words"
    else:
        return True, word_count, "PASS"


def main():
    parser = argparse.ArgumentParser(description="Validate documentation word counts")
    parser.add_argument('--verbose', '-v', action='store_true',
                       help="Show detailed output for all files")
    parser.add_argument('--json', action='store_true',
                       help="Output results in JSON format")
    args = parser.parse_args()

    base_dir = get_base_dir()
    results = []
    all_passed = True

    # Validate each file
    for rel_path, (min_words, max_words) in WORD_COUNT_TARGETS.items():
        file_path = base_dir / rel_path
        passed, word_count, status = validate_file(file_path, min_words, max_words)

        results.append({
            'file': rel_path,
            'word_count': word_count,
            'target_range': f"{min_words}-{max_words}",
            'min_words': min_words,
            'max_words': max_words,
            'passed': passed,
            'status': status
        })

        if not passed:
            all_passed = False

    # Output results
    if args.json:
        output = {
            'passed': all_passed,
            'total_files': len(results),
            'passed_files': sum(1 for r in results if r['passed']),
            'failed_files': sum(1 for r in results if not r['passed']),
            'results': results
        }
        print(json.dumps(output, indent=2))
    else:
        # Text output
        print("=" * 80)
        print("Word Count Validation Report")
        print("=" * 80)
        print()

        # Summary statistics
        total = len(results)
        passed = sum(1 for r in results if r['passed'])
        failed = total - passed

        print(f"Total files:  {total}")
        print(f"Passed:       {passed}")
        print(f"Failed:       {failed}")
        print()

        # Individual file results
        if args.verbose or not all_passed:
            print("-" * 80)
            print(f"{'File':<50} {'Words':<10} {'Target':<15} {'Status':<20}")
            print("-" * 80)

            for result in results:
                file_name = os.path.basename(result['file'])
                word_count = result['word_count']
                target_range = result['target_range']
                status = result['status']

                # ASCII-safe status display
                if result['passed']:
                    status_display = f"[PASS] {status}"
                else:
                    status_display = f"[FAIL] {status}"

                print(f"{file_name:<50} {word_count:<10} {target_range:<15} {status_display:<20}")

        print()
        print("=" * 80)

        if all_passed:
            print("[SUCCESS] All word count validations PASSED")
        else:
            print(f"[FAILED] {failed} validation(s) FAILED")

        print("=" * 80)

    # Exit with appropriate code
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
