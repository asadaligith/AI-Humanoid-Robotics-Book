#!/usr/bin/env python3
"""
Context7 to APA Citation Converter

This script reads citation metadata from Context7 (.context7/metadata/citations.json)
and generates APA-formatted citations in markdown format for the appendix.

Usage:
    python context7_to_apa.py

Output:
    Updates docs/appendices/citations.md with formatted references
"""

import json
import os
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Any


def load_citations(citations_file: Path) -> List[Dict[str, Any]]:
    """Load citations from Context7 metadata file."""
    if not citations_file.exists():
        print(f"Warning: Citations file not found at {citations_file}")
        return []

    with open(citations_file, 'r', encoding='utf-8') as f:
        data = json.load(f)

    return data.get('citations', [])


def format_authors_apa(authors: List[str]) -> str:
    """Format authors list in APA style."""
    if not authors:
        return "Unknown Author"

    if len(authors) == 1:
        return authors[0]
    elif len(authors) == 2:
        return f"{authors[0]}, & {authors[1]}"
    elif len(authors) <= 20:
        formatted = ", ".join(authors[:-1])
        return f"{formatted}, & {authors[-1]}"
    else:
        # APA 7th edition: list up to 19 authors, then ellipsis, then last author
        formatted = ", ".join(authors[:19])
        return f"{formatted}, ... {authors[-1]}"


def format_citation_apa(citation: Dict[str, Any]) -> str:
    """Format a single citation in APA 7th edition style."""
    citation_id = citation.get('id', 'UNKNOWN')
    title = citation.get('title', 'Untitled')
    authors = citation.get('authors', [])
    year = citation.get('year', 'n.d.')
    url = citation.get('url', '')
    doi = citation.get('doi', '')
    citation_type = citation.get('type', 'unknown')

    # Format authors
    authors_str = format_authors_apa(authors)

    # Format title based on type
    if citation_type == 'research_paper':
        title_formatted = f"{title}."
    elif citation_type in ['documentation', 'vendor_guide']:
        title_formatted = f"*{title}*."
    else:
        title_formatted = f"{title}."

    # Build citation
    parts = [authors_str, f"({year}).", title_formatted]

    # Add DOI or URL
    if doi:
        parts.append(f"https://doi.org/{doi}")
    elif url:
        parts.append(url)

    apa_citation = " ".join(parts)

    # Add Context7 ID reference
    return f"- **[{citation_id}]** {apa_citation}\n"


def generate_citations_markdown(citations: List[Dict[str, Any]]) -> str:
    """Generate complete citations markdown content."""
    if not citations:
        return """# Citations

No citations have been added yet. Citations will appear here as they are captured during content development.

"""

    # Sort citations by ID
    sorted_citations = sorted(citations, key=lambda x: x.get('id', ''))

    # Group by module if available
    by_module = {}
    ungrouped = []

    for citation in sorted_citations:
        tags = citation.get('tags', [])
        module_tag = next((tag for tag in tags if tag.startswith('module-')), None)

        if module_tag:
            if module_tag not in by_module:
                by_module[module_tag] = []
            by_module[module_tag].append(citation)
        else:
            ungrouped.append(citation)

    # Build markdown
    content = [
        "# Citations\n",
        "\n",
        f"**Total Citations**: {len(citations)}\n",
        f"**Last Updated**: {datetime.now().strftime('%Y-%m-%d')}\n",
        "\n",
        "This appendix contains all references cited throughout the AI Humanoid Robotics Book, ",
        "formatted in APA 7th edition style. Citations are organized by module.\n",
        "\n",
        "---\n",
        "\n"
    ]

    # Add module sections
    module_names = {
        'module-01': 'Module 1: ROS 2 Nervous System',
        'module-02': 'Module 2: Digital Twin',
        'module-03': 'Module 3: AI Robot Brain',
        'module-04': 'Module 4: Vision-Language-Action',
        'module-05': 'Module 5: Capstone'
    }

    for module_key in sorted(by_module.keys()):
        module_name = module_names.get(module_key, module_key)
        content.append(f"## {module_name}\n\n")

        for citation in by_module[module_key]:
            content.append(format_citation_apa(citation))

        content.append("\n")

    # Add ungrouped citations
    if ungrouped:
        content.append("## General References\n\n")
        for citation in ungrouped:
            content.append(format_citation_apa(citation))
        content.append("\n")

    return "".join(content)


def main():
    """Main entry point."""
    # Paths
    repo_root = Path(__file__).parent.parent.parent
    citations_file = repo_root / '.context7' / 'metadata' / 'citations.json'
    output_file = repo_root / 'docs' / 'appendices' / 'citations.md'

    print("Context7 to APA Citation Converter")
    print("=" * 50)
    print(f"Loading citations from: {citations_file}")

    # Load citations
    citations = load_citations(citations_file)
    print(f"Found {len(citations)} citations")

    # Generate markdown
    markdown_content = generate_citations_markdown(citations)

    # Ensure output directory exists
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # Write output
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(markdown_content)

    print(f"Citations written to: {output_file}")
    print("Done!")


if __name__ == '__main__':
    main()
