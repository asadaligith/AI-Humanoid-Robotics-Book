# Contributing Guide

## Welcome Contributors! 🎉

Thank you for your interest in contributing to the AI & Humanoid Robotics educational platform. This guide will help you get started.

## Ways to Contribute

### 1. Documentation Improvements
- Fix typos and grammatical errors
- Clarify confusing explanations
- Add missing examples or diagrams
- Translate content to other languages

### 2. Code Examples
- Add new ROS 2 examples
- Improve existing code samples
- Create interactive demos
- Add unit tests

### 3. Bug Reports
- Report broken links
- Identify incorrect information
- Flag outdated dependencies

### 4. Feature Requests
- Suggest new modules or topics
- Propose UI/UX improvements
- Request additional integrations

## Getting Started

### Prerequisites

- Git installed
- Node.js 18+ (for Docusaurus)
- Python 3.9+ (for chatbot/examples)
- GitHub account

### Fork and Clone

```bash
# 1. Fork the repository on GitHub
# 2. Clone your fork
git clone https://github.com/YOUR_USERNAME/AI-Humanoid-Robotics-Book.git
cd AI-Humanoid-Robotics-Book

# 3. Add upstream remote
git remote add upstream https://github.com/asadaligith/AI-Humanoid-Robotics-Book.git

# 4. Install dependencies
npm install
```

### Local Development

```bash
# Start development server
npm start

# Open http://localhost:3000
```

## Contribution Workflow

### 1. Create a Branch

```bash
# Update main branch
git checkout master
git pull upstream master

# Create feature branch
git checkout -b feature/your-feature-name

# Or for bug fixes
git checkout -b fix/issue-description
```

### 2. Make Changes

**Documentation Changes:**
- Edit `.md` files in `docs/` directory
- Follow existing formatting conventions
- Add code examples where helpful

**Code Changes:**
- Add examples to `examples/` directory
- Follow ROS 2 coding standards
- Include comments and docstrings

**Chatbot Changes:**
- Update files in `chatbot/` directory
- Write unit tests
- Update API documentation

### 3. Test Your Changes

```bash
# Test build
npm run build

# Check for broken links
npm run build 2>&1 | grep "Broken link"

# Test locally
npm run serve
```

### 4. Commit Your Changes

```bash
# Stage changes
git add .

# Commit with descriptive message
git commit -m "feat: add example for TF2 static transforms

- Added static_broadcaster.py example
- Included launch file configuration
- Updated README with usage instructions"
```

**Commit Message Format:**
```
<type>: <short summary>

<detailed description>

- Bullet point 1
- Bullet point 2
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `style`: Formatting, no code change
- `refactor`: Code restructuring
- `test`: Adding tests
- `chore`: Maintenance

### 5. Push and Create Pull Request

```bash
# Push to your fork
git push origin feature/your-feature-name

# Go to GitHub and create Pull Request
```

## Pull Request Guidelines

### PR Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Code refactoring

## Checklist
- [ ] Code follows project style guidelines
- [ ] Documentation has been updated
- [ ] All tests pass
- [ ] No broken links
- [ ] Commit messages are clear

## Screenshots (if applicable)
Add screenshots for UI changes
```

### Review Process

1. **Automated Checks**: CI/CD runs tests
2. **Maintainer Review**: Code review by maintainers
3. **Feedback**: Address requested changes
4. **Merge**: Once approved, PR is merged

## Code Style Guidelines

### Markdown Documentation

```markdown
# Use H1 for page title (only one per page)

## Use H2 for main sections

### Use H3 for subsections

- Use bullet points for lists
- Keep lines under 120 characters
- Add blank line between sections

**Bold** for emphasis
*Italic* for terminology

`inline code` for commands
```

### Code Examples

**Python (ROS 2):**
```python
#!/usr/bin/env python3
"""
Module docstring describing purpose.
"""
import rclpy
from rclpy.node import Node

class MyNode(Node):
    """Class docstring."""

    def __init__(self):
        """Initialize node."""
        super().__init__('my_node')
        # Implementation
```

**JavaScript:**
```javascript
// Use meaningful variable names
const navigationItems = getMenuItems();

// Add comments for complex logic
// Calculate weighted average of scores
const average = scores.reduce((sum, val) => sum + val, 0) / scores.length;
```

### File Naming

```
# Use kebab-case for files
ros2-publisher-example.md
gazebo-world-setup.md

# Use snake_case for Python
publisher_node.py
slam_config.py

# Use PascalCase for components
NavBar.js
ChatWidget.tsx
```

## Documentation Standards

### Structure

```markdown
# Title

## Overview
Brief introduction to the topic

## Prerequisites
- Requirement 1
- Requirement 2

## Installation
Step-by-step setup instructions

## Usage
How to use with examples

## Exercises
Hands-on practice exercises

## Common Issues
Troubleshooting guide

## References
External resources
```

### Code Blocks

````markdown
```python
# Always specify language
def example():
    return "Hello"
```

```bash
# Include $ prompt for commands
$ ros2 run my_package my_node
```
````

### Images

```markdown
# Store in static/img/
![Robot Architecture](../static/img/architecture.png)

# Alt text is required for accessibility
```

## Testing Requirements

### For Documentation

1. Build succeeds without errors
2. No broken links
3. Code examples are syntactically correct
4. Images load properly

### For Code Examples

1. Code runs without errors
2. Includes usage instructions
3. Has example output
4. Follows ROS 2 standards

### For Chatbot

1. Unit tests pass
2. Integration tests pass
3. API endpoints respond correctly
4. No security vulnerabilities

## Community Guidelines

### Code of Conduct

1. **Be Respectful**: Treat everyone with respect
2. **Be Constructive**: Provide helpful feedback
3. **Be Patient**: Remember everyone is learning
4. **Be Inclusive**: Welcome diverse perspectives

### Communication

- **GitHub Issues**: Bug reports and feature requests
- **Pull Requests**: Code and documentation changes
- **Discussions**: General questions and ideas

### Getting Help

**Stuck on something?**
1. Check existing documentation
2. Search closed issues
3. Ask in Discussions
4. Tag maintainers in PR

## Recognition

Contributors will be:
- Listed in CONTRIBUTORS.md
- Credited in release notes
- Given contributor badge

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (Apache 2.0).

## Additional Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [ROS 2 Contribution Guide](https://docs.ros.org/en/humble/Contributing.html)
- [Markdown Guide](https://www.markdownguide.org/)
- [Git Best Practices](https://git-scm.com/book/en/v2)

## Questions?

If you have questions not covered in this guide:
1. Check the FAQ
2. Open a Discussion
3. Contact maintainers

Thank you for contributing to robotics education! 🤖
