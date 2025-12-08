---
id: 002
title: Debug and fix MDX compilation errors
stage: green
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.analyze
labels: ["debugging", "mdx", "docusaurus", "bug-fix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docs/modules/module-05-capstone/benchmarking.md
  - docs/modules/module-05-capstone/chapter-03-navigation-perception.md
  - docs/modules/module-05-capstone/chapter-04-manipulation.md
  - docs/modules/module-05-capstone/chapter-05-simulation-deployment.md
  - docs/modules/module-05-capstone/chapter-06-jetson-deployment.md
  - docs/modules/module-05-capstone/testing-methodology.md
  - docs/modules/module-05-capstone/troubleshooting.md
tests:
  - npm run build (webpack compilation)
---

## Prompt

there have many errors in project, not showing any page , not found page, nor any UI , run this project npm start and check errors in terminal and resolved it

## Response snapshot

Identified and resolved 7 MDX compilation errors caused by improper escaping of less-than symbols (`<`) followed by digits in markdown files. The issue was that Docusaurus MDX parser interprets `<[0-9]` patterns as invalid JSX tag names.

**Root cause**: Markdown content contained expressions like `<30s`, `<100ms`, `<15 seconds`, `<90 seconds`, etc., which MDX interprets as JSX tags. JSX tag names cannot start with digits, causing compilation failures.

**Solution applied**: Replaced all instances of `<` followed by digits with HTML entity `&lt;` to escape them properly:
- benchmarking.md: 5 instances fixed
- chapter-03-navigation-perception.md: 2 instances fixed
- chapter-04-manipulation.md: 2 instances fixed
- chapter-05-simulation-deployment.md: 8 instances fixed (including table entries)
- chapter-06-jetson-deployment.md: 1 instance fixed
- testing-methodology.md: 11 instances fixed
- troubleshooting.md: 2 instances fixed

**Total**: 31 MDX syntax errors across 7 files resolved.

**Verification**: Ran `npm run build` which compiled successfully with zero MDX errors:
- ✔ Server: Compiled successfully in 14.85s
- ✔ Client: Compiled successfully in 27.37s

## Outcome

- ✅ Impact: All 7 MDX compilation errors resolved, Docusaurus can now compile the documentation site successfully
- 🧪 Tests: Build compilation verified (npm run build passed with no MDX errors)
- 📁 Files: 7 markdown files modified with 31 total fixes applied
- 🔁 Next prompts: User should restart dev server (`npm start`) with fresh port or use `npm run build && npm run serve` to view the site
- 🧠 Reflection: Systematic grep search for `<[0-9]` pattern ensured all instances were found and fixed; cache clearing was critical for testing fixes

## Evaluation notes (flywheel)

- Failure modes observed: Persistent webpack cache caused old errors to appear even after fixes; port conflicts from multiple background processes
- Graders run and results (PASS/FAIL): Build compilation - PASS (no MDX errors), Dev server - BLOCKED (port conflict, but not due to code errors)
- Prompt variant (if applicable): N/A (bug fix task)
- Next experiment (smallest change to try): User should kill all Node processes on port 3000 and restart dev server cleanly, or use production build for immediate verification
