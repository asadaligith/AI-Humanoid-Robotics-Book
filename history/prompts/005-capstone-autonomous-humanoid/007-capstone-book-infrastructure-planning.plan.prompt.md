---
id: 007
title: Capstone Book Infrastructure Planning
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: 005-capstone-autonomous-humanoid
branch: 005-capstone-autonomous-humanoid
user: Asad
command: /sp.plan
labels: ["capstone", "planning", "docusaurus", "context7", "book-infrastructure", "ci-cd", "architecture"]
links:
  spec: specs/005-capstone-autonomous-humanoid/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-capstone-autonomous-humanoid/plan.md
tests:
  - Constitution check (6/6 principles validated)
  - Technical context generation complete
  - Phase 0 research decisions documented (7 choices)
  - Phase 1 design artifacts embedded in plan
---

## Prompt

/sp.plan Project: AI Humanoid Robotics Book — Technical Plan (Docusaurus + Context7 MCP)

Goal:
- Turn module specs into a reproducible, deployable technical book using Docusaurus, Context7 MCP server, Spec-Kit Plus, and Claude Code automation. Maintain APA citations and research-concurrent writing.

Create:
1. Architecture sketch (high level)
   - Authoring layer: Spec-Kit Plus prompts + Claude Code pipelines
   - Content store: Context7 MCP, source MD/MDX, assets
   - Build layer: Docusaurus with custom plugins
   - CI/CD: GitHub Actions → GitHub Pages

2. Section structure (book -> modules -> chapters)
   - Front matter (Preface, How to Use This Book, Conventions, Hardware Options)
   - Module 1: ROS 2 Nervous System (3 chapters, 700-1500 words)
   - Module 2: Digital Twin (5 chapters, 700-1500 words)
   - Module 3: Isaac AI Brain (5 chapters, 700-1500 words)
   - Module 4: VLA (5 chapters, 700-1500 words)
   - Module 5: Capstone (6 chapters, total 1200-2000 words split as: Ch1 350-400, Ch2 300-350, Ch3 300-350, Ch4 250-300, Ch5 300-350, Ch6 200-250)
   - Appendices (Setup scripts, Hardware BOM, Citations APA, Glossary)

3. Research approach
   - Research-concurrent (parallel research + writing)
   - Context7 source capture strategy: store citation metadata + DOI/URL
   - APA citations auto-generated from Context7 at build time
   - Quality gates: technical accuracy, source-backed claims, executable code

4. Quality validation / testing strategy
   - Content validation: linting (markdownlint), link checking, citation format validation
   - Technical tests: pytest on Python examples, ROS 2 package builds, Gazebo/Isaac SDF/USD validation
   - End-to-end acceptance: sample chapter with working code + citations rendered correctly in Docusaurus

5. Acceptance criteria
   - Module-level: Each module meets spec requirements (user stories, success criteria)
   - Book-level: Deployable to GitHub Pages with working nav, search, APA citations
   - Capstone-specific: 6 chapters total 1200-2000 words, end-to-end demo reproducible in Gazebo, architecture diagrams included

6. Implementation phases
   - Phase A: Setup (Days 0-3)
     - Initialize Docusaurus project, configure Context7 MCP, establish CI/CD pipelines, create citation build script
   - Phase B: Module Drafts (Weeks 1-2)
     - Iterative module creation with peer review, parallel research + Context7 capture, runnable code examples with CI validation
   - Phase C: Capstone & Polishing (Week 3)
     - Write 6 capstone chapters, create integration demo, performance benchmarking, final APA formatting and review
   - Phase D: Release
     - Deploy to GitHub Pages, student validation pilot, collect feedback for revisions

Technical constraints:
- Docusaurus 3.x (MDX support, theming, versioning)
- Context7 MCP as external metadata store (not embedded in MD)
- ROS 2 Humble/Iron compatibility
- Python 3.10-3.14 (ROS 2 compatible)
- Gazebo Fortress or Isaac Sim 2023.1
- GitHub Actions CI/CD
- Docker for containerized testing (ROS 2, Gazebo)
- pytest for Python code validation

Citation workflow:
1. Author writes content in MDX
2. References captured in Context7 MCP (ID, DOI, URL, title, authors, year, tags)
3. In-text citations use Context7 ID or DOI placeholder
4. Build-time script queries Context7, generates APA citations, injects into final MD
5. Docusaurus renders static site with formatted references

Success metrics:
- All 5 modules published with working code examples
- >95% uptime on GitHub Pages
- APA citation accuracy >99%
- Student pilot: >70% successfully run Module 1 examples
- Capstone: >50% complete simulation demo

## Response snapshot

Created comprehensive technical implementation plan for AI Humanoid Robotics Book infrastructure integrating Docusaurus, Context7 MCP, and SpecKit Plus.

**File created**: `specs/005-capstone-autonomous-humanoid/plan.md` (887 lines)

**Key deliverables**:
- **Technical Context**: Full stack specification (Docusaurus 3.x, Context7 MCP, ROS 2 Humble/Iron, Gazebo Fortress, Isaac Sim 2023.1, pytest, Docker, GitHub Actions)
- **Constitution Check**: All 6 principles validated with specific controls (APA citations, staged integration, CI/CD enforcement, modular chapter structure)
- **Project Structure**: Complete hierarchy showing docs/, examples/, tests/, assets/, scripts/ with Capstone split into 6 chapters (350-400, 300-350, 300-350, 250-300, 300-350, 200-250 words)
- **Phase 0 Research**: 7 technology decisions documented (Docusaurus vs alternatives, Context7 integration, dual simulation strategy, run-locally code, build-time citations, Jetson reference platform, headless CI)
- **Phase 1 Design Artifacts** (embedded in plan):
  - Content structure with book hierarchy and chapter template
  - Citation metadata schema (JSON format with Context7 IDs)
  - ROS 2 interface contracts (5 nodes: voice_input, llm_planner, navigation, object_detection, manipulation)
  - LLM prompt templates with validation examples
  - State machine (11 states with transitions and failure handling)
  - Student quickstart guide (15-minute setup)

**Architecture Highlights**:
- Authoring layer: SpecKit Plus + Claude Code automation
- Content store: Context7 MCP (external metadata) + Git (source MD/MDX)
- Build layer: Docusaurus with custom citation plugin
- CI/CD: GitHub Actions → pytest + linting → GitHub Pages
- Citation workflow: Context7 capture → build-time APA conversion → static site

**Implementation Phases**:
- Phase A (Days 0-3): Docusaurus setup, Context7 config, CI/CD, citation script
- Phase B (Weeks 1-2): Module drafts with parallel research, runnable examples, CI validation
- Phase C (Week 3): 6 capstone chapters, integration demo, benchmarking, APA review
- Phase D: GitHub Pages deployment, student pilot, feedback collection

**Quality Validation**:
- Content: markdownlint, link checking, citation format validation
- Technical: pytest (Python), ROS 2 builds, Gazebo/Isaac validation
- Acceptance: deployable GitHub Pages, working nav/search, APA rendered correctly

**Next steps**: Run `/sp.tasks` to generate detailed implementation task breakdown

## Outcome

- ✅ Impact: Created comprehensive book infrastructure plan integrating 5 modules into deployable Docusaurus site with automated APA citations via Context7 MCP
- 🧪 Tests: Constitution check PASS (6/6 principles), technical context validated, research decisions documented (7 choices)
- 📁 Files: plan.md (887 lines with embedded Phase 0/1 artifacts)
- 🔁 Next prompts: `/sp.tasks` for implementation task breakdown
- 🧠 Reflection: Treated book infrastructure as "implementation" for Capstone feature. Embedded Phase 0/1 content in plan.md rather than separate files for coherence. Emphasized research-concurrent workflow with Context7 automation for APA compliance.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution check PASS (6/6), technical context validated, Phase 0 research decisions complete (7), Phase 1 design artifacts embedded
- Prompt variant (if applicable): Meta-plan for entire book infrastructure (not just single module implementation)
- Next experiment (smallest change to try): Consider extracting research.md, data-model.md, contracts/*.md as separate files if plan.md becomes too large for future reference
