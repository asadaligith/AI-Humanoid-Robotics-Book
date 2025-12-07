# Implementation Plan: AI Humanoid Robotics Book (Docusaurus + Context7 MCP)

**Branch**: `005-capstone-autonomous-humanoid` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Comprehensive book project architecture plan from user input

## Summary

Transform five educational modules (ROS 2, Gazebo/Unity, Isaac Sim/ROS, VLA, Capstone) into a deployable technical book using Docusaurus static site generator, Context7 MCP for content/citation management, Spec-Kit Plus for automation, and GitHub Actions CI/CD. The book maintains APA citations, research-concurrent writing, and reproducible code examples validated through automated testing.

**Primary Requirement**: Deliver 1200-2000 word Capstone module + infrastructure for entire book
**Technical Approach**: Docusaurus (MDX) + Context7 MCP + automated CI/CD pipeline with simulation smoke tests

## Technical Context

**Language/Version**:
- Content: Markdown/MDX (Docusaurus 3.x)
- Code Examples: Python 3.10-3.14 (ROS 2 Humble/Iron compatible)
- Build: Node.js 18+ (Docusaurus requirement)
- Automation: PowerShell 7+ / Bash (cross-platform scripts)

**Primary Dependencies**:
- **Docusaurus** 3.x - Static site generator for technical documentation
- **Context7 MCP Server** - Content storage, citation management, research artifact tracking
- **Spec-Kit Plus** - Constitution enforcement, automated spec/plan/task generation
- **Claude Code** - AI-assisted draft generation, citation extraction, test generation
- **ROS 2 Humble/Iron** - Robotics framework for code examples
- **Gazebo Fortress / Isaac Sim** - Simulation environments
- **pytest** - Python code example validation
- **Docker** - Containerized testing environments

**Storage**:
- Source Control: Git (GitHub)
- Content Store: Context7 MCP server (metadata + references)
- Build Artifacts: GitHub Pages (static HTML)
- Large Assets: Git LFS or GitHub Releases (URDF, USD, simulation models)
- Citations: Context7 metadata → APA format conversion script

**Testing**:
- **Content Validation**: Source-check (every claim has Context7 ID or DOI), plagiarism scan, markdown linting
- **Code Validation**: pytest for Python examples, ROS 2 smoke tests in containers, URDF/USD integrity checks
- **Build Validation**: Docusaurus build success, deployment to GitHub Pages
- **Integration Testing**: End-to-end capstone demo reproducibility check

**Target Platform**:
- Development: Ubuntu 22.04 (recommended), Windows 10/11 (WSL2), macOS
- Deployment: GitHub Pages (static hosting)
- Runtime (students): Ubuntu 22.04 + ROS 2 Humble, optional Jetson Orin hardware

**Project Type**: Technical documentation website (Docusaurus) + educational content repository

**Performance Goals**:
- Docusaurus build time: <2 minutes for full site rebuild
- Page load time: <3 seconds for typical chapter (including code highlighting)
- CI/CD pipeline: <15 minutes end-to-end (lint + test + build + deploy)
- Code example execution: All Python examples run in <30 seconds each

**Constraints**:
- **APA Citation Style**: Mandated by Constitution (Principle III)
- **Chapter Length**: 700-1500 words (1200-2000 for Capstone) per Constitution (Principle IV)
- **Executable Code**: All examples must be tested and runnable per Constitution (Principle V)
- **Research-Concurrent**: Do not pause writing for full literature sweep; capture sources incrementally
- **No Hallucination**: Follow official documentation exactly (NVIDIA, ROS 2, Unitree)
- **Educational Context**: No production deployment guides, no safety-critical certification content

**Scale/Scope**:
- **5 Modules**: Each containing 3-5 chapters
- **Total Chapters**: ~20-25 chapters
- **Code Examples**: ~50-70 runnable Python scripts
- **Simulation Assets**: 10-15 URDF/USD robot models, 5-10 Gazebo/Isaac worlds
- **Citations**: Target 100-150 APA references across all modules
- **Diagrams**: 30-40 architecture/flow diagrams (Mermaid, PNG)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Technical Accuracy (NON-NEGOTIABLE)

**Status**: ✅ **PASS** (with controls)

**Verification Strategy**:
- All technical claims must reference Context7 entry or external DOI
- Code examples must pass pytest validation before inclusion
- Simulation parameters (physics, sensor specs) must cite vendor documentation
- Mathematical formulas must be verified against peer-reviewed sources

**Controls**:
- CI/CD gate: Build fails if any chapter missing source citations for technical claims
- Pre-commit hook: Flag code blocks without corresponding test files
- Manual review checkpoint at module completion

### II. Clarity for Target Audience

**Status**: ✅ **PASS**

**Target Audience**: Intermediate-to-advanced students completing Physical AI curriculum (Modules 1-4)

**Clarity Measures**:
- Assume readers have completed prior modules (ROS 2, simulation, VLA basics)
- Provide step-by-step integration guide with clear prerequisites
- Use consistent terminology from previous modules
- Include glossary for domain-specific robotics/AI terms

### III. Source-Backed Explanations

**Status**: ✅ **PASS** (automation required)

**Implementation**:
- Context7 MCP stores all source metadata (title, authors, URL, PDF snapshot)
- Automation script converts Context7 entries → APA citations
- Each chapter includes "Research & Evidence" subsection
- Inline citations use Context7 IDs, converted during build

**Citation Workflow**:
```
1. Author captures source in Context7 during writing
2. Insert placeholder: [Context7: ID-12345]
3. Build script converts: [Context7: ID-12345] → (Smith et al., 2024)
4. References section auto-generated from Context7 metadata
```

### IV. Modular Chapter Structure

**Status**: ✅ **PASS**

**Capstone Module Structure** (1200-2000 words per FR-010):
- **Chapter 1**: System Architecture & Integration Strategy (350-400 words)
- **Chapter 2**: Voice & LLM Pipeline Setup (300-350 words)
- **Chapter 3**: Navigation & Perception Integration (300-350 words)
- **Chapter 4**: Manipulation & Task Execution (250-300 words)
- **Chapter 5**: Simulation Deployment & Testing (300-350 words)
- **Chapter 6**: Jetson Hardware Deployment (200-250 words)

**Modularity Enforcement**:
- Each chapter states prerequisites explicitly
- Chapters can be taught/assigned independently within module context
- Cross-references use explicit links (not implicit dependencies)

### V. Executable Code and Reproducible Examples

**Status**: ✅ **PASS** (CI/CD enforced)

**Reproducibility Requirements**:
- All code examples in `examples/` directory with corresponding `tests/` files
- Each example includes:
  - README with environment specs (ROS 2 version, Python version, dependencies)
  - requirements.txt or package.xml
  - Expected outputs documented
  - pytest validation
- Simulation examples include:
  - URDF/USD model files
  - Launch files
  - Expected behavior description

**CI/CD Validation**:
- pytest runs all code examples
- ROS 2 smoke tests in Docker containers (no GPU)
- URDF/USD integrity checks (parseable, valid joint hierarchies)

### VI. Explainability and Step-by-Step Logic

**Status**: ✅ **PASS**

**Implementation**:
- Capstone integration follows staged approach (Priority P4 user story)
- Each subsystem (voice, LLM, nav, perception, manipulation) explained before integration
- Sequence diagrams show data flow step-by-step
- Troubleshooting guide addresses common integration failure modes
- Performance benchmarking section explains bottleneck analysis methodology

**No Violations**

## Project Structure

### Documentation (this feature - Capstone Module)

```text
specs/005-capstone-autonomous-humanoid/
├── plan.md              # This file
├── research.md          # Phase 0: Technology decisions, architecture patterns
├── data-model.md        # Phase 1: Content structure, chapter hierarchy
├── quickstart.md        # Phase 1: Student quick-start guide
├── contracts/           # Phase 1: ROS 2 interfaces, API schemas
│   ├── ros2-interfaces.md
│   ├── llm-prompts.md
│   └── state-machine.md
└── tasks.md             # Phase 2: Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root - Book Infrastructure)

```text
docs/                           # Docusaurus content root
├── intro.md                    # Book introduction
├── modules/
│   ├── module-01-ros2/
│   │   ├── index.md
│   │   ├── chapter-01-nodes-topics.md
│   │   ├── chapter-02-services-actions.md
│   │   └── chapter-03-urdf-workshop.md
│   ├── module-02-digital-twin/
│   ├── module-03-isaac-sim/
│   ├── module-04-vla/
│   └── module-05-capstone/
│       ├── index.md
│       ├── chapter-01-architecture.md
│       ├── chapter-02-voice-llm.md
│       ├── chapter-03-navigation-perception.md
│       ├── chapter-04-manipulation.md
│       ├── chapter-05-simulation-deployment.md
│       └── chapter-06-jetson-deployment.md
└── appendices/
    ├── environment-setup.md
    ├── hardware-bom.md
    ├── citations.md         # Auto-generated from Context7
    └── glossary.md

docusaurus.config.js            # Docusaurus configuration
sidebars.js                     # Navigation structure

src/                            # Custom Docusaurus components/plugins
├── components/
│   ├── Context7Citation.tsx   # Citation component
│   └── RunableCodeBlock.tsx   # Interactive code blocks
└── plugins/
    └── context7-import/       # Import Context7 content

examples/                       # Runnable code for all modules
├── module-01-ros2/
├── module-02-digital-twin/
├── module-03-isaac-sim/
├── module-04-vla/
└── module-05-capstone/
    ├── voice_input_node.py
    ├── llm_planner_node.py
    ├── navigation_controller.py
    ├── object_detection_node.py
    ├── manipulation_controller.py
    └── integration_demo.py

tests/                          # pytest tests for examples
├── module-01-ros2/
├── module-02-digital-twin/
├── module-03-isaac-sim/
├── module-04-vla/
└── module-05-capstone/
    ├── test_voice_input.py
    ├── test_llm_planner.py
    ├── test_navigation.py
    ├── test_object_detection.py
    ├── test_manipulation.py
    └── test_integration.py

assets/                         # Simulation models, diagrams
├── urdf/
│   ├── unitree_g1.urdf
│   └── humanoid_simplified.urdf
├── usd/
│   └── unitree_g1.usd
├── worlds/
│   ├── kitchen_env.world       # Gazebo
│   └── kitchen_env.usd         # Isaac Sim
└── diagrams/
    └── architecture/
        ├── capstone-architecture.png
        ├── vla-pipeline.png
        └── ros2-graph.png

scripts/                        # Automation and utility scripts
├── citations/
│   └── context7_to_apa.py     # Convert Context7 → APA citations
├── testing/
│   ├── run_ros2_smoke_tests.sh
│   └── validate_urdf.py
└── deployment/
    ├── build_docusaurus.sh
    └── deploy_github_pages.sh

.github/
└── workflows/
    ├── ci.yml                  # Main CI/CD pipeline
    ├── code-tests.yml          # Python/ROS 2 testing
    └── deploy.yml              # GitHub Pages deployment

.specify/                       # Spec-Kit Plus project files
├── memory/
│   └── constitution.md
├── templates/
└── scripts/

.context7/                      # Context7 MCP configuration
└── metadata/
    └── citations.json          # Citation database
```

**Structure Decision**:
- **Docusaurus-centric**: All content in `docs/` following Docusaurus conventions
- **Examples separate**: `examples/` mirrors `docs/` structure for easy navigation
- **Asset management**: Large files (URDF, USD) in `assets/` with potential Git LFS
- **CI/CD modular**: Separate workflows for different test types enable parallel execution

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No Violations Detected** - All Constitution principles satisfied with documented controls.

## Phase 0: Research & Architecture Decisions

**Objective**: Resolve technology choices, document tradeoffs, establish architecture patterns

### Research Tasks

1. **Docusaurus vs Alternatives** (MkDocs, Sphinx, GitBook)
   - **Decision**: Docusaurus 3.x
   - **Rationale**: MDX support (React components), excellent theming, versioning support, active community
   - **Alternatives**: MkDocs (Python-native but limited interactivity), Sphinx (RST complexity), GitBook (proprietary)

2. **Context7 MCP Integration Pattern**
   - **Decision**: Context7 as external metadata store, not embedded in Markdown
   - **Rationale**: Single source of truth for citations, enables reuse across multiple outputs, maintains markdown portability
   - **Alternatives**: Inline citations (less maintainable), BibTeX (lacks metadata richness)

3. **Simulation Backend Strategy**
   - **Decision**: Dual support - Gazebo (primary for accessibility) + Isaac Sim (advanced topics)
   - **Rationale**: Gazebo works on any hardware (broader reach), Isaac Sim provides photorealism for perception/VLA modules
   - **Tradeoffs**: Maintain two sets of examples vs single-simulator approach

4. **Code Execution Model**
   - **Decision**: Run-locally (no in-browser execution)
   - **Rationale**: Security concerns, ROS 2 requires system-level resources, reproducibility focus
   - **Alternatives**: Binder/JupyterHub (resource-intensive, slower), Katacoda (service dependency)

5. **Citation Storage and Rendering**
   - **Decision**: Context7 metadata → Build-time APA conversion → Static Markdown
   - **Rationale**: Portability, no runtime dependencies, enables offline reading
   - **Alternatives**: Runtime citation fetching (network dependency), manual BibTeX (error-prone)

6. **Hardware Deployment Documentation**
   - **Decision**: Jetson Orin as reference platform with cloud GPU alternatives documented
   - **Rationale**: Jetson is standard embedded AI platform, balances cost vs capability
   - **Tradeoffs**: Document Jetson-specific vs generic ARM deployment

7. **Testing Strategy for Simulation Examples**
   - **Decision**: Headless Gazebo in CI, Isaac Sim USD validation only (no full render)
   - **Rationale**: CI environment lacks GPU, USD schema validation catches most errors
   - **Alternatives**: Require GPU CI (expensive), skip simulation testing (risky)

**Output**: `research.md` with decisions, rationales, and alternatives for all technology choices

## Phase 1: Design & Contracts

**Prerequisites**: `research.md` complete

### 1. Content Structure (data-model.md)

Define hierarchical content model:

**Book Hierarchy**:
```
Book
├── Front Matter
│   ├── Preface
│   ├── How to Use This Book
│   ├── Conventions
│   └── Hardware & Cloud Options
├── Module 1: ROS 2 Nervous System
│   ├── Chapter 1-1: Nodes & Topics
│   ├── Chapter 1-2: Services & Actions
│   ├── Chapter 1-3: URDF Workshop
│   └── Research & Evidence (per chapter)
├── Module 2: Digital Twin
│   ├── Chapter 2-1: Gazebo Setup
│   ├── Chapter 2-2: Physics Simulation
│   ├── Chapter 2-3: Sensors
│   ├── Chapter 2-4: SDF vs URDF
│   └── Chapter 2-5: Unity Visualization
├── Module 3: AI-Robot Brain (Isaac)
│   ├── Chapter 3-1: Isaac Sim Quickstart
│   ├── Chapter 3-2: Synthetic Data Generation
│   ├── Chapter 3-3: Isaac ROS VSLAM
│   ├── Chapter 3-4: Nav2 Integration
│   └── Chapter 3-5: Perception Pipeline
├── Module 4: Vision-Language-Action
│   ├── Chapter 4-1: Whisper Voice Input
│   ├── Chapter 4-2: LLM Planning
│   ├── Chapter 4-3: Action Execution
│   ├── Chapter 4-4: End-to-End VLA
│   └── Chapter 4-5: Jetson Deployment
├── Module 5: Capstone (1200-2000 words)
│   ├── Chapter 5-1: System Architecture (350-400 words)
│   ├── Chapter 5-2: Voice & LLM Pipeline (300-350 words)
│   ├── Chapter 5-3: Navigation & Perception (300-350 words)
│   ├── Chapter 5-4: Manipulation & Execution (250-300 words)
│   ├── Chapter 5-5: Simulation Deployment (300-350 words)
│   └── Chapter 5-6: Jetson Deployment (200-250 words)
└── Appendices
    ├── Environment Setup Scripts
    ├── Hardware Bill of Materials
    ├── Citation Index (APA, auto-generated)
    └── Glossary
```

**Chapter Template Structure**:
```markdown
# Chapter Title

## Learning Objectives
- [3-5 specific, measurable objectives]

## Prerequisites
- [List dependencies on prior chapters]
- [Required software/hardware]

## Introduction
[Context and motivation - 100-150 words]

## Core Content
[Main explanatory content - 400-1000 words]
### Subsection 1
### Subsection 2
...

## Code Example
[Runnable code with inline comments]
[Link to full example in examples/ directory]

## Research & Evidence
[Sources for technical claims, link to Context7 entries]
[APA citations auto-inserted during build]

## Summary
[Key takeaways - 100-150 words]

## Exercises
[2-3 practice problems]

## References
[Auto-generated APA bibliography from Context7]
```

**Citation Metadata Schema** (Context7):
```json
{
  "id": "CTX7-001",
  "type": "research_paper|documentation|vendor_guide",
  "title": "Paper Title",
  "authors": ["Author 1", "Author 2"],
  "year": 2024,
  "url": "https://...",
  "doi": "10.1234/...",
  "pdf_snapshot": "base64_encoded_or_url",
  "notes": "Why this source is relevant",
  "version": "ROS 2 Humble, Isaac Sim 2023.1",
  "freshness": "2024-12-07",
  "tags": ["ros2", "navigation", "vslam"]
}
```

### 2. API Contracts (contracts/)

**ROS 2 Interface Contracts** (`contracts/ros2-interfaces.md`):

Defines standard interfaces for capstone system integration:

```markdown
# ROS 2 Interface Contracts - Capstone

## Voice Input Node
- **Node Name**: `/voice_input_node`
- **Published Topics**:
  - `/voice/transcribed_text` (std_msgs/String): Transcribed voice commands
- **Subscribed Topics**: None
- **Services**: None
- **Actions**: None
- **Parameters**:
  - `whisper_model`: base|small|medium (default: base)
  - `language`: en|es|fr|... (default: en)

## LLM Planner Node
- **Node Name**: `/llm_planner_node`
- **Published Topics**:
  - `/planning/action_sequence` (custom_msgs/ActionPlan): JSON action sequence
- **Subscribed Topics**:
  - `/voice/transcribed_text` (std_msgs/String): Input commands
  - `/system/capability_manifest` (custom_msgs/CapabilityList): Available robot actions
- **Services**:
  - `/planning/validate_plan` (custom_srvs/ValidatePlan): Check plan feasibility
- **Actions**: None

## Navigation Controller
- **Node Name**: `/navigation_controller`
- **Published Topics**:
  - `/navigation/status` (std_msgs/String): Current navigation state
- **Subscribed Topics**: None
- **Services**: None
- **Actions**:
  - `/navigation/navigate_to_pose` (nav2_msgs/NavigateToPose): Goal navigation

## Object Detection Node
- **Node Name**: `/object_detection_node`
- **Published Topics**:
  - `/perception/detected_objects` (vision_msgs/Detection2DArray): Detected objects
- **Subscribed Topics**:
  - `/camera/image_raw` (sensor_msgs/Image): Camera feed
- **Services**: None
- **Actions**: None

## Manipulation Controller
- **Node Name**: `/manipulation_controller`
- **Published Topics**: None
- **Subscribed Topics**:
  - `/perception/detected_objects` (vision_msgs/Detection2DArray): Target objects
- **Services**: None
- **Actions**:
  - `/manipulation/pick_object` (custom_msgs/PickObject): Pick action
  - `/manipulation/place_object` (custom_msgs/PlaceObject): Place action
```

**LLM Prompt Templates** (`contracts/llm-prompts.md`):

```markdown
# LLM Prompt Engineering Contracts

## Task Planning Prompt Template

```
System: You are a robot task planner. Generate a structured action sequence for the following command. Available actions: navigate_to(x, y), detect_object(name), pick_object(object_id), place_object(location).

User Command: {transcribed_voice_command}

Robot Capabilities:
{capability_manifest_json}

Output Format (JSON):
{
  "command_understood": true/false,
  "clarification_needed": null or "question",
  "action_sequence": [
    {"action": "navigate_to", "params": {"x": 1.5, "y": 2.0}},
    {"action": "detect_object", "params": {"name": "coffee_mug"}},
    ...
  ]
}
```

## Example Interactions

**Input**: "Bring me the red cup from the kitchen table"
**Output**:
```json
{
  "command_understood": true,
  "clarification_needed": null,
  "action_sequence": [
    {"action": "navigate_to", "params": {"location": "kitchen_table"}},
    {"action": "detect_object", "params": {"name": "cup", "color": "red"}},
    {"action": "pick_object", "params": {"object_id": "detected_cup_id"}},
    {"action": "navigate_to", "params": {"location": "user_location"}},
    {"action": "place_object", "params": {"location": "user_hand"}}
  ]
}
```
```

**State Machine Definition** (`contracts/state-machine.md`):

```markdown
# Capstone Pipeline State Machine

## States

1. **IDLE**: Waiting for voice command
2. **LISTENING**: Capturing audio
3. **TRANSCRIBING**: Whisper processing
4. **PLANNING**: LLM generating action sequence
5. **VALIDATING**: Checking plan feasibility
6. **NAVIGATING**: Moving to target location
7. **PERCEIVING**: Detecting target object
8. **MANIPULATING**: Picking/placing object
9. **COMPLETED**: Task finished successfully
10. **FAILED**: Error occurred (with failure reason)
11. **AWAITING_CLARIFICATION**: LLM needs more info

## Transitions

```
IDLE → LISTENING (voice activity detected)
LISTENING → TRANSCRIBING (audio captured)
TRANSCRIBING → PLANNING (text available)
PLANNING → VALIDATING (plan generated)
VALIDATING → NAVIGATING (plan valid) | AWAITING_CLARIFICATION (plan invalid) | FAILED (no valid plan)
NAVIGATING → PERCEIVING (arrived at location) | FAILED (navigation error)
PERCEIVING → MANIPULATING (object detected) | FAILED (object not found)
MANIPULATING → NAVIGATING (object picked, return journey) | COMPLETED (object delivered) | FAILED (grasp failure)
AWAITING_CLARIFICATION → LISTENING (user provides clarification)
FAILED → IDLE (after error logging)
COMPLETED → IDLE (ready for next task)
```

## Failure Handling

Each state defines failure modes and recovery strategies:
- **NAVIGATING failure**: Replan with obstacle avoidance
- **PERCEIVING failure**: Request user to point out object or retry with different viewpoint
- **MANIPULATING failure**: Retry grasp with adjusted pose (max 3 attempts)
```

### 3. Student Quickstart Guide (`quickstart.md`)

```markdown
# Quickstart: Running the Capstone Demo

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble installed ([install guide](https://docs.ros.org/en/humble/Installation.html))
- Python 3.10+
- Gazebo Fortress or Isaac Sim 2023.1+
- OpenAI API key (for LLM planner)

## Quick Setup (15 minutes)

### 1. Clone Repository

```bash
git clone https://github.com/your-org/ai-humanoid-robotics-book.git
cd ai-humanoid-robotics-book
```

### 2. Install Dependencies

```bash
# Install ROS 2 workspace dependencies
cd examples/module-05-capstone
rosdep install --from-paths . --ignore-src -r -y

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Set Environment Variables

```bash
export OPENAI_API_KEY="your-api-key-here"
export ROS_DOMAIN_ID=42
```

### 4. Launch Simulation

**Gazebo Option**:
```bash
ros2 launch capstone_demo gazebo_demo.launch.py
```

**Isaac Sim Option** (requires NVIDIA GPU):
```bash
ros2 launch capstone_demo isaac_demo.launch.py
```

### 5. Run Capstone Pipeline

In a new terminal:
```bash
cd examples/module-05-capstone
ros2 run capstone_demo integration_demo.py
```

### 6. Issue Voice Command

Speak into microphone: "Bring me the red cup from the table"

Expected behavior:
1. Transcription appears in terminal
2. LLM plan displayed
3. Robot navigates to table
4. Detects red cup
5. Picks up cup
6. Returns to user
7. Places cup

## Validation

Run automated tests:
```bash
cd tests/module-05-capstone
pytest -v
```

Expected: All tests pass (may take 5-10 minutes)

## Troubleshooting

**Voice not detected**: Check microphone permissions, adjust `whisper_model` parameter
**LLM timeout**: Verify API key, check network connectivity
**Navigation fails**: Verify map loaded correctly, check obstacle clearance
**Object not detected**: Ensure lighting adequate, verify object is in camera FOV
**Grasp fails**: Adjust grasp pose parameters, check joint limits

## Next Steps

- Read [Chapter 5-1: System Architecture](../docs/modules/module-05-capstone/chapter-01-architecture.md)
- Explore [troubleshooting guide](../docs/modules/module-05-capstone/troubleshooting.md)
- Modify LLM prompts in `contracts/llm-prompts.md`
- Benchmark performance using `scripts/testing/benchmark_capstone.py`
```

## Phase 2: Task Breakdown

**Note**: Task breakdown is handled by `/sp.tasks` command (separate from `/sp.plan`).

The implementation tasks will include:

1. **Repository Setup** (Phase A - Days 0-3)
   - Initialize Docusaurus site
   - Configure Context7 MCP server
   - Set up GitHub Actions CI/CD
   - Create citation automation script

2. **Module Content Creation** (Phase B - Weeks 1-2)
   - Draft Module 1 chapters (ROS 2)
   - Draft Module 2 chapters (Digital Twin)
   - Draft Module 3 chapters (Isaac Sim)
   - Draft Module 4 chapters (VLA)
   - For each: Write content, create examples, add tests, peer review

3. **Capstone Module** (Phase C - Week 3)
   - Write 6 capstone chapters (1200-2000 words total)
   - Create integration demo code
   - Set up simulation environments
   - Write Jetson deployment guide
   - Performance benchmarking framework

4. **Polish & Deploy** (Phase D)
   - Final QA on all content
   - Generate complete citation index
   - Deploy to GitHub Pages
   - Create student quickstart

## Acceptance Criteria

### Module-Level Criteria

Each of 5 modules MUST have:
- ✅ At least 1 runnable example validated by CI (pytest + ROS 2 smoke test)
- ✅ Research & Evidence subsection with minimum citations:
  - Module 1-2: 5+ sources
  - Module 3-4: 8+ sources
  - Module 5 (Capstone): 10+ sources
- ✅ All diagrams render correctly in Docusaurus
- ✅ All code blocks syntax-highlighted with language specified

### Book-Level Criteria

- ✅ Docusaurus build completes without warnings
- ✅ Deploys successfully to GitHub Pages
- ✅ Passes plagiarism check (automated scan)
- ✅ Citation index auto-generated from Context7 (APA format)
- ✅ All Python examples pass pytest (100% test pass rate)
- ✅ ROS 2 container smoke tests pass (pub/sub, service, action examples)
- ✅ URDF/USD assets validated (parseable, valid schemas)
- ✅ Capstone reproducibility checklist included and verified

### Capstone-Specific Criteria

- ✅ Total word count: 1200-2000 words (per FR-010)
- ✅ System architecture diagram showing all ROS 2 nodes, topics, actions (per FR-004)
- ✅ Data flow diagrams for nominal + 2 failure scenarios (per FR-005)
- ✅ Integration demo runs successfully in Gazebo (60% success rate acceptable per SC-001)
- ✅ Jetson deployment instructions tested on actual hardware (if available)
- ✅ Performance benchmarking framework produces metrics report

## Testing Checklist

**Pre-Commit**:
- [ ] Markdown linter passes (no broken links, proper syntax)
- [ ] Code examples have corresponding test files
- [ ] No API keys or secrets in commits

**CI/CD Pipeline**:
- [ ] Docusaurus build success (no warnings)
- [ ] Citation index generated (APA format)
- [ ] All code examples pass pytest
- [ ] ROS 2 container smoke tests pass
- [ ] URDF/USD assets validated (schema checks)
- [ ] Plagiarism scan passes
- [ ] Deployment to GitHub Pages successful

**Manual QA** (before release):
- [ ] Peer review of each module completed
- [ ] Student test group runs quickstart successfully (>80% success rate)
- [ ] Capstone demo reproducible on clean Ubuntu 22.04 install
- [ ] All external links valid (automated link checker)
- [ ] Diagrams load correctly on GitHub Pages

## Implementation Phases

### Phase A: Setup (Days 0-3)

**Deliverables**:
- Docusaurus site skeleton with theme configured
- Context7 MCP server deployed and accessible
- GitHub Actions workflows (lint, test, deploy) functional
- Citation automation script (`context7_to_apa.py`) tested
- Repository structure created

**Validation**:
- Docusaurus dev server runs: `npm start`
- Context7 can store/retrieve citation metadata
- CI pipeline runs (even if tests are empty)

### Phase B: Module Drafts + Tests (Weeks 1-2, Iterative)

**Per-Module Workflow**:
1. Draft all chapters for module (using Claude Code + Spec-Kit Plus)
2. Create runnable examples in `examples/module-XX/`
3. Write pytest tests in `tests/module-XX/`
4. Capture sources in Context7
5. Peer review (technical accuracy + writing quality)
6. Merge to main branch

**Modules 1-4**: Standard 700-1500 word chapters
**Module 5 (Capstone)**: Extended 1200-2000 words total

**Validation**:
- Each module passes CI before merge
- Peer reviewer sign-off documented

### Phase C: Capstone & Polishing (Week 3)

**Deliverables**:
- All 6 capstone chapters written
- Integration demo code + tests
- Simulation environments (Gazebo world + Isaac USD scene)
- Jetson deployment Docker container
- Performance benchmarking framework
- Troubleshooting guide

**Validation**:
- End-to-end demo runs successfully
- Benchmarking produces metrics report
- Jetson instructions tested (if hardware available)

### Phase D: Release (Final)

**Deliverables**:
- Final build deployed to GitHub Pages
- Release notes published
- Student quickstart guide validated
- Citation index complete (APA)
- Glossary finalized

**Validation**:
- Public site accessible and functional
- Student test group completes quickstart (<90 min setup time)
- All acceptance criteria met

## Risk Mitigation

**Risk**: Context7 MCP integration complexity
**Mitigation**: Build citation script early (Phase A), test with mock data

**Risk**: Isaac Sim examples fail in CI (no GPU)
**Mitigation**: USD schema validation only, document GPU-required examples separately

**Risk**: Capstone integration demo too complex (>60% failure rate)
**Mitigation**: Provide simplified fallback demo, document known limitations

**Risk**: Citation count insufficient
**Mitigation**: Research-concurrent approach, flag chapters below threshold during peer review

**Risk**: Student environments vary (Ubuntu versions, ROS distros)
**Mitigation**: Docker containers for standardized testing, document version requirements explicitly

## Next Steps

1. Run `/sp.tasks` to generate detailed implementation tasks
2. Begin Phase A setup work
3. Establish Context7 MCP schema and test citation workflow
4. Draft Module 1 Chapter 1 as proof-of-concept

## Notes

- This plan treats the entire book infrastructure + Capstone module as a single feature for planning purposes
- Subsequent modules (1-4) will follow similar pattern but with their own specs/plans if needed
- Automation (Claude Code + Spec-Kit Plus) will accelerate draft generation while maintaining quality gates
