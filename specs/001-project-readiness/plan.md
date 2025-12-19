# Implementation Plan: Project Readiness - 100% Launch Path

**Branch**: `001-project-readiness` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Gap analysis identifying 42% overall readiness with critical gaps in documentation, implementation, and integration

---

## Summary

This plan provides a complete roadmap to achieve 100% project readiness for the "Physical AI & Humanoid Robotics" educational platform. The project currently stands at 42% completion with strong specifications (85%) but significant gaps in documentation delivery (20%), implementation (15%), and testing infrastructure (0%). This plan organizes work into 7 phases progressing from foundation validation through final release, addressing all identified gaps systematically.

**Primary Objective**: Execute a phase-based approach to complete all modules (1-5), integrate the RAG chatbot learning assistant, establish testing and deployment infrastructure, and validate the complete end-to-end learning experience.

**Success Metric**: Achieve 100% project readiness across all dimensions (specs, docs, implementation, testing, deployment, integration) enabling student enrollment and hands-on learning.

---

## Technical Context

**Language/Version**:
- Python 3.10+ (ROS 2, backend services, AI pipelines)
- JavaScript ES6+ (Docusaurus platform, RAG chatbot frontend widget)
- C++ (ROS 2 nodes, low-level robotics code)
- Markdown (documentation chapters)

**Primary Dependencies**:
- **Platform**: Docusaurus 3.x, React 18+, Node.js 20+
- **Robotics**: ROS 2 Humble Hawksbill, Gazebo Fortress, NVIDIA Isaac Sim 2023.1+
- **AI/ML**: OpenAI API (GPT-4, Whisper, text-embedding-3-large), Anthropic Claude API
- **Backend**: FastAPI 0.104+, Qdrant 1.7+ (vector DB), Neon Serverless Postgres
- **Deployment**: GitHub Pages (static site), Render/Fly.io/Railway (backend services)
- **Hardware**: NVIDIA Jetson Orin (optional real-world deployment), Unitree G1/G2 humanoid robot (simulation + optional physical)

**Storage**:
- **Vector DB**: Qdrant Cloud (document embeddings, RAG retrieval)
- **Relational DB**: Neon Serverless Postgres (conversation history, user sessions)
- **Static Assets**: GitHub repository + GitHub Pages CDN
- **Code Examples**: GitHub repository (separate org or submodules)

**Testing**:
- **Documentation**: Manual review + word count validation + link checking
- **Code Examples**: pytest (Python), Google Test (C++), ROS 2 launch tests
- **Platform**: Jest (React components), Playwright (E2E), Lighthouse (performance)
- **RAG Chatbot**: Custom accuracy tests (>90% target), latency tests (<2s), grounding validation
- **CI/CD**: GitHub Actions workflows

**Target Platform**:
- **Primary**: Ubuntu 22.04 LTS (student workstations, development)
- **Secondary**: NVIDIA Jetson Orin (edge deployment)
- **Web**: Modern browsers (Chrome 90+, Firefox 90+, Safari 14+)
- **Hardware**: x86_64 with NVIDIA GPU (RTX 2060+ for Isaac Sim modules)

**Project Type**: Educational platform - hybrid (static site + backend services + code repositories)

**Performance Goals**:
- **Documentation**: Page load <2s (Lighthouse score >90)
- **RAG Chatbot**: Query response <2s, >90% accuracy, 100% refuse off-topic
- **Code Examples**: Execution time <30s for typical exercises
- **Platform**: Build time <5 minutes (Docusaurus)

**Constraints**:
- **Chapter Length**: 700-1500 words (enforced by constitution)
- **Budget**: $70K-90K total project (13 weeks, 3-4 FTE as estimated in gap analysis)
- **Timeline**: 13 weeks to production launch (per gap analysis Phase 1-5 plan)
- **Free Tier Hosting**: RAG backend must fit Render/Fly.io/Railway free tier initially
- **ROS 2 Version Lock**: ROS 2 Humble only (compatibility constraint)
- **GPU Requirements**: Must provide CPU fallback guidance for students without NVIDIA GPUs

**Scale/Scope**:
- **Documentation**: 35 chapters across 5 modules (~25,000 words total)
- **Code Examples**: 30+ runnable repositories (6 per module × 5 modules)
- **Students**: Target 100-500 initial users (scalable to 10K+)
- **RAG Knowledge Base**: ~50-100 documents (chapters + specs + FAQs)
- **Deployment Environments**: 3 (dev, staging/GitHub Pages, production)

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Technical Accuracy (NON-NEGOTIABLE)
- ✅ **PASS**: All module specifications cite authoritative sources (ROS 2 docs, NVIDIA docs, research papers)
- ✅ **PASS**: Capstone module demonstrates source-backed explanations
- ⚠️ **REVIEW REQUIRED**: Modules 1-4 documentation content (prepared but not deployed) must be verified for technical accuracy before publication
- 🔍 **ACTION**: Add technical review checkpoint in Phase 1 before documentation deployment

### Clarity for Target Audience
- ✅ **PASS**: Specifications define clear learning outcomes for intermediate-to-advanced learners
- ✅ **PASS**: Word count limits (700-1500) enforced to maintain focus
- ✅ **PASS**: Prerequisites explicitly stated (Ubuntu 22.04, ROS 2 Humble, Python/C++ knowledge)

### Source-Backed Explanations
- ✅ **PASS**: All specifications include references sections
- ⚠️ **REVIEW REQUIRED**: Code examples (not yet created) must include inline citations for algorithms
- 🔍 **ACTION**: Add citation requirement to code example templates in Phase 2

### Modular Chapter Structure
- ✅ **PASS**: All modules follow consistent structure (7 chapters per module)
- ✅ **PASS**: Each chapter designed as standalone unit with clear objectives
- ✅ **PASS**: Dependencies explicitly stated in specifications

### Executable Code and Reproducible Examples
- ❌ **VIOLATION**: No code examples exist yet (0/30+ required)
- 🔍 **ACTION**: Phase 2 dedicated to creating tested, runnable examples
- 🔍 **ACTION**: Add environment specification requirements (Dockerfile, requirements.txt) to all examples

### Explainability and Step-by-Step Logic
- ✅ **PASS**: Specifications use Given-When-Then acceptance scenarios
- ✅ **PASS**: Capstone documentation demonstrates step-by-step explanations
- ⚠️ **REVIEW REQUIRED**: Modules 1-4 content must demonstrate progressive complexity
- 🔍 **ACTION**: Add pedagogical review in Phase 1

### Quality Gates Compliance
- ✅ **PASS**: Source verification process defined
- ⚠️ **PARTIAL**: Code execution gate not yet established (no examples to test)
- ❌ **VIOLATION**: Peer review process mentioned but not formalized
- ❌ **VIOLATION**: Accessibility check not defined
- 🔍 **ACTION**: Formalize review process in Phase 6 (QA & Validation)
- 🔍 **ACTION**: Add accessibility audit checklist

**Overall Constitution Status**: ⚠️ **CONDITIONAL PASS** - Proceed with Phase 0 research, address violations in subsequent phases

---

## High-Level Roadmap Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                      PROJECT READINESS ROADMAP (13 WEEKS)                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  Phase 0: Foundation & Validation (Week 1)                                  │
│  ├─ Resolve technical unknowns                                              │
│  ├─ Validate existing content technical accuracy                            │
│  └─ Establish quality gates and review processes                            │
│       │                                                                       │
│       ▼                                                                       │
│  Phase 1: Core Curriculum Completion (Weeks 2-3)                            │
│  ├─ Deploy Modules 1-4 documentation (26 chapters)                          │
│  ├─ Technical review and accessibility audit                                │
│  └─ Update sidebars and navigation                                          │
│       │                                                                       │
│       ▼                                                                       │
│  Phase 2: Simulation & AI Stack Integration (Weeks 4-7) [PARALLEL]          │
│  ├─ Branch A: Module 1-2 Code Examples (ROS 2, Gazebo, Unity)              │
│  ├─ Branch B: Module 3 Examples (Isaac Sim, VSLAM, Nav2)                   │
│  └─ Branch C: Setup automation (Docker, install scripts)                    │
│       │                                                                       │
│       ▼                                                                       │
│  Phase 3: VLA & Conversational Robotics (Weeks 7-9) [PARALLEL]             │
│  ├─ Branch A: Module 4 VLA Pipeline Examples                                │
│  └─ Branch B: RAG Chatbot Backend + Frontend Widget                         │
│       │                                                                       │
│       ▼                                                                       │
│  Phase 4: Capstone & Sim-to-Real (Weeks 9-10)                              │
│  ├─ Refine capstone code to fully runnable state                            │
│  ├─ Integration testing across all modules                                  │
│  └─ Jetson deployment automation scripts                                    │
│       │                                                                       │
│       ▼                                                                       │
│  Phase 5: Platform, UI/UX & Chatbot Integration (Weeks 10-11)              │
│  ├─ Integrate RAG chatbot widget into Docusaurus                            │
│  ├─ Platform polish (progress tracking, search, navigation)                 │
│  └─ Deploy RAG backend to Render/Fly.io                                     │
│       │                                                                       │
│       ▼                                                                       │
│  Phase 6: Final QA, Validation & Release (Weeks 12-13)                     │
│  ├─ Beta testing with 5-10 students                                         │
│  ├─ Bug fixes and accessibility improvements                                │
│  ├─ CI/CD pipeline setup                                                    │
│  └─ Production launch checklist                                             │
│                                                                               │
└─────────────────────────────────────────────────────────────────────────────┘

LEGEND:
  [PARALLEL]  = Tasks can be executed concurrently
  CRITICAL PATH = Phase 0 → Phase 1 → Phase 2 → Phase 4 → Phase 6
  OPTIONAL = Jetson physical deployment, video tutorials
```

---

## Phase 0: Foundation & Validation (Week 1)

### Objectives
1. Resolve all technical unknowns and "NEEDS CLARIFICATION" items
2. Validate existing prepared content for technical accuracy
3. Establish quality gates, review processes, and validation checklists
4. Create integration architecture specification
5. Create testing framework specification

### Modules Impacted
- **All Modules** (cross-cutting quality infrastructure)
- **Integration Architecture** (new specification)
- **Testing Framework** (new specification)

### Tasks

#### Task 0.1: Research & Resolve Technical Unknowns
- **Owner**: Systems Architect
- **Effort**: 16 hours
- **Dependencies**: None (can start immediately)
- **Priority**: CRITICAL PATH

**Research Items**:
1. **Qdrant Cloud Configuration**: Free tier limits, embedding dimensions for text-embedding-3-large
2. **Neon Postgres Schema**: Optimal schema for conversation history, session management
3. **Docusaurus RAG Widget Integration**: Best practices for injecting React component into MDX
4. **GitHub Actions Workflow**: Free tier limits for CI/CD, caching strategies
5. **Docker Multi-Stage Builds**: Optimization for ROS 2 + Isaac ROS images (size reduction)
6. **Jetson Orin Deployment**: Minimum JetPack version, CUDA compatibility with Isaac ROS

**Deliverable**: `specs/001-project-readiness/research.md` documenting all findings with citations

#### Task 0.2: Technical Accuracy Validation of Prepared Content
- **Owner**: Subject Matter Expert (Robotics)
- **Effort**: 12 hours
- **Dependencies**: None
- **Priority**: CRITICAL PATH

**Validation Checklist**:
- [ ] Review Module 1 chapters (7 chapters) for ROS 2 Humble accuracy
- [ ] Review Module 2 chapters (7 chapters) for Gazebo Fortress + Unity compatibility
- [ ] Review Module 3 chapters (7 chapters) for Isaac Sim 2023.1+ API correctness
- [ ] Review Module 4 chapters (7 chapters) for OpenAI Whisper + GPT-4 integration accuracy
- [ ] Verify all citations link to authoritative sources
- [ ] Check code snippets for syntax errors (if any embedded)

**Deliverable**: Validation report with corrections list, updated chapter files if needed

#### Task 0.3: Create Integration Architecture Specification
- **Owner**: Principal Architect + Technical Writer
- **Effort**: 20 hours
- **Dependencies**: None (can run in parallel with 0.1)
- **Priority**: CRITICAL PATH

**Specification Content** (specs/000-integration-architecture/spec.md):
1. **System Architecture Diagram** (C4 model - Context + Container levels)
   - Book platform (Docusaurus static site)
   - RAG chatbot (FastAPI backend + Qdrant + Neon + frontend widget)
   - Code examples (GitHub repositories)
   - Module dependencies (ROS 2 → Gazebo → Isaac → VLA pipeline)

2. **Module Dependency Graph** (Directed Acyclic Graph)
   - Module 1 (ROS 2) is prerequisite for Modules 2, 3, 4, 5
   - Module 2 (Simulation) feeds Module 3 (Perception training data)
   - Module 3 (Nav2) integrates with Module 4 (VLA commands)
   - Modules 1-4 culminate in Module 5 (Capstone)

3. **Shared ROS 2 Interfaces Package** (`humanoid_msgs`)
   - Action definitions for VLA pipeline
   - Service definitions for perception queries
   - Message definitions for custom sensor data

4. **Integration Testing Strategy**
   - Contract tests (verify ROS 2 interface compatibility)
   - Integration tests (Module N → Module N+1 data flow)
   - System tests (end-to-end voice command → robot movement)

5. **Data Flow Diagrams** (3 key workflows)
   - Voice command → LLM planning → ROS execution (Module 4 → Module 1)
   - Sensor data → perception → navigation → manipulation (Modules 2 → 3 → 5)
   - Simulation training → real-world deployment (Module 2/3 → Module 5 Jetson)

6. **Version Compatibility Matrix**
   - ROS 2 Humble + Gazebo Fortress + Isaac Sim 2023.1 + Python 3.10 + CUDA 11.8+

**Deliverable**: `specs/000-integration-architecture/spec.md` (6 user stories, 12 FRs, diagrams)

#### Task 0.4: Create Testing Framework Specification
- **Owner**: QA Lead + DevOps Engineer
- **Effort**: 16 hours
- **Dependencies**: None (can run in parallel with 0.1, 0.3)
- **Priority**: CRITICAL PATH

**Specification Content** (specs/007-testing-framework/spec.md):
1. **Unit Testing Strategy**
   - Python: pytest with coverage >80%
   - C++: Google Test with coverage >70%
   - ROS 2 Nodes: launch tests + integration tests

2. **Integration Testing Strategy**
   - Module boundaries: Verify Module N outputs consumed by Module N+1
   - ROS 2 interfaces: Contract tests for `humanoid_msgs` package
   - RAG chatbot: Accuracy tests (>90% target), latency tests (<2s)

3. **System Testing Strategy**
   - End-to-end workflows: Voice → Plan → Execute (Modules 4 → 1 → 5)
   - Simulation validation: Gazebo + Isaac Sim scene loading
   - Platform testing: Docusaurus build, page load times

4. **CI/CD Pipeline Definition** (GitHub Actions)
   - Trigger: On PR to main branch
   - Jobs:
     - Lint (Python: black, flake8; C++: clang-format)
     - Test (pytest, Google Test, launch tests)
     - Build (Docusaurus, Docker images)
     - Deploy (GitHub Pages on merge to main)
   - Caching: pip cache, Docker layers, ROS 2 workspace

5. **Test Data Generation Strategies**
   - Synthetic sensor data (Isaac Sim)
   - Sample voice commands (Whisper)
   - RAG test queries (conceptual, procedural, troubleshooting)

**Deliverable**: `specs/007-testing-framework/spec.md` (5 user stories, 12 FRs, CI/CD workflow diagram)

#### Task 0.5: Establish Quality Gates & Review Processes
- **Owner**: Program Manager + Education Specialist
- **Effort**: 8 hours
- **Dependencies**: Task 0.2 (validation checklist)
- **Priority**: HIGH

**Quality Gate Definitions**:
1. **Technical Accuracy Gate**
   - Minimum 2 authoritative sources per technical claim
   - All code snippets tested in isolated environment
   - Peer review by subject matter expert

2. **Accessibility Gate**
   - WCAG 2.1 AA compliance (alt text, keyboard navigation, screen reader)
   - Lighthouse accessibility score >90
   - Manual test with screen reader (NVDA/JAWS)

3. **Pedagogical Review Gate**
   - Learning objectives clearly stated
   - Progressive complexity (Bloom's taxonomy alignment)
   - Exercises map to learning outcomes

4. **Deployment Readiness Gate**
   - Word count within 700-1500 range
   - All links functional (no 404s)
   - Markdown formatted correctly for Docusaurus
   - No placeholders or TODO markers

**Deliverable**: `docs/quality-gates.md` checklist, review process documentation

### Completion Criteria
- [x] `research.md` created with all technical unknowns resolved
- [x] Module 1-4 content validated for technical accuracy (corrections applied)
- [x] `specs/000-integration-architecture/spec.md` published
- [x] `specs/007-testing-framework/spec.md` published
- [x] Quality gates and review processes documented
- [x] All Phase 0 tasks reviewed and approved by architect

**Phase 0 Duration**: 1 week (5 business days)
**Phase 0 Team**: 3 people (Architect, SME, QA Lead)

---

## Phase 1: Core Curriculum Completion (Weeks 2-3)

### Objectives
1. Deploy Modules 1-4 documentation to make content accessible to students
2. Conduct technical review and accessibility audit of all chapters
3. Update platform navigation (sidebars.js) with chapter links
4. Test Docusaurus build and fix broken links
5. Validate that 80% of curriculum content is now live

### Modules Impacted
- **Module 1**: ROS 2 Fundamentals (7 chapters)
- **Module 2**: Digital Twin - Gazebo & Unity (7 chapters)
- **Module 3**: AI-Robot Brain - Isaac/Nav2 (7 chapters)
- **Module 4**: Vision-Language-Action (7 chapters)
- **Platform**: Docusaurus navigation and build system

### Tasks

#### Task 1.1: Create Module 1-4 Markdown Files
- **Owner**: Technical Writer
- **Effort**: 4 hours (manual creation) OR 30 minutes (if permissions resolved)
- **Dependencies**: Task 0.2 (validation complete)
- **Priority**: CRITICAL PATH (P0 blocker)

**Execution Steps**:
1. Create directory structure:
   ```
   docs/modules/module-01-ros2-fundamentals/
   docs/modules/module-02-digital-twin-gazebo-unity/
   docs/modules/module-03-ai-robot-brain-isaac/
   docs/modules/module-04-vla-vision-language-action/
   ```

2. Copy prepared content for each module (7 chapters each):
   - index.md
   - chapter-01-*.md
   - chapter-02-*.md
   - chapter-03-*.md
   - chapter-04-*.md
   - chapter-05-*.md
   - chapter-06-*.md
   - chapter-07-summary.md

3. Apply corrections from Task 0.2 validation

**Deliverable**: 28 markdown files on disk (4 modules × 7 chapters each)

#### Task 1.2: Update Sidebars Configuration
- **Owner**: Frontend Developer
- **Effort**: 1 hour
- **Dependencies**: Task 1.1 (files created)
- **Priority**: CRITICAL PATH

**Updates to sidebars.js**:
```javascript
{
  type: 'category',
  label: 'Module 1: ROS 2 Fundamentals',
  link: { type: 'doc', id: 'modules/module-01-ros2-fundamentals/index' },
  items: [
    'modules/module-01-ros2-fundamentals/chapter-01-introduction',
    'modules/module-01-ros2-fundamentals/chapter-02-pubsub',
    'modules/module-01-ros2-fundamentals/chapter-03-packages',
    'modules/module-01-ros2-fundamentals/chapter-04-services',
    'modules/module-01-ros2-fundamentals/chapter-05-actions',
    'modules/module-01-ros2-fundamentals/chapter-06-urdf',
    'modules/module-01-ros2-fundamentals/chapter-07-summary',
  ],
},
// Repeat for Modules 2, 3, 4
```

**Deliverable**: Updated `sidebars.js` with all module chapter links

#### Task 1.3: Build Testing and Link Validation
- **Owner**: DevOps Engineer
- **Effort**: 2 hours
- **Dependencies**: Task 1.2 (sidebars updated)
- **Priority**: CRITICAL PATH

**Testing Steps**:
1. Run local Docusaurus build:
   ```bash
   npm run build
   ```

2. Check for build warnings (broken links, missing images)

3. Run link checker:
   ```bash
   npx broken-link-checker http://localhost:3000 --recursive
   ```

4. Fix any broken internal links

5. Verify navigation works (all chapters accessible)

**Deliverable**: Clean build with zero broken links, navigation tested

#### Task 1.4: Technical Review of Deployed Content
- **Owner**: Subject Matter Expert (Robotics)
- **Effort**: 8 hours
- **Dependencies**: Task 1.3 (build passes)
- **Priority**: HIGH

**Review Checklist** (per constitution quality gates):
- [ ] Source verification (all technical claims cited)
- [ ] Code execution (all embedded code snippets valid)
- [ ] Peer review (technical accuracy confirmed)
- [ ] Style check (constitution principles followed)
- [ ] Accessibility check (diagrams labeled, code commented)

**Deliverable**: Review report, corrections applied

#### Task 1.5: Accessibility Audit
- **Owner**: Accessibility Specialist (or Frontend Developer with WCAG training)
- **Effort**: 6 hours
- **Dependencies**: Task 1.3 (build passes)
- **Priority**: HIGH

**Audit Items**:
1. Run Lighthouse accessibility audit (target score >90)
2. Test keyboard navigation (Tab, Enter, Esc)
3. Test with screen reader (NVDA on Windows or VoiceOver on macOS)
4. Verify alt text on all images/diagrams
5. Check color contrast ratios (WCAG AA: 4.5:1 for normal text)
6. Verify heading hierarchy (no skipped levels)

**Deliverable**: Accessibility report, fixes applied, WCAG 2.1 AA compliance achieved

#### Task 1.6: Deploy to GitHub Pages
- **Owner**: DevOps Engineer
- **Effort**: 1 hour
- **Dependencies**: Task 1.4 and 1.5 (reviews pass)
- **Priority**: CRITICAL PATH

**Deployment Steps**:
1. Merge documentation branch to main
2. GitHub Actions workflow triggers build
3. Deploy to GitHub Pages (https://asadaligith.github.io/AI-Humanoid-Robotics-Book/)
4. Verify live site loads correctly
5. Smoke test navigation and chapter content

**Deliverable**: Modules 1-4 live on production site

### Completion Criteria
- [x] 28 chapter files created and validated
- [x] Sidebars configuration updated
- [x] Docusaurus build passes with zero errors
- [x] Technical review completed (all corrections applied)
- [x] Accessibility audit passed (WCAG 2.1 AA)
- [x] Modules 1-4 deployed to GitHub Pages
- [x] 80% of curriculum content now accessible to students

**Phase 1 Duration**: 2 weeks (10 business days)
**Phase 1 Team**: 3 people (Technical Writer, SME, DevOps/Frontend)

---

## Phase 2: Simulation & AI Stack Integration (Weeks 4-7)

### Objectives
1. Create runnable code examples for Modules 1-2 (ROS 2, Gazebo, Unity)
2. Create runnable code examples for Module 3 (Isaac Sim, VSLAM, Nav2)
3. Develop setup automation scripts (Docker, install scripts)
4. Validate all examples execute successfully on Ubuntu 22.04
5. Document prerequisites and troubleshooting guides

### Modules Impacted
- **Module 1**: ROS 2 Fundamentals (6 examples)
- **Module 2**: Digital Twin (3 examples minimum)
- **Module 3**: AI-Robot Brain (2 examples minimum)
- **Infrastructure**: Docker, install scripts, CI/CD

### Task Organization
**PARALLEL EXECUTION**: Tasks 2.1, 2.2, 2.3 can run concurrently with separate engineers

---

#### Task 2.1: Module 1 Code Examples (ROS 2 Fundamentals) [PARALLEL]
- **Owner**: ROS 2 Developer
- **Effort**: 24 hours
- **Dependencies**: Phase 1 complete (documentation live)
- **Priority**: CRITICAL PATH

**Examples to Create** (per Module 1 spec FR-002):
1. **Example 1.1**: Publisher-Subscriber Pattern
   - Topic: `/robot/sensor_data`
   - Publisher: Simulated sensor (10 Hz)
   - Subscriber: Data logger
   - Validation: Subscriber receives messages, logs to file

2. **Example 1.2**: Custom ROS 2 Package
   - Package name: `my_robot_controller`
   - Node: `controller_node` (accepts velocity commands)
   - Build: `colcon build`
   - Validation: Package builds without errors

3. **Example 1.3**: Service Server-Client
   - Service: `/add_two_ints` (std_srvs/AddTwoInts)
   - Server: Adds two integers, returns sum
   - Client: Sends request, prints response
   - Validation: Client receives correct sum

4. **Example 1.4**: Action Server-Client
   - Action: `/fibonacci` (action_tutorials_interfaces/Fibonacci)
   - Server: Generates Fibonacci sequence
   - Client: Sends goal, monitors progress, receives result
   - Validation: Client receives full sequence

5. **Example 1.5**: URDF Robot Description
   - Robot: Humanoid with 10+ links (torso, head, arms, legs)
   - Joints: Revolute (shoulders, elbows, hips, knees)
   - Visualization: `ros2 launch urdf_tutorial display.launch.py`
   - Validation: Robot loads in RViz2, joints movable

6. **Example 1.6**: TF2 Transform Broadcaster
   - Frames: `world → base_link → camera_link`
   - Broadcaster: Publishes transforms
   - Listener: Queries transform `world → camera_link`
   - Validation: Transform computed correctly

**File Structure** (per example):
```
examples/module-01-ros2-fundamentals/
├── example-01-pubsub/
│   ├── README.md
│   ├── publisher.py
│   ├── subscriber.py
│   ├── package.xml
│   └── setup.py
├── example-02-package/
│   ├── README.md
│   ├── my_robot_controller/
│   │   ├── controller_node.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── test/
│       └── test_controller.py
... (examples 3-6 similarly structured)
```

**Requirements per Example**:
- [ ] README.md with setup instructions, dependencies, expected output
- [ ] Executable code (Python 3.10+ or C++17)
- [ ] requirements.txt or package.xml for dependencies
- [ ] Test file (pytest or launch test)
- [ ] Inline citations for algorithms (if applicable)
- [ ] Environment specification (ROS 2 Humble, Ubuntu 22.04)

**Deliverable**: 6 tested, runnable ROS 2 examples with documentation

---

#### Task 2.2: Module 2 Code Examples (Gazebo & Unity) [PARALLEL]
- **Owner**: Simulation Engineer
- **Effort**: 20 hours
- **Dependencies**: Phase 1 complete
- **Priority**: CRITICAL PATH

**Examples to Create** (per Module 2 spec FR-002):
1. **Example 2.1**: Loading Humanoid URDF into Gazebo
   - World: `humanoid_world.sdf` (ground plane + lighting)
   - Robot: Import URDF from Module 1 Example 1.5
   - Physics: Gravity enabled, collision detection
   - Launch: `ros2 launch gazebo_ros gazebo.launch.py world:=humanoid_world.sdf`
   - Validation: Robot spawns, responds to gravity

2. **Example 2.2**: Virtual Sensor Integration
   - Sensors: Depth camera (head), LiDAR (torso), IMU (base)
   - Plugins: `libgazebo_ros_camera.so`, `libgazebo_ros_ray_sensor.so`, `libgazebo_ros_imu_sensor.so`
   - Topics: `/camera/depth/image_raw`, `/scan`, `/imu/data`
   - Validation: Sensor data published, viewable in RViz2

3. **Example 2.3**: Unity Visualization with ROS 2
   - Unity Scene: Minimal humanoid robot imported via URDF Importer
   - ROS-TCP Connector: Unity ↔ ROS 2 bridge
   - Joint State Subscriber: Unity updates robot pose from `/joint_states`
   - Launch: ROS 2 joint state publisher + Unity scene
   - Validation: Unity robot mirrors ROS 2 joint commands

**File Structure**:
```
examples/module-02-digital-twin/
├── example-01-gazebo-humanoid/
│   ├── README.md
│   ├── worlds/humanoid_world.sdf
│   ├── launch/spawn_humanoid.launch.py
│   └── urdf/humanoid.urdf (symlink to Module 1)
├── example-02-sensors/
│   ├── README.md
│   ├── worlds/sensor_world.sdf
│   ├── launch/sensors.launch.py
│   └── rviz/sensor_viz.rviz
├── example-03-unity-ros/
│   ├── README.md
│   ├── unity_project/ (Unity scene files)
│   ├── ros2_bridge/joint_publisher.py
│   └── setup_instructions.md (Unity Robotics Hub installation)
```

**Deliverable**: 3 tested simulation examples with Gazebo + Unity integration

---

#### Task 2.3: Module 3 Code Examples (Isaac Sim, VSLAM, Nav2) [PARALLEL]
- **Owner**: Computer Vision Engineer
- **Effort**: 30 hours
- **Dependencies**: Phase 1 complete
- **Priority**: CRITICAL PATH

**Examples to Create** (per Module 3 spec FR-005):
1. **Example 3.1**: Isaac Sim VSLAM with Simulated Camera
   - Scene: Isaac Sim warehouse environment
   - Robot: Humanoid USD model with RealSense camera
   - VSLAM: Isaac ROS `visual_slam` package
   - Output: Odometry (`/visual_slam/tracking/odometry`), map features
   - Validation: VSLAM tracks camera pose, map builds in RViz2

2. **Example 3.2**: Perception Pipeline (Object Detection + Depth)
   - Scene: Isaac Sim with target objects (cubes, cylinders)
   - Nodes: Isaac ROS DOPE (object pose) + stereo depth estimation
   - Topics: `/dope/pose`, `/depth/image`
   - Validation: Objects detected, 6D poses published, depth map accurate

**GPU Requirements Notice**:
- All examples include GPU requirement check script
- Provide CPU fallback instructions (use pre-recorded data, cloud GPU alternatives)
- Document minimum GPU: NVIDIA RTX 2060 (compute capability 7.5)

**File Structure**:
```
examples/module-03-isaac-brain/
├── example-01-vslam/
│   ├── README.md
│   ├── isaac_sim_scene/warehouse.usd
│   ├── launch/vslam.launch.py
│   ├── scripts/check_gpu.py
│   └── fallback/recorded_data.bag (for CPU users)
├── example-02-perception/
│   ├── README.md
│   ├── isaac_sim_scene/objects.usd
│   ├── launch/perception_pipeline.launch.py
│   ├── config/dope_config.yaml
│   └── scripts/check_gpu.py
```

**Deliverable**: 2 tested Isaac ROS examples with GPU checks and CPU fallbacks

---

#### Task 2.4: Setup Automation Scripts [PARALLEL]
- **Owner**: DevOps Engineer
- **Effort**: 16 hours
- **Dependencies**: None (can start in parallel with 2.1-2.3)
- **Priority**: HIGH

**Scripts to Create**:
1. **install-ros2-humble.sh**: Automated ROS 2 Humble installation
   - Detects Ubuntu version (must be 22.04)
   - Adds ROS 2 apt repository
   - Installs `ros-humble-desktop-full`
   - Configures environment (`source /opt/ros/humble/setup.bash`)
   - Validation: `ros2 --version` returns Humble

2. **install-gazebo-fortress.sh**: Automated Gazebo Fortress installation
   - Adds Gazebo apt repository
   - Installs `gz-fortress`
   - Installs ROS 2 Gazebo bridge packages
   - Validation: `gz sim --version` returns Fortress

3. **install-isaac-sim.sh**: Isaac Sim installation guide (not automated due to Omniverse Launcher requirement)
   - GPU check (NVIDIA driver, CUDA version)
   - Omniverse Launcher download instructions
   - Isaac Sim installation steps
   - Environment variables setup
   - Validation: Isaac Sim launches successfully

4. **install-all.sh**: Orchestrator script
   - Runs all installation scripts in sequence
   - Error handling (stops on first failure)
   - Progress logging
   - Final validation report

**Deliverable**: 4 tested installation scripts in `scripts/setup/`

---

#### Task 2.5: Dockerfiles for Reproducibility [PARALLEL]
- **Owner**: DevOps Engineer
- **Effort**: 24 hours
- **Dependencies**: Tasks 2.1-2.3 (examples created)
- **Priority**: HIGH

**Docker Images to Create**:
1. **Dockerfile.ros2-base**: ROS 2 Humble base image
   - Base: `ubuntu:22.04`
   - Installs ROS 2 Humble
   - Size target: <2GB

2. **Dockerfile.gazebo**: Gazebo Fortress + ROS 2
   - Base: `Dockerfile.ros2-base`
   - Installs Gazebo Fortress
   - Includes Module 1-2 examples
   - Size target: <3GB

3. **Dockerfile.isaac-ros**: Isaac ROS + dependencies (GPU required)
   - Base: `nvcr.io/nvidia/isaac-ros:humble` (NVIDIA official image)
   - Includes Module 3 examples
   - GPU passthrough required (`--gpus all`)
   - Size target: <10GB (large due to CUDA)

**Multi-Stage Build Optimization**:
- Use multi-stage builds to minimize final image size
- Cache pip and apt dependencies
- Clean up build artifacts

**Deliverable**: 3 Docker images published to GitHub Container Registry or Docker Hub

---

#### Task 2.6: Validation and Documentation
- **Owner**: Technical Writer + DevOps
- **Effort**: 12 hours
- **Dependencies**: Tasks 2.1-2.5 complete
- **Priority**: CRITICAL PATH

**Validation Steps**:
1. Fresh Ubuntu 22.04 VM setup
2. Run `install-all.sh` script
3. Execute all Module 1-2-3 examples
4. Verify Docker images build and run
5. Document any issues encountered

**Documentation to Create**:
- `docs/getting-started/prerequisites.md`: Hardware requirements, software versions
- `docs/getting-started/installation.md`: Step-by-step setup guide
- `docs/troubleshooting/common-issues.md`: FAQ for setup problems

**Deliverable**: Validated setup process, troubleshooting documentation

### Completion Criteria
- [x] 11 code examples created and tested (6 + 3 + 2)
- [x] 4 installation scripts validated on clean Ubuntu 22.04
- [x] 3 Docker images built and published
- [x] All examples documented with README, prerequisites, expected output
- [x] Troubleshooting guide published
- [x] GPU requirement checks implemented with CPU fallbacks

**Phase 2 Duration**: 4 weeks (20 business days)
**Phase 2 Team**: 3-4 people (ROS Dev, Simulation Engineer, CV Engineer, DevOps)

---

## Phase 3: VLA & Conversational Robotics (Weeks 7-9)

### Objectives
1. Create Module 4 VLA pipeline example (Whisper → LLM → ROS execution)
2. Implement RAG chatbot backend (FastAPI + Qdrant + Neon + OpenAI)
3. Develop chatbot frontend widget (React component)
4. Validate RAG accuracy (>90% target) and latency (<2s)
5. Prepare deployment scripts for backend services

### Modules Impacted
- **Module 4**: Vision-Language-Action (1 end-to-end example)
- **RAG Chatbot**: Complete backend + frontend implementation
- **Platform**: Docusaurus integration preparation

### Task Organization
**PARALLEL EXECUTION**: Tasks 3.1 and 3.2 can run concurrently

---

#### Task 3.1: Module 4 VLA Pipeline Example [PARALLEL]
- **Owner**: AI/Robotics Engineer
- **Effort**: 24 hours
- **Dependencies**: Phase 2 complete (ROS 2 examples available)
- **Priority**: CRITICAL PATH

**Example to Create** (per Module 4 spec FR-006):
**Example 4.1**: End-to-End VLA Pipeline in Simulation
- **Components**:
  1. **Voice Input**: Microphone capture OR pre-recorded audio file
  2. **Whisper Transcription**: OpenAI Whisper (base model) transcribes to text
  3. **LLM Planning**: Anthropic Claude API converts text → action plan JSON
  4. **ROS Executor**: Parses action plan, invokes ROS 2 action clients
  5. **Simulation**: Gazebo humanoid executes actions (navigate, pick, place)

- **Workflow**:
  ```
  User speaks: "Pick up the red cube"
    → Whisper: "Pick up the red cube"
    → Claude: {"actions": [{"type": "navigate_to", "target": "red_cube"}, {"type": "pick", "object": "red_cube"}]}
    → ROS Executor: Calls /navigate_to action, then /pick action
    → Gazebo: Robot moves to cube, grasps cube
  ```

- **Validation**:
  - [ ] Voice transcription accuracy >85% (tested with 10 sample commands)
  - [ ] LLM generates valid action plans (tested with 10 commands)
  - [ ] ROS executor invokes correct actions in sequence
  - [ ] Simulation completes task successfully (>60% success rate)
  - [ ] End-to-end latency <15 seconds

**File Structure**:
```
examples/module-04-vla/
├── example-01-vla-pipeline/
│   ├── README.md
│   ├── whisper_node.py (voice → text)
│   ├── llm_planner.py (text → action plan)
│   ├── ros_executor.py (action plan → ROS actions)
│   ├── gazebo_world/vla_demo.sdf
│   ├── launch/vla_pipeline.launch.py
│   ├── config/claude_api.yaml (API key placeholder)
│   ├── test_audio/sample_commands/ (10 WAV files)
│   └── validation/test_vla.py (pytest suite)
```

**Cost Warning**:
- Document LLM API costs (Anthropic Claude API pricing)
- Provide free tier alternatives (open-source LLMs: LLaMA, Mistral)

**Deliverable**: 1 tested end-to-end VLA example with validation suite

---

#### Task 3.2: RAG Chatbot Backend Implementation [PARALLEL]
- **Owner**: Backend Developer
- **Effort**: 24 hours
- **Dependencies**: Phase 0 research complete (Qdrant config, Neon schema)
- **Priority**: CRITICAL PATH (P0 blocker from gap analysis)

**Backend Architecture** (per specs/001-rag-chatbot/spec.md):
1. **FastAPI Application** (`src/chatbot/backend/`)
   - Endpoint: `POST /api/chat` (query, session_id)
   - Endpoint: `GET /api/health` (health check)
   - Authentication: Optional API key for rate limiting

2. **Document Ingestion Pipeline** (`src/chatbot/ingestion/`)
   - Input: Markdown files from `docs/modules/`
   - Processing:
     - Parse markdown → chunks (500 tokens each)
     - Generate embeddings (OpenAI text-embedding-3-large)
     - Store in Qdrant vector DB (collection: `robotics_docs`)
   - Metadata: chapter title, module, section, URL

3. **RAG Retrieval** (`src/chatbot/rag/`)
   - Query → Embedding → Qdrant search (top-k=5)
   - Context assembly: Concatenate retrieved chunks
   - LLM prompt: System prompt + context + user query
   - OpenAI API call (GPT-4 or GPT-3.5-turbo)
   - Response filtering: Refuse off-topic queries

4. **Conversation History** (`src/chatbot/db/`)
   - Neon Postgres schema:
     ```sql
     CREATE TABLE conversations (
       id UUID PRIMARY KEY,
       session_id VARCHAR(255),
       user_query TEXT,
       assistant_response TEXT,
       retrieved_chunks JSONB,
       created_at TIMESTAMP DEFAULT NOW()
     );
     ```
   - SQLAlchemy ORM models

5. **Configuration** (`src/chatbot/config/`)
   - Environment variables (.env):
     - `OPENAI_API_KEY`
     - `QDRANT_URL`, `QDRANT_API_KEY`
     - `NEON_DATABASE_URL`
   - Settings management (Pydantic BaseSettings)

**File Structure**:
```
src/chatbot/
├── backend/
│   ├── main.py (FastAPI app)
│   ├── routes/chat.py (endpoints)
│   ├── models/request.py (Pydantic models)
│   └── tests/test_api.py (pytest)
├── ingestion/
│   ├── ingest_docs.py (document processing)
│   ├── chunking.py (text chunking logic)
│   └── embeddings.py (OpenAI embedding client)
├── rag/
│   ├── retriever.py (Qdrant search)
│   ├── generator.py (LLM response generation)
│   └── prompt_templates.py (system prompts)
├── db/
│   ├── models.py (SQLAlchemy ORM)
│   └── connection.py (database connection)
├── config/
│   ├── settings.py (Pydantic settings)
│   └── .env.example (template)
└── requirements.txt
```

**Testing**:
- [ ] Unit tests: Retriever returns relevant chunks (>90% precision)
- [ ] Integration tests: End-to-end query → response
- [ ] Accuracy tests: 20 test queries → evaluate response correctness
- [ ] Latency tests: p95 latency <2 seconds
- [ ] Grounding tests: 100% refuse off-topic queries

**Deliverable**: Fully functional RAG backend with >90% accuracy, <2s latency

---

#### Task 3.3: RAG Chatbot Frontend Widget [PARALLEL]
- **Owner**: Frontend Developer
- **Effort**: 16 hours
- **Dependencies**: Task 3.2 (backend API available)
- **Priority**: CRITICAL PATH

**Frontend Widget** (`src/components/ChatWidget/`):
1. **React Component** (TypeScript)
   - State management: Query input, conversation history, loading state
   - API integration: Fetch from `/api/chat`
   - UI:
     - Floating chat button (bottom-right corner)
     - Chat panel (expandable/collapsible)
     - Message list (user + assistant messages)
     - Input field with send button
     - Loading indicator during API call

2. **Styling** (CSS Modules or Tailwind CSS)
   - Matches Docusaurus theme (dark mode support)
   - Responsive design (mobile-friendly)
   - Accessibility: ARIA labels, keyboard navigation

3. **Error Handling**:
   - Display error message if API fails
   - Retry button
   - Offline indicator

**File Structure**:
```
src/components/ChatWidget/
├── index.tsx (main component)
├── ChatButton.tsx (floating button)
├── ChatPanel.tsx (message list + input)
├── Message.tsx (individual message bubble)
├── api.ts (fetch wrapper for /api/chat)
├── styles.module.css (component styles)
└── tests/ChatWidget.test.tsx (Jest + React Testing Library)
```

**Integration with Docusaurus**:
- Add widget to `src/theme/Root.tsx` (global theme wrapper)
- Widget appears on all pages
- Session ID persists in localStorage

**Deliverable**: Functional chat widget ready for Docusaurus integration

---

#### Task 3.4: RAG Accuracy Validation
- **Owner**: ML Engineer + QA
- **Effort**: 12 hours
- **Dependencies**: Task 3.2 (backend complete)
- **Priority**: HIGH

**Validation Dataset** (20 test queries across 3 categories):
1. **Conceptual Questions** (7 queries)
   - "What is ROS 2?"
   - "Explain SLAM in robotics"
   - "What is the difference between URDF and SDF?"

2. **Procedural Questions** (7 queries)
   - "How do I install ROS 2 Humble?"
   - "How to create a ROS 2 publisher?"
   - "How to spawn a robot in Gazebo?"

3. **Troubleshooting Questions** (6 queries)
   - "Why is my Gazebo simulation slow?"
   - "Error: 'No module named rclpy'"
   - "How to fix GPU not detected in Isaac Sim?"

**Evaluation Metrics**:
1. **Accuracy**: Human evaluator rates response correctness (1-5 scale)
   - Target: Average score ≥4.5/5 (90%)

2. **Grounding**: Verify response cites correct source chunks
   - Target: 100% of responses include source citations

3. **Off-Topic Refusal**: Test with 5 off-topic queries
   - "What's the weather today?"
   - "Tell me a joke"
   - Target: 100% refusal rate

4. **Latency**: Measure p95 response time
   - Target: <2 seconds

**Deliverable**: Validation report, adjustments to prompts/retrieval if needed

---

#### Task 3.5: Deployment Preparation
- **Owner**: DevOps Engineer
- **Effort**: 8 hours
- **Dependencies**: Tasks 3.2, 3.3 complete
- **Priority**: HIGH

**Deployment Scripts**:
1. **Dockerfile** for backend:
   ```dockerfile
   FROM python:3.10-slim
   WORKDIR /app
   COPY requirements.txt .
   RUN pip install -r requirements.txt
   COPY src/chatbot /app/chatbot
   CMD ["uvicorn", "chatbot.backend.main:app", "--host", "0.0.0.0", "--port", "8000"]
   ```

2. **docker-compose.yml** (local testing):
   ```yaml
   services:
     backend:
       build: .
       ports: ["8000:8000"]
       env_file: .env
   ```

3. **Deployment guide** (`docs/deployment/rag-backend.md`):
   - Render/Fly.io/Railway deployment steps
   - Environment variable configuration
   - Database migration commands
   - Health check verification

**Deliverable**: Deployment scripts and documentation ready for Phase 5

### Completion Criteria
- [x] Module 4 VLA example created and validated (>60% success rate)
- [x] RAG backend implemented with >90% accuracy, <2s latency
- [x] Frontend widget created and tested
- [x] Validation report confirms quality targets met
- [x] Deployment scripts prepared

**Phase 3 Duration**: 3 weeks (15 business days)
**Phase 3 Team**: 3 people (AI/Robotics Engineer, Backend Dev, Frontend Dev)

---

## Phase 4: Capstone & Sim-to-Real (Weeks 9-10)

### Objectives
1. Refine capstone code to fully runnable, reproducible state
2. Integrate all modules (1-4) into capstone workflow
3. Create integration tests across module boundaries
4. Develop Jetson deployment automation scripts
5. Validate end-to-end capstone demo

### Modules Impacted
- **Module 5**: Capstone Autonomous Humanoid
- **Integration**: Cross-module data flows
- **Deployment**: Jetson Orin automation

### Tasks

#### Task 4.1: Capstone Code Refactoring
- **Owner**: Robotics Engineer
- **Effort**: 16 hours
- **Dependencies**: Phase 2-3 complete (all module examples available)
- **Priority**: CRITICAL PATH

**Refactoring Goals**:
1. **Modularity**: Separate perception, planning, navigation, manipulation into independent ROS 2 packages
2. **Configuration**: Externalize parameters (YAML config files)
3. **Error Handling**: Graceful failure recovery (retry logic, fallback behaviors)
4. **Logging**: Structured logging (ROS 2 logger + custom metrics topics)

**File Structure Cleanup**:
```
examples/module-05-capstone/
├── humanoid_bringup/ (launch files)
├── humanoid_perception/ (YOLOv8, depth processing)
├── humanoid_navigation/ (Nav2 integration)
├── humanoid_manipulation/ (MoveIt 2 integration)
├── humanoid_vla/ (voice commands, LLM planning)
├── config/ (YAML parameters)
├── launch/ (orchestration launch files)
└── tests/ (integration tests)
```

**Deliverable**: Refactored capstone code with clear module boundaries

#### Task 4.2: Integration Testing
- **Owner**: QA Engineer
- **Effort**: 24 hours
- **Dependencies**: Task 4.1 (refactored code)
- **Priority**: CRITICAL PATH

**Integration Test Scenarios** (per specs/007-testing-framework/spec.md):
1. **Module 1 → Module 5**: ROS 2 pub/sub communication
   - Test: Publish sensor data → Capstone consumes data
   - Validation: Data received correctly, no message loss

2. **Module 2 → Module 3**: Simulation → Perception
   - Test: Gazebo depth camera → Isaac ROS perception pipeline
   - Validation: Objects detected in simulated scene

3. **Module 3 → Module 4**: Perception → VLA Planning
   - Test: Object pose → LLM generates "pick object" plan
   - Validation: Plan references detected object correctly

4. **Module 4 → Module 5**: VLA → Capstone Execution
   - Test: Voice command → Full fetch-and-deliver task
   - Validation: Robot completes task autonomously

**Test Framework**:
- Use `launch_pytest` for ROS 2 integration tests
- Mock external services (LLM API) with pre-recorded responses
- Automated execution in CI/CD

**Deliverable**: Integration test suite with 10+ test cases, all passing

#### Task 4.3: Jetson Deployment Automation
- **Owner**: Embedded Systems Engineer
- **Effort**: 30 hours
- **Dependencies**: Task 4.1 (capstone code ready)
- **Priority**: HIGH (P2 from gap analysis, but spec promises it)

**Automation Scripts**:
1. **jetson-setup.sh**: Initial Jetson Orin configuration
   - Install JetPack (minimum version check)
   - Install CUDA, cuDNN (version validation)
   - Install ROS 2 Humble (ARM64 build)
   - Install Isaac ROS packages (recompile if needed)

2. **deploy-capstone.sh**: Deploy capstone to Jetson
   - Transfer Docker image OR build on Jetson
   - Configure environment variables
   - Start services (perception, navigation, VLA)
   - Health check all nodes

3. **Hardware Validation** (`scripts/jetson/validate_hw.sh`):
   - GPU check (CUDA available, driver version)
   - Camera check (RealSense detection)
   - Network check (LLM API connectivity)
   - Actuator check (robot motors responsive)

**Docker Image for Jetson**:
- Base: `nvcr.io/nvidia/l4t-base:r35.2.1` (Jetson Linux)
- Multi-stage build optimized for ARM64
- Size target: <8GB

**Deliverable**: Jetson deployment scripts, Docker image, validation suite

#### Task 4.4: Sim-to-Real Transfer Guide
- **Owner**: Robotics Researcher + Technical Writer
- **Effort**: 12 hours
- **Dependencies**: Task 4.3 (Jetson deployment tested)
- **Priority**: MEDIUM

**Guide Content** (`docs/guides/sim-to-real-transfer.md`):
1. **Physics Parameter Tuning**:
   - Match Gazebo friction to real-world measurements
   - Calibrate sensor noise models
   - Adjust PID controller gains

2. **Domain Randomization**:
   - Vary lighting conditions in simulation
   - Randomize object textures
   - Add sensor noise (Gaussian, salt-and-pepper)

3. **Sensor Calibration**:
   - RealSense camera intrinsic/extrinsic calibration
   - IMU bias removal
   - Lidar offset correction

4. **Reality Gap Mitigation**:
   - Collect real-world data, fine-tune perception models
   - Use sim-to-real transfer learning techniques
   - Document expected performance degradation (simulation: 80% accuracy → real: 60%)

**Deliverable**: Comprehensive sim-to-real guide with checklists

#### Task 4.5: End-to-End Capstone Validation
- **Owner**: Program Manager + Robotics Engineer
- **Effort**: 12 hours
- **Dependencies**: Tasks 4.1-4.4 complete
- **Priority**: CRITICAL PATH

**Validation Checklist**:
- [ ] Simulation demo: Voice command → Robot fetches object (>60% success rate, 10 trials)
- [ ] Jetson deployment: All services start successfully, health checks pass
- [ ] Integration tests: All 10+ tests passing
- [ ] Documentation: Troubleshooting guide covers common errors
- [ ] Performance: End-to-end latency <30 seconds (simulation)

**Deliverable**: Validation report, capstone demo ready for showcase

### Completion Criteria
- [x] Capstone code refactored and modular
- [x] Integration tests created and passing
- [x] Jetson deployment automation tested
- [x] Sim-to-real guide published
- [x] End-to-end capstone demo validated

**Phase 4 Duration**: 2 weeks (10 business days)
**Phase 4 Team**: 3-4 people (Robotics Engineer, QA, Embedded Engineer, Researcher)

---

## Phase 5: Platform, UI/UX & Chatbot Integration (Weeks 10-11)

### Objectives
1. Integrate RAG chatbot widget into Docusaurus platform
2. Deploy RAG backend to Render/Fly.io/Railway
3. Implement platform enhancements (search, progress tracking, navigation)
4. Conduct usability testing with 3-5 users
5. Optimize platform performance (Lighthouse score >90)

### Modules Impacted
- **Platform**: Docusaurus UI/UX
- **RAG Chatbot**: Production deployment
- **User Experience**: Navigation, search, progress tracking

### Tasks

#### Task 5.1: RAG Backend Deployment
- **Owner**: DevOps Engineer
- **Effort**: 8 hours
- **Dependencies**: Phase 3 Task 3.5 (deployment scripts ready)
- **Priority**: CRITICAL PATH (P0 blocker)

**Deployment Steps**:
1. **Choose Platform**: Render (recommended for free tier + easy setup)
2. **Create Render Service**:
   - Type: Web Service
   - Environment: Docker
   - Plan: Free tier (512MB RAM, shared CPU)
   - Auto-deploy: On push to `main` branch

3. **Configure Environment Variables**:
   - `OPENAI_API_KEY` (from OpenAI dashboard)
   - `QDRANT_URL`, `QDRANT_API_KEY` (from Qdrant Cloud)
   - `NEON_DATABASE_URL` (from Neon console)

4. **Initialize Database**:
   - Run migration: `alembic upgrade head` (create tables)
   - Ingest initial documents: `python src/chatbot/ingestion/ingest_docs.py`

5. **Health Check**:
   - Test endpoint: `curl https://rag-backend.onrender.com/api/health`
   - Validate response: `{"status": "healthy"}`

**Deliverable**: RAG backend live at production URL

#### Task 5.2: Chatbot Widget Integration
- **Owner**: Frontend Developer
- **Effort**: 8 hours
- **Dependencies**: Task 5.1 (backend deployed)
- **Priority**: CRITICAL PATH

**Integration Steps**:
1. **Update API Endpoint**: Point widget to production URL (Render)
2. **Add Widget to Docusaurus Theme**: Modify `src/theme/Root.tsx`
   ```tsx
   import ChatWidget from '@site/src/components/ChatWidget';
   export default function Root({children}) {
     return (
       <>
         {children}
         <ChatWidget apiUrl="https://rag-backend.onrender.com/api" />
       </>
     );
   }
   ```

3. **Test Widget**:
   - Local build: `npm run build && npm run serve`
   - Test queries: Verify responses from production backend
   - Test session persistence (localStorage)

4. **Deploy to GitHub Pages**:
   - Merge to `main` branch
   - GitHub Actions triggers Docusaurus build + deploy
   - Verify widget appears on live site

**Deliverable**: RAG chatbot widget live on production site

#### Task 5.3: Platform Search Enhancement
- **Owner**: Frontend Developer
- **Effort**: 6 hours
- **Dependencies**: None (can run in parallel with 5.1-5.2)
- **Priority**: MEDIUM

**Enhancements**:
1. **Algolia DocSearch** (Docusaurus built-in):
   - Apply for Algolia DocSearch free tier
   - Add Algolia API keys to `docusaurus.config.js`
   - Configure indexing schedule (weekly crawl)

2. **Search Result Relevance**:
   - Boost module documentation in search ranking
   - Add synonyms (e.g., "ROS" → "Robot Operating System")

**Deliverable**: Improved search with Algolia integration

#### Task 5.4: Progress Tracking Feature
- **Owner**: Frontend Developer
- **Effort**: 12 hours
- **Dependencies**: None
- **Priority**: MEDIUM (nice-to-have)

**Feature Design**:
1. **Completion Tracking**:
   - localStorage stores completed chapters (array of IDs)
   - Checkbox on each chapter page: "Mark as completed"

2. **Progress Dashboard**:
   - New page: `/progress`
   - Shows completion % per module
   - Progress bar visualization

3. **Navigation Badges**:
   - Sidebar shows checkmarks next to completed chapters

**Deliverable**: Progress tracking feature (optional, P2 priority)

#### Task 5.5: Performance Optimization
- **Owner**: Frontend Developer + DevOps
- **Effort**: 8 hours
- **Dependencies**: Tasks 5.1-5.4 complete
- **Priority**: HIGH

**Optimization Targets**:
1. **Lighthouse Score >90**:
   - Performance: >90 (optimize images, lazy loading)
   - Accessibility: >90 (WCAG compliance from Phase 1)
   - Best Practices: >90 (HTTPS, security headers)
   - SEO: >90 (meta tags, sitemap)

2. **Bundle Size Reduction**:
   - Code splitting (Docusaurus automatic)
   - Tree-shaking (remove unused dependencies)
   - Image optimization (WebP format, responsive images)

3. **CDN Configuration**:
   - GitHub Pages already uses CDN
   - Add caching headers for static assets

**Deliverable**: Lighthouse report with >90 scores across all categories

#### Task 5.6: Usability Testing
- **Owner**: UX Researcher OR Program Manager
- **Effort**: 12 hours
- **Dependencies**: Tasks 5.1-5.5 complete
- **Priority**: HIGH

**Testing Protocol**:
1. **Recruit 3-5 Users**: Mix of robotics students and professionals
2. **Task Scenarios**:
   - Task 1: Find and complete Module 1 Chapter 2 (pub/sub)
   - Task 2: Use chatbot to ask "How to install ROS 2?"
   - Task 3: Navigate to Module 3 and run an Isaac Sim example

3. **Observation Metrics**:
   - Task completion rate
   - Time to complete tasks
   - Number of navigation errors
   - Chatbot query success rate

4. **Feedback Collection**:
   - Post-task survey (SUS: System Usability Scale)
   - Open-ended questions (What was confusing? What was helpful?)

**Deliverable**: Usability report with recommendations for improvements

### Completion Criteria
- [x] RAG backend deployed to production
- [x] Chatbot widget integrated and functional
- [x] Platform search enhanced (Algolia)
- [x] Progress tracking feature implemented (if time permits)
- [x] Lighthouse score >90 across all categories
- [x] Usability testing completed, feedback incorporated

**Phase 5 Duration**: 2 weeks (10 business days)
**Phase 5 Team**: 2-3 people (Frontend Dev, DevOps, UX Researcher)

---

## Phase 6: Final QA, Validation & Release (Weeks 12-13)

### Objectives
1. Conduct beta testing with 5-10 students
2. Fix critical bugs and address accessibility issues
3. Set up CI/CD pipeline for automated testing and deployment
4. Create instructor guide and grading rubrics
5. Execute final production launch checklist
6. Public release announcement

### Modules Impacted
- **All Modules** (comprehensive validation)
- **Platform** (CI/CD, monitoring)
- **Documentation** (instructor resources)

### Tasks

#### Task 6.1: Beta Testing Program
- **Owner**: Program Manager
- **Effort**: 40 hours (distributed over 1 week)
- **Dependencies**: Phases 1-5 complete
- **Priority**: CRITICAL PATH

**Beta Testing Plan**:
1. **Recruit 5-10 Beta Testers**:
   - Target: Mix of grad students and early-career robotics engineers
   - Prerequisites: Ubuntu 22.04, basic ROS 2 knowledge

2. **Provide Access**:
   - Email with onboarding instructions
   - Pre-survey: Experience level, hardware specs

3. **Test Curriculum**:
   - Week 1: Modules 1-2 (ROS 2, Gazebo)
   - Week 2: Modules 3-4 (Isaac, VLA)
   - Week 3 (optional): Capstone project

4. **Feedback Channels**:
   - GitHub Issues for bug reports
   - Weekly survey for UX feedback
   - Office hours Q&A sessions

5. **Metrics Tracking**:
   - Success rate per module (target: >70%)
   - Average time per chapter
   - Chatbot usage stats (queries per user)
   - Number of bug reports

**Deliverable**: Beta testing report with prioritized bug list

#### Task 6.2: Bug Fixes and Accessibility Improvements
- **Owner**: Development Team (all)
- **Effort**: 40 hours
- **Dependencies**: Task 6.1 (beta testing feedback)
- **Priority**: CRITICAL PATH

**Bug Triage**:
- **P0 (Blocker)**: Prevents module completion (fix immediately)
- **P1 (Critical)**: Major UX issues (fix before launch)
- **P2 (Important)**: Minor issues (fix if time permits)
- **P3 (Nice-to-have)**: Defer to post-launch

**Accessibility Improvements** (from beta feedback):
- Re-run Lighthouse accessibility audit
- Fix any WCAG violations (color contrast, alt text, keyboard nav)
- Test with screen reader (final validation)

**Deliverable**: All P0 and P1 bugs fixed, accessibility score >90

#### Task 6.3: CI/CD Pipeline Setup
- **Owner**: DevOps Engineer
- **Effort**: 12 hours
- **Dependencies**: None (can run in parallel with 6.1)
- **Priority**: HIGH

**GitHub Actions Workflows**:
1. **Workflow: Docusaurus Build and Deploy** (`.github/workflows/deploy.yml`)
   - Trigger: Push to `main` branch
   - Jobs:
     - Lint markdown files
     - Build Docusaurus site
     - Run link checker
     - Deploy to GitHub Pages

2. **Workflow: Backend Tests** (`.github/workflows/backend-tests.yml`)
   - Trigger: Pull request to `main`
   - Jobs:
     - Lint Python code (black, flake8)
     - Run pytest suite
     - Check test coverage (>80%)

3. **Workflow: Code Examples Validation** (`.github/workflows/examples.yml`)
   - Trigger: Weekly cron job
   - Jobs:
     - Build Docker images
     - Run all code examples
     - Report failures to maintainers

**Caching Strategy**:
- Cache `node_modules`, `pip` packages, Docker layers
- Reduce build time from 10 minutes to <3 minutes

**Deliverable**: CI/CD pipelines active, automated deployment working

#### Task 6.4: Instructor Guide and Grading Rubrics
- **Owner**: Education Specialist + Technical Writer
- **Effort**: 12 hours
- **Dependencies**: None (can run in parallel)
- **Priority**: MEDIUM

**Instructor Guide** (`docs/instructor/guide.md`):
1. **Course Structure**: Recommended 12-week syllabus
2. **Learning Objectives**: Per module, mapped to Bloom's taxonomy
3. **Assessment Strategies**: Formative (exercises) + Summative (capstone)
4. **Grading Rubrics**: Point allocation per module (see below)
5. **Troubleshooting**: Common student issues and solutions

**Grading Rubrics** (`docs/instructor/rubrics.md`):
- **Module 1 (100 points)**:
  - Pub/Sub working (20 pts)
  - Package created (15 pts)
  - Service/Action implemented (25 pts)
  - URDF renders (25 pts)
  - Code quality (15 pts)
- **Repeat for Modules 2-4**
- **Capstone (200 points)**:
  - Task success rate (80 pts: >60% = full credit)
  - Code quality and modularity (40 pts)
  - Documentation (40 pts)
  - Demo presentation (40 pts)

**Deliverable**: Instructor guide and rubrics published

#### Task 6.5: Final Production Launch Checklist
- **Owner**: Program Manager
- **Effort**: 8 hours
- **Dependencies**: Tasks 6.1-6.4 complete
- **Priority**: CRITICAL PATH

**Launch Checklist**:
- [ ] All P0 and P1 bugs fixed
- [ ] Beta testing feedback incorporated
- [ ] Accessibility audit passed (WCAG 2.1 AA)
- [ ] Lighthouse scores >90 (Performance, Accessibility, Best Practices, SEO)
- [ ] RAG chatbot accuracy >90%, latency <2s
- [ ] CI/CD pipelines active and passing
- [ ] All 35 chapters deployed and accessible
- [ ] All 30+ code examples tested and documented
- [ ] Instructor guide and rubrics published
- [ ] Legal review: Open-source licenses verified (MIT/Apache 2.0)
- [ ] Privacy policy updated (if collecting user data)
- [ ] Analytics enabled (Google Analytics or privacy-respecting alternative)
- [ ] Monitoring dashboards configured (Render, GitHub Pages uptime)
- [ ] Backup plan for backend outages (fallback to static FAQ)

**Deliverable**: Launch checklist 100% complete, signed off by stakeholders

#### Task 6.6: Public Release Announcement
- **Owner**: Marketing + Program Manager
- **Effort**: 8 hours
- **Dependencies**: Task 6.5 (checklist complete)
- **Priority**: HIGH

**Release Activities**:
1. **Blog Post**: Publish launch announcement on project website
2. **Social Media**: Twitter, LinkedIn posts with demo video/GIFs
3. **Community Outreach**:
   - Post to ROS Discourse, r/robotics subreddit
   - Email robotics education mailing lists
   - Submit to Hacker News, Product Hunt (if applicable)
4. **Press Release**: If affiliated with institution, issue press release
5. **Demo Video**: 3-minute overview of platform + capstone demo

**Metrics to Track Post-Launch**:
- Website traffic (Google Analytics)
- GitHub stars/forks on code examples repository
- Chatbot usage (queries per day)
- Beta tester enrollment for future cohorts

**Deliverable**: Public release executed, initial metrics tracked

### Completion Criteria
- [x] Beta testing completed with 5-10 students
- [x] All P0/P1 bugs fixed
- [x] CI/CD pipeline operational
- [x] Instructor resources published
- [x] Final launch checklist 100% complete
- [x] Public release announcement live
- [x] **🎉 Project at 100% readiness for production use**

**Phase 6 Duration**: 2 weeks (10 business days)
**Phase 6 Team**: Full team (4-5 people)

---

## Module-Wise Task Breakdown Summary

### Module 1: ROS 2 Fundamentals
| Subcomponent | Book Chapter | Lab/Exercise | Capstone Relevance |
|--------------|--------------|--------------|-------------------|
| Pub/Sub Pattern | Chapter 2 | Example 1.1: Publisher-Subscriber | Sensor data communication |
| ROS 2 Packages | Chapter 3 | Example 1.2: Custom Package | Modular code organization |
| Services | Chapter 4 | Example 1.3: Service Server-Client | Object detection query service |
| Actions | Chapter 5 | Example 1.4: Action Server-Client | Navigation action, manipulation action |
| URDF Modeling | Chapter 6 | Example 1.5: Humanoid URDF | Robot description for simulation |
| TF2 Transforms | Chapter 6 | Example 1.6: TF Broadcaster | Coordinate frame management |

### Module 2: Digital Twin (Gazebo & Unity)
| Subcomponent | Book Chapter | Lab/Exercise | Capstone Relevance |
|--------------|--------------|--------------|-------------------|
| Gazebo Simulation | Chapter 2-3 | Example 2.1: Load URDF in Gazebo | Test capstone in simulation |
| Physics Tuning | Chapter 3 | Manual exercise: Adjust friction, gravity | Realistic simulation behavior |
| Virtual Sensors | Chapter 4 | Example 2.2: Depth camera, LiDAR, IMU | Perception data source |
| Unity Visualization | Chapter 5-6 | Example 2.3: Unity ROS bridge | Alternative visualization, demos |

### Module 3: AI-Robot Brain (Isaac/Nav2)
| Subcomponent | Book Chapter | Lab/Exercise | Capstone Relevance |
|--------------|--------------|--------------|-------------------|
| Isaac Sim Setup | Chapter 2 | Manual: Install Isaac Sim, GPU check | Photorealistic simulation |
| Synthetic Data Gen | Chapter 3 | Manual: Generate training images | Train perception models |
| Isaac ROS VSLAM | Chapter 4 | Example 3.1: VSLAM with RealSense | Localization in capstone |
| Perception Pipeline | Chapter 5 | Example 3.2: DOPE object detection | Detect objects to manipulate |
| Nav2 Integration | Chapter 6 | Manual: Configure Nav2 for humanoid | Autonomous navigation |

### Module 4: Vision-Language-Action (VLA)
| Subcomponent | Book Chapter | Lab/Exercise | Capstone Relevance |
|--------------|--------------|--------------|-------------------|
| Whisper Transcription | Chapter 2 | Manual: Transcribe sample audio | Voice input for capstone |
| LLM Planning | Chapter 3 | Manual: Send prompt to Claude API | High-level task planning |
| ROS Executor | Chapter 4 | Manual: Parse action plan → ROS actions | Execute LLM plans |
| End-to-End VLA | Chapter 5-6 | Example 4.1: Voice → Plan → Execute | Full capstone integration |
| Jetson Deployment | Chapter 7 | Manual: Deploy to Jetson (optional) | Real-world robot deployment |

### Module 5: Capstone
*(Already documented, Phase 4 refines implementation)*
| Subcomponent | Book Chapter | Lab/Exercise | Capstone Relevance |
|--------------|--------------|--------------|-------------------|
| System Architecture | Chapter 1 | Manual: Review architecture diagram | Understand full system |
| Perception Integration | Chapter 2 | Code: YOLOv8 + depth processing | Object detection |
| Navigation Integration | Chapter 3 | Code: Nav2 path planning | Move to object location |
| Manipulation Integration | Chapter 4 | Code: MoveIt 2 grasp planning | Pick and place |
| VLA Integration | Chapter 5 | Code: Voice command → execution | User interaction |
| Testing & Benchmarking | Chapter 6-7 | Code: Automated tests, metrics | Validation |

---

## Capstone Milestone Checklist

### Milestone 1: Perception System Functional (Week 9)
- [ ] YOLOv8 object detection running (>80% accuracy on COCO classes)
- [ ] Depth estimation from RealSense or simulated camera
- [ ] Object pose estimation (6D poses published to `/detected_objects`)
- [ ] RViz visualization of detected objects

### Milestone 2: Navigation System Functional (Week 9)
- [ ] Nav2 costmap generation from LiDAR/depth
- [ ] Global path planning (A*/Dijkstra)
- [ ] Local planning (DWB/TEB for humanoid footprint)
- [ ] Robot navigates to goal pose in simulation

### Milestone 3: Manipulation System Functional (Week 10)
- [ ] MoveIt 2 configured for humanoid arms
- [ ] Grasp pose generation (pre-defined per object class)
- [ ] Pick action: Approach, grasp, lift
- [ ] Place action: Move to target, release

### Milestone 4: VLA Integration Complete (Week 10)
- [ ] Voice command transcription (Whisper)
- [ ] LLM generates action plan (Claude API)
- [ ] ROS executor invokes perception, navigation, manipulation
- [ ] End-to-end task completes in <30 seconds (simulation)

### Milestone 5: Sim-to-Real Deployment Ready (Week 11)
- [ ] Jetson deployment scripts tested
- [ ] Hardware validation suite passing
- [ ] Performance benchmarks documented (simulation vs Jetson)
- [ ] Troubleshooting guide covers Jetson-specific issues

### Milestone 6: Final Demo Validated (Week 11)
- [ ] 10 trial runs in simulation (>60% success rate)
- [ ] Demo video recorded (3-5 minutes)
- [ ] Documentation complete (setup, usage, troubleshooting)
- [ ] Capstone ready for student handoff

---

## Chatbot & Platform Plan

### RAG Chatbot Architecture
```
┌─────────────────────────────────────────────────────────────┐
│                     CHATBOT ARCHITECTURE                    │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Frontend (Docusaurus)                                       │
│  ├─ ChatWidget Component (React)                            │
│  │  ├─ User input field                                     │
│  │  ├─ Message history display                              │
│  │  └─ API client (fetch to backend)                        │
│  └─ Session persistence (localStorage)                      │
│       │                                                       │
│       │ HTTPS                                                │
│       ▼                                                       │
│  Backend (FastAPI on Render/Fly.io)                          │
│  ├─ POST /api/chat endpoint                                 │
│  │  ├─ Query preprocessing                                  │
│  │  ├─ Retrieval: Qdrant vector search (top-k=5)          │
│  │  ├─ Context assembly (retrieved chunks)                  │
│  │  ├─ LLM call (OpenAI GPT-4 + system prompt)             │
│  │  └─ Response post-processing (off-topic filter)          │
│  ├─ GET /api/health (monitoring)                            │
│  └─ Conversation logging (Neon Postgres)                    │
│       │                                                       │
│       ▼                                                       │
│  Data Stores                                                 │
│  ├─ Qdrant Cloud (vector DB)                                │
│  │  └─ Collection: robotics_docs (embeddings)               │
│  ├─ Neon Serverless Postgres (relational DB)                │
│  │  └─ Table: conversations (query, response, metadata)     │
│  └─ OpenAI API (embeddings + completions)                   │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### Document Ingestion Pipeline
1. **Source**: Markdown files from `docs/modules/` (35 chapters)
2. **Preprocessing**:
   - Parse markdown → extract text, code snippets, headings
   - Chunk text (500 tokens per chunk with 50-token overlap)
   - Metadata: `{module: "module-01", chapter: "chapter-02", title: "Pub/Sub Pattern", url: "/docs/..."}`
3. **Embedding**:
   - OpenAI text-embedding-3-large (3072 dimensions)
   - Batch processing (100 chunks per API call)
4. **Storage**:
   - Qdrant vector DB (collection: `robotics_docs`)
   - Indexing: HNSW algorithm for fast search

### Query Types and Agent Behaviors
| Query Type | Example | Agent Behavior |
|------------|---------|----------------|
| **Conceptual** | "What is SLAM?" | Retrieve definition from Chapter 3-4, provide concise explanation with source citation |
| **Procedural** | "How to install ROS 2?" | Retrieve setup instructions, provide step-by-step guide with commands |
| **Troubleshooting** | "Error: rclpy not found" | Retrieve troubleshooting guide, suggest solutions (check environment, reinstall) |
| **Off-Topic** | "Tell me a joke" | Refuse politely: "I can only answer questions about robotics and AI. Please ask about the book content." |
| **Ambiguous** | "How do I start?" | Clarifying question: "Do you mean start ROS 2, start Gazebo, or start a new chapter?" |

### Validation of Chatbot Accuracy and Grounding
**Accuracy Tests** (Task 3.4):
- 20 test queries across 3 categories (conceptual, procedural, troubleshooting)
- Human evaluation: Rate response correctness (1-5 scale, target: ≥4.5/5)
- Automated evaluation: Check if response contains keywords from source chunks

**Grounding Validation**:
- Verify response cites correct source chunks (100% grounding rate)
- Display source citations to user: "Source: Module 1, Chapter 2 - Pub/Sub Pattern"

**Off-Topic Refusal**:
- Test with 5 off-topic queries (weather, jokes, recipes)
- Target: 100% refusal rate
- Refusal message includes redirect: "Try asking about ROS 2, Gazebo, or robotics topics."

---

## UI/UX & Platform Plan

### Main Platform Landing Page
**Current State** (from gap analysis):
- Homepage implemented with module cards, progress indicators
- Deployed to GitHub Pages (asadaligith.github.io/AI-Humanoid-Robotics-Book/)

**Enhancements** (Phase 5):
- [ ] Add hero section with demo video (capstone showcase)
- [ ] Add "Get Started" CTA button → Module 1 Chapter 1
- [ ] Add testimonials section (beta tester feedback)
- [ ] Add FAQ section (common questions)

### Book Reading Experience
**Current State**:
- Docusaurus default theme (left sidebar navigation, right TOC)
- Modules 1-4 content live after Phase 1

**Enhancements** (Phase 5):
- [ ] Add "Next Chapter" button at bottom of each page
- [ ] Add estimated reading time per chapter (e.g., "8 min read")
- [ ] Add code syntax highlighting (already supported by Docusaurus)
- [ ] Add copy button on code blocks (Docusaurus plugin)

### Module Navigation
**Current State**:
- Sidebar shows module categories (Module 1-5)
- Collapsed by default, expand on click

**Enhancements** (Phase 5):
- [ ] Add breadcrumb navigation (Home > Module 1 > Chapter 2)
- [ ] Add module progress indicator (3/7 chapters completed)
- [ ] Add search integration (Algolia DocSearch)

### Chatbot Embedding
**Design** (Task 5.2):
- Floating chat button (bottom-right corner, fixed position)
- Click to expand chat panel (400px width, 600px height)
- Dark mode support (matches Docusaurus theme)
- Mobile-responsive (full-screen on small devices)

### Progress Tracking (Student/Reader)
**Feature** (Task 5.4 - Optional P2):
- localStorage tracks completed chapters (`completed: ["module-01/chapter-01", ...]`)
- Checkbox on each chapter page: "Mark as completed"
- Progress dashboard page (`/progress`):
  - Module 1: 3/7 chapters (42%)
  - Module 2: 0/7 chapters (0%)
  - Overall: 3/35 chapters (8%)

---

## Timeline & Prioritization

### Critical Path (Cannot Launch Without)
```
Phase 0 (Week 1): Foundation & Validation
  └─> Phase 1 (Weeks 2-3): Core Curriculum Completion
      └─> Phase 2 (Weeks 4-7): Code Examples (Modules 1-3)
          └─> Phase 4 (Weeks 9-10): Capstone Integration
              └─> Phase 6 (Weeks 12-13): Final QA & Release
```

**Critical Path Duration**: 13 weeks
**Critical Path Blockers**: Any delay in Phases 0, 1, 2, 4, 6 delays launch

### Parallelizable Tasks
**Phase 2** (Weeks 4-7):
- Task 2.1 (Module 1 examples) || Task 2.2 (Module 2 examples) || Task 2.3 (Module 3 examples)
- Task 2.4 (Setup scripts) || Task 2.5 (Docker images)

**Phase 3** (Weeks 7-9):
- Task 3.1 (Module 4 VLA example) || Task 3.2 (RAG backend) || Task 3.3 (RAG frontend)

**Phase 5** (Weeks 10-11):
- Task 5.1 (Backend deployment) || Task 5.3 (Search enhancement) || Task 5.4 (Progress tracking)

**Parallelization Benefit**: Reduces calendar time by ~4 weeks (from 17 weeks sequential to 13 weeks parallel)

### Optional / Stretch Goals (P2-P3)
| Task | Priority | Effort | Defer Condition |
|------|----------|--------|----------------|
| Progress tracking feature (Task 5.4) | P2 | 12 hours | If UX testing shows low demand |
| Video tutorials (not in plan) | P3 | 60 hours | Post-launch enhancement |
| Multi-language support (i18n) | P3 | 40 hours/language | Post-launch, if international demand |
| Physical robot hardware guide | P2 | 20 hours | If no beta testers have Unitree G1/G2 |
| Advanced Jetson optimization | P2 | 16 hours | If free tier Jetson deployment sufficient |

---

## Risk Areas and Mitigation Strategies

| Risk | Likelihood | Impact | Mitigation Strategy |
|------|------------|--------|---------------------|
| **File creation permissions persist** | Medium | High | Manual copy-paste approach (Task 1.1 assumes this). If unresolved, allocate 4 hours instead of 30 minutes. |
| **ROS 2/Gazebo version incompatibilities** | High | High | Create compatibility matrix in Phase 0 research. Test on clean Ubuntu 22.04 VM before each release. |
| **GPU access for students (Module 3)** | High | Medium | Provide CPU fallback instructions (pre-recorded data). Document cloud GPU alternatives (Google Colab with GPU runtime). |
| **LLM API costs prohibitive (Module 4)** | Medium | Medium | Document open-source alternatives (LLaMA, Mistral). Provide mock LLM responses for testing. Add rate limiting to chatbot. |
| **RAG chatbot accuracy <90%** | Medium | High | Iterative prompt engineering in Phase 3. Expand knowledge base with FAQs. Add human feedback loop ("Was this helpful?"). |
| **Beta testers find critical bugs** | High | High | Allocate 40 hours bug fix buffer (Task 6.2). Triage bugs as P0/P1/P2/P3. Defer P2/P3 to post-launch. |
| **Jetson hardware unavailable to students** | High | Low | Make Jetson deployment optional (simulation-only path valid). Provide video demo of Jetson deployment for reference. |
| **Backend free tier limits exceeded** | Medium | Medium | Monitor Render/Fly.io usage. Set up usage alerts. Plan upgrade to paid tier ($7/month) if needed. |
| **Scope creep (video tutorials, i18n)** | Medium | Medium | Strictly enforce P0/P1/P2/P3 prioritization. Defer P3 items to post-launch. Communicate scope boundaries to stakeholders. |
| **Team velocity slower than estimated** | High | High | Use agile sprints (2-week iterations). Re-prioritize weekly based on progress. Cut scope if needed (defer optional features). |

---

## Final Readiness Checklist

### Pre-Launch Validation (100% Required)
- [ ] **Specifications**: All 6 core components (Modules 1-5, RAG chatbot) fully specified
- [ ] **Documentation**: 35 chapters deployed (Modules 1-5)
- [ ] **Code Examples**: 30+ runnable examples tested on Ubuntu 22.04
- [ ] **RAG Chatbot**: Backend deployed, widget integrated, >90% accuracy, <2s latency
- [ ] **Platform**: Lighthouse score >90 (Performance, Accessibility, Best Practices, SEO)
- [ ] **Testing**: Integration tests passing, beta testing completed
- [ ] **CI/CD**: GitHub Actions workflows active, automated deployment working
- [ ] **Accessibility**: WCAG 2.1 AA compliance verified
- [ ] **Legal**: Open-source licenses verified, privacy policy updated
- [ ] **Monitoring**: Uptime monitoring configured, analytics enabled
- [ ] **Instructor Resources**: Guide and rubrics published
- [ ] **Bug Fixes**: All P0 and P1 bugs resolved
- [ ] **Capstone**: End-to-end demo validated (>60% success rate)
- [ ] **Jetson**: Deployment scripts tested (optional but recommended)

### Success Metrics (Post-Launch Tracking)
- **Week 1**: 50+ unique visitors, 10+ chatbot queries
- **Week 4**: 100+ unique visitors, 5+ beta tester enrollments
- **Month 3**: 500+ unique visitors, 50+ chapter completions
- **Month 6**: 1000+ unique visitors, 10+ capstone project submissions

### Governance and Iteration
- **Bi-weekly retrospectives**: Review progress, adjust priorities
- **Monthly stakeholder updates**: Present metrics, gather feedback
- **Post-launch monitoring**: Track user engagement, bug reports, feature requests
- **Continuous improvement**: Release minor updates (bug fixes, content corrections) bi-weekly

---

## Completion Summary

**Project Readiness Upon Plan Completion**:
- **Current**: 42% ready (strong specs, weak implementation)
- **Target**: 100% ready (complete curriculum, functional platform, validated quality)

**Plan Execution Metrics**:
- **Total Duration**: 13 weeks (3.25 months)
- **Team Size**: 3-4 FTE (Full-Time Equivalents)
- **Total Effort**: 700-900 hours (as estimated in gap analysis)
- **Budget**: $70K-90K (assuming $100/hour blended rate)

**Fastest Path to MVP** (if resource-constrained):
- **6-Week MVP**: Phase 0 (1 week) + Phase 1 (2 weeks) + Phase 6 (2 weeks) = Docs-only launch (no code examples, no chatbot)
- **9-Week Enhanced MVP**: Add Phase 2 (4 weeks, code examples) = Docs + examples (no chatbot)
- **13-Week Full Launch**: All phases = Complete platform as specified

**This plan concludes Phase 2 planning. Next steps**:
1. Review and approve this plan
2. Run `/sp.tasks` to generate detailed task breakdown (tasks.md)
3. Run `/sp.implement` to execute implementation

---

**Plan Status**: ✅ Complete
**Branch**: `001-project-readiness`
**Deliverables**:
- `plan.md` (this file)
- `research.md` (to be created in Phase 0)
- `data-model.md` (not applicable - no data model for this meta-planning feature)
- `contracts/` (not applicable - no APIs for this feature)
- `quickstart.md` (not applicable - plan is the deliverable)
