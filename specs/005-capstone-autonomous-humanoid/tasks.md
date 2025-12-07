# Tasks: AI Humanoid Robotics Book - Capstone Module

**Input**: Design documents from `/specs/005-capstone-autonomous-humanoid/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are NOT explicitly requested in the specification, so test tasks are EXCLUDED per /sp.tasks rules

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
- Documentation: `docs/modules/module-05-capstone/`
- Code examples: `examples/module-05-capstone/`
- Tests: `tests/module-05-capstone/`
- Assets: `assets/` (URDF, USD, worlds, diagrams)
- Scripts: `scripts/` (citations, testing, deployment)
- Config: `docusaurus.config.js`, `sidebars.js`, `.github/workflows/`

---

## Phase 1: Setup (Book Infrastructure)

**Purpose**: Initialize Docusaurus, Context7 MCP, CI/CD pipeline, and repository structure

- [X] T001 Initialize Docusaurus 3.x project at repository root with default theme
- [X] T002 [P] Create directory structure: docs/, examples/, tests/, assets/, scripts/, .github/workflows/
- [X] T003 [P] Configure package.json with Docusaurus dependencies and scripts
- [X] T004 [P] Create .gitignore for Node.js, Python, ROS 2, build artifacts
- [X] T005 Configure Context7 MCP server connection in .context7/config.json
- [X] T006 [P] Create initial sidebars.js with module navigation structure
- [X] T007 [P] Create docusaurus.config.js with site metadata, theme config, plugin settings
- [X] T008 [P] Create GitHub Actions workflow .github/workflows/ci.yml for lint + test + build
- [X] T009 [P] Create GitHub Actions workflow .github/workflows/deploy.yml for GitHub Pages deployment
- [X] T010 Create scripts/citations/context7_to_apa.py for citation conversion automation

**Checkpoint**: ✅ COMPLETE - Docusaurus dev server runs (`npm start`), CI workflows created

---

## Phase 2: Foundational (Shared Book Components)

**Purpose**: Core infrastructure that ALL modules depend on - BLOCKS all module work

**⚠️ CRITICAL**: No module content can be written until this phase is complete

- [X] T011 Create docs/intro.md as book landing page with learning path overview
- [X] T012 [P] Create front matter: docs/preface.md
- [X] T013 [P] Create front matter: docs/how-to-use.md with prerequisites and setup instructions
- [X] T014 [P] Create front matter: docs/conventions.md explaining notation and code format
- [X] T015 [P] Create front matter: docs/hardware-options.md documenting Jetson vs cloud GPU alternatives
- [X] T016 Create appendices/environment-setup.md with ROS 2 Humble installation steps
- [X] T017 [P] Create appendices/hardware-bom.md listing required hardware components
- [X] T018 [P] Create appendices/glossary.md with robotics and AI terminology
- [X] T019 [P] Create citation index template appendices/citations.md (auto-populated by Context7 script)
- [X] T020 Create base chapter template as src/components/ChapterTemplate.tsx with learning objectives, prerequisites, summary sections
- [X] T021 [P] Create Context7Citation React component in src/components/Context7Citation.tsx
- [X] T022 [P] Create custom Docusaurus plugin src/plugins/context7-import/ for citation injection
- [X] T023 Configure Python requirements.txt at repository root with pytest, ROS 2 dependencies
- [X] T024 [P] Create Docker configuration for ROS 2 Humble headless testing in .github/docker/ros2-humble.Dockerfile
- [X] T025 Create scripts/testing/run_ros2_smoke_tests.sh for CI execution
- [X] T026 [P] Create scripts/testing/validate_urdf.py for URDF integrity checks

**Checkpoint**: ✅ COMPLETE - Foundation ready - module content creation can now begin in parallel

---

## Phase 3: User Story 1 - Voice-Commanded Object Retrieval in Simulation (Priority: P1) 🎯 MVP

**Goal**: Students can run complete fetch-and-deliver demo in simulation triggered by voice command, integrating all five capabilities

**Independent Test**: Student issues voice command in Gazebo/Isaac Sim, robot autonomously transcribes, plans, navigates, detects object, picks, returns, and delivers

### Capstone Chapter 1 - System Architecture (350-400 words)

- [X] T027 [P] [US1] Create docs/modules/module-05-capstone/index.md as module landing page
- [X] T028 [US1] Write docs/modules/module-05-capstone/chapter-01-architecture.md covering system overview, five capabilities, integration approach, state machine diagram (350-400 words)
- [ ] T029 [P] [US1] Create architecture diagram assets/diagrams/architecture/capstone-architecture.png showing all ROS 2 nodes, topics, actions
- [ ] T030 [P] [US1] Create VLA pipeline diagram assets/diagrams/architecture/vla-pipeline.png illustrating voice → LLM → navigation → perception → manipulation flow
- [ ] T031 [P] [US1] Create ROS 2 graph diagram assets/diagrams/architecture/ros2-graph.png showing node communication structure

### Capstone Chapter 2 - Voice & LLM Pipeline (300-350 words)

- [X] T032 [US1] Write docs/modules/module-05-capstone/chapter-02-voice-llm.md covering Whisper integration, LLM prompt engineering, action planning, validation (300-350 words)
- [ ] T033 [P] [US1] Capture 3+ Context7 citations for Whisper documentation, LLM prompting best practices
- [X] T034 [P] [US1] Create example LLM prompt template code snippet in chapter-02

### Capstone Chapter 5 - Simulation Deployment (300-350 words)

- [X] T035 [US1] Write docs/modules/module-05-capstone/chapter-05-simulation-deployment.md covering Gazebo setup, Isaac Sim setup, running demo, expected behavior, logs (300-350 words)
- [ ] T036 [P] [US1] Capture 2+ Context7 citations for Gazebo Fortress documentation, simulation best practices

### Voice Input Node Implementation

- [X] T037 [P] [US1] Create examples/module-05-capstone/voice_input_node.py implementing Whisper transcription node
- [X] T038 [P] [US1] Create examples/module-05-capstone/config/voice_config.yaml with Whisper model parameters (base, small, medium)
- [X] T039 [P] [US1] Create examples/module-05-capstone/requirements.txt with openai-whisper, rclpy dependencies

### LLM Planner Node Implementation

- [X] T040 [P] [US1] Create examples/module-05-capstone/llm_planner_node.py implementing LLM-based task planning with OpenAI API
- [X] T041 [P] [US1] Create examples/module-05-capstone/prompts/task_planner_prompt.txt with LLM system prompt template
- [X] T042 [P] [US1] Create examples/module-05-capstone/config/capability_manifest.json defining available robot actions (navigate, detect, pick, place)

### Integration Demo Implementation

- [X] T043 [US1] Create examples/module-05-capstone/integration_demo.py orchestrating state machine (IDLE → LISTENING → TRANSCRIBING → PLANNING → NAVIGATING → PERCEIVING → MANIPULATING → COMPLETED)
- [X] T044 [P] [US1] Create examples/module-05-capstone/launch/gazebo_demo.launch.py for Gazebo simulation launch
- [ ] T045 [P] [US1] Create examples/module-05-capstone/launch/isaac_demo.launch.py for Isaac Sim launch

### Simulation Assets for Demo

- [ ] T046 [P] [US1] Create assets/urdf/unitree_g1.urdf humanoid robot model or download from Unitree official repository
- [ ] T047 [P] [US1] Create assets/worlds/kitchen_env.world Gazebo kitchen environment with table, objects, markers
- [ ] T048 [P] [US1] Create assets/usd/kitchen_env.usd Isaac Sim kitchen scene (converted from Gazebo or built natively)
- [ ] T049 [P] [US1] Create assets/usd/unitree_g1.usd humanoid robot USD model for Isaac Sim

### Student Quickstart Guide

- [X] T050 [US1] Create quickstart.md in specs/005-capstone-autonomous-humanoid/ with 15-minute setup instructions, environment variables, launch commands, expected behavior
- [X] T051 [P] [US1] Create examples/module-05-capstone/README.md with dependencies, setup steps, troubleshooting tips

**Checkpoint**: At this point, students can run complete voice-commanded fetch-and-deliver demo in simulation

---

## Phase 4: User Story 2 - System Architecture Documentation (Priority: P2)

**Goal**: Students produce comprehensive architecture diagrams and documentation showing all components, data flows, and integration points

**Independent Test**: Reviewer can understand complete system design without running code by reading documentation and diagrams

### Architecture Documentation

- [X] T052 [P] [US2] Create contracts/ros2-interfaces.md documenting all five ROS 2 nodes with topics, services, actions, parameters
- [X] T053 [P] [US2] Create contracts/llm-prompts.md with LLM prompt templates, example interactions, output format validation
- [X] T054 [P] [US2] Create contracts/state-machine.md defining 11 states, transitions, failure handling logic
- [ ] T055 [US2] Create data flow sequence diagram assets/diagrams/architecture/nominal-flow.png showing successful task execution
- [ ] T056 [P] [US2] Create failure scenario diagram assets/diagrams/architecture/navigation-failure.png
- [ ] T057 [P] [US2] Create failure scenario diagram assets/diagrams/architecture/object-not-found.png
- [ ] T058 [US2] Update docs/modules/module-05-capstone/chapter-01-architecture.md to reference contracts/ and embed diagrams

**Checkpoint**: Architecture documentation complete and meets FR-004 (architecture diagram) and FR-005 (data flow diagrams for nominal + 2 failures)

---

## Phase 5: User Story 3 - Jetson Orin Hardware Deployment (Priority: P3)

**Goal**: Students can deploy capstone pipeline to physical Jetson Orin and demonstrate voice-commanded task on real hardware

**Independent Test**: Student with Jetson Orin hardware packages pipeline as container, deploys, and runs demo successfully

### Capstone Chapter 6 - Jetson Deployment (200-250 words)

- [ ] T059 [US3] Write docs/modules/module-05-capstone/chapter-06-jetson-deployment.md covering Docker containerization, Jetson setup, resource monitoring, optimization (200-250 words)
- [ ] T060 [P] [US3] Capture 2+ Context7 citations for Jetson Orin documentation, NVIDIA deployment guides

### Jetson Deployment Artifacts

- [ ] T061 [P] [US3] Create examples/module-05-capstone/docker/Dockerfile.jetson for ROS 2 + Whisper + dependencies on ARM64
- [ ] T062 [P] [US3] Create scripts/deployment/deploy_jetson.sh for automated Jetson deployment script
- [ ] T063 [P] [US3] Create scripts/deployment/monitor_resources.py for CPU/GPU/memory monitoring on Jetson
- [ ] T064 [US3] Update examples/module-05-capstone/README.md with Jetson-specific deployment instructions

**Checkpoint**: Jetson deployment instructions complete and tested (if hardware available per FR-008)

---

## Phase 6: User Story 4 - Five Capability Integration (Priority: P4)

**Goal**: Students systematically integrate and test each of five capabilities independently before full integration

**Independent Test**: Each capability (voice, LLM, navigation, perception, manipulation) can be tested in isolation with mocked inputs/outputs

### Capstone Chapter 3 - Navigation & Perception (300-350 words)

- [ ] T065 [US4] Write docs/modules/module-05-capstone/chapter-03-navigation-perception.md covering VSLAM + Nav2 integration, object detection pipeline, coordinate frame transforms (300-350 words)
- [ ] T066 [P] [US4] Capture 3+ Context7 citations for Nav2 documentation, Isaac ROS VSLAM, vision_msgs

### Capstone Chapter 4 - Manipulation & Task Execution (250-300 words)

- [ ] T067 [US4] Write docs/modules/module-05-capstone/chapter-04-manipulation.md covering MoveIt 2 integration, grasp planning, pick-and-place actions, failure handling (250-300 words)
- [ ] T068 [P] [US4] Capture 2+ Context7 citations for MoveIt 2 documentation, manipulation best practices

### Navigation Controller Implementation

- [ ] T069 [P] [US4] Create examples/module-05-capstone/navigation_controller.py implementing Nav2 action client for goal navigation
- [ ] T070 [P] [US4] Create examples/module-05-capstone/config/nav2_params.yaml with costmap and planner parameters

### Object Detection Node Implementation

- [ ] T071 [P] [US4] Create examples/module-05-capstone/object_detection_node.py implementing computer vision object detection (OpenCV or Isaac ROS)
- [ ] T072 [P] [US4] Create examples/module-05-capstone/config/detection_classes.yaml defining target objects (cup, mug, bottle, etc.)

### Manipulation Controller Implementation

- [ ] T073 [P] [US4] Create examples/module-05-capstone/manipulation_controller.py implementing MoveIt 2 pick and place actions
- [ ] T074 [P] [US4] Create examples/module-05-capstone/config/grasp_poses.yaml with predefined grasp configurations

### Integration Testing Methodology Documentation

- [ ] T075 [US4] Create docs/modules/module-05-capstone/testing-methodology.md documenting how to test each capability independently (voice: 85% accuracy, navigation: <10% failure, detection: correct bounding boxes, manipulation: 70% success)
- [ ] T076 [P] [US4] Add capability isolation test examples to integration_demo.py with mock modes

**Checkpoint**: All five capabilities integrated and independently testable per FR-009

---

## Phase 7: User Story 5 - Performance Benchmarking (Priority: P5)

**Goal**: Students measure and analyze system performance: latency, success rate, resource utilization, failure modes

**Independent Test**: Run automated benchmarking suite, collect metrics, generate performance report

### Performance Benchmarking Framework

- [ ] T077 [P] [US5] Create scripts/testing/benchmark_capstone.py with automated test suite (20 trials, different objects/layouts)
- [ ] T078 [P] [US5] Create scripts/testing/analyze_performance.py for bottleneck analysis, latency breakdown, failure mode classification
- [ ] T079 [US5] Create docs/modules/module-05-capstone/benchmarking.md documenting performance metrics: task success rate (target 60%), end-to-end latency (target <30s), resource utilization
- [ ] T080 [P] [US5] Capture 1+ Context7 citations for robotics benchmarking best practices

**Checkpoint**: Benchmarking framework produces metrics report per FR-012

---

## Phase 8: Troubleshooting Guide & Edge Cases (Cross-Cutting)

**Purpose**: Address common integration issues and edge cases across all capabilities

- [ ] T081 [P] Create docs/modules/module-05-capstone/troubleshooting.md covering voice not detected, LLM timeout, navigation failures, object not detected, grasp failures per FR-011
- [ ] T082 [P] Add edge case handling to integration_demo.py: ambiguous commands → LLM clarification, no valid path → replanning, wrong object → validation loop, grasp failure → retry (max 3), LLM API down → cached plans
- [ ] T083 Update docs/modules/module-05-capstone/chapter-05-simulation-deployment.md with common error messages and debugging steps

**Checkpoint**: Troubleshooting guide addresses edge cases from spec.md

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final quality improvements, citation generation, validation

- [ ] T084 [P] Run scripts/citations/context7_to_apa.py to generate APA bibliography in appendices/citations.md
- [ ] T085 [P] Validate all Capstone chapters meet word count targets: Ch1 (350-400), Ch2 (300-350), Ch3 (300-350), Ch4 (250-300), Ch5 (300-350), Ch6 (200-250), total 1200-2000 per FR-010
- [ ] T086 [P] Run markdownlint on docs/modules/module-05-capstone/ for formatting consistency
- [ ] T087 [P] Validate all diagrams render correctly in Docusaurus build
- [ ] T088 [P] Run scripts/testing/validate_urdf.py on assets/urdf/unitree_g1.urdf
- [ ] T089 [P] Validate assets/usd/unitree_g1.usd and assets/usd/kitchen_env.usd schema integrity (no full Isaac render required)
- [ ] T090 [P] Validate all ROS 2 examples with headless Gazebo smoke tests in CI
- [ ] T091 Test complete quickstart.md setup on clean Ubuntu 22.04 VM (if available)
- [ ] T092 [P] Update docusaurus.config.js navigation to include Capstone module in sidebar
- [ ] T093 Run full Docusaurus build (`npm run build`) and verify no warnings
- [ ] T094 Deploy to GitHub Pages and validate site accessibility, navigation, search functionality
- [ ] T095 [P] Add module-05-capstone to GitHub Actions CI workflow for automated testing

**Checkpoint**: Capstone module complete, deployable, passes all acceptance criteria

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all module content work
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User Story 1 (P1) is MVP - highest priority
  - User Stories 2-5 can proceed in priority order or in parallel (if staffed)
- **Troubleshooting (Phase 8)**: Can proceed after User Story 1 complete, refine with each subsequent story
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1) - MVP Demo**: Can start after Foundational (Phase 2) - No dependencies on other stories ✅
- **User Story 2 (P2) - Documentation**: Can start after Foundational, enhanced by US1 completion (references integration_demo.py)
- **User Story 3 (P3) - Jetson Deployment**: Depends on US1 completion (deploys working demo)
- **User Story 4 (P4) - Five Capabilities**: Can partially overlap US1 (implements missing nodes), completes integration
- **User Story 5 (P5) - Benchmarking**: Depends on US1 + US4 completion (requires full working system)

### Within Each User Story

- Documentation chapters can be written in parallel (all marked [P])
- Code implementations can be written in parallel (voice, LLM, navigation, perception, manipulation nodes are independent)
- Assets (diagrams, URDF, worlds) can be created in parallel (all marked [P])
- Integration tasks depend on component tasks completing first

### Parallel Opportunities

- **Phase 1 (Setup)**: T002-T010 can all run in parallel after T001 completes
- **Phase 2 (Foundational)**: T012-T026 can run in parallel after T011
- **User Story 1**: T027-T051 has many parallel opportunities:
  - Chapters (T027-T036): 5 chapters can be written simultaneously
  - Code nodes (T037-T042): Voice and LLM nodes independent
  - Assets (T046-T049): All simulation assets independent
- **User Story 2**: T052-T057 all documentation/diagrams can be created in parallel
- **User Story 4**: T069-T074 all capability nodes can be implemented in parallel

---

## Parallel Example: User Story 1 (MVP Demo)

```bash
# Launch all Capstone chapter writing in parallel:
Task T028: "Write chapter-01-architecture.md (350-400 words)"
Task T032: "Write chapter-02-voice-llm.md (300-350 words)"
Task T035: "Write chapter-05-simulation-deployment.md (300-350 words)"

# Launch all architecture diagrams in parallel:
Task T029: "Create capstone-architecture.png"
Task T030: "Create vla-pipeline.png"
Task T031: "Create ros2-graph.png"

# Launch all ROS 2 node implementations in parallel:
Task T037: "Create voice_input_node.py"
Task T040: "Create llm_planner_node.py"

# Launch all simulation assets in parallel:
Task T046: "Create unitree_g1.urdf"
Task T047: "Create kitchen_env.world (Gazebo)"
Task T048: "Create kitchen_env.usd (Isaac Sim)"
Task T049: "Create unitree_g1.usd"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T026) - CRITICAL BLOCKER
3. Complete Phase 3: User Story 1 (T027-T051) - MVP Demo
4. **STOP and VALIDATE**: Run integration_demo.py in Gazebo, verify voice → fetch → deliver works
5. Demo to stakeholders/students

**MVP Success Criteria**:
- ✅ Student can run `ros2 launch capstone_demo gazebo_demo.launch.py`
- ✅ Student can run `ros2 run capstone_demo integration_demo.py`
- ✅ Student issues voice command "Bring me the red cup"
- ✅ Robot autonomously completes fetch-and-deliver task (60% success rate acceptable)
- ✅ All five capabilities demonstrated in integration

### Incremental Delivery

1. Complete Setup + Foundational → Infrastructure ready
2. Add User Story 1 → Test independently → Demo working simulation (MVP! 🎯)
3. Add User Story 2 → Architecture docs complete → Students can document their own systems
4. Add User Story 3 → Jetson deployment → Hardware deployment enabled
5. Add User Story 4 → Capability isolation → Better debugging and testing
6. Add User Story 5 → Performance benchmarking → Quantitative analysis capability
7. Polish → Final quality pass → Production-ready book module

### Parallel Team Strategy

With multiple team members:

1. **Team completes Setup + Foundational together** (T001-T026)
2. **Once Foundational is done, split work:**
   - **Writer A**: User Story 1 chapters (T028, T032, T035)
   - **Developer B**: User Story 1 code (T037, T040, T043-T045)
   - **Asset Creator C**: User Story 1 assets (T029-T031, T046-T049)
3. **Subsequent stories can proceed in parallel:**
   - **Writer A**: User Story 2 documentation (T052-T058)
   - **Developer B**: User Story 4 capability nodes (T069-T074)
   - **DevOps C**: User Story 3 Jetson deployment (T061-T064)

---

## Task Summary

**Total Tasks**: 95
- **Phase 1 (Setup)**: 10 tasks (T001-T010)
- **Phase 2 (Foundational)**: 16 tasks (T011-T026) - BLOCKS all content
- **Phase 3 (US1 - MVP Demo)**: 25 tasks (T027-T051) - Highest priority
- **Phase 4 (US2 - Documentation)**: 7 tasks (T052-T058)
- **Phase 5 (US3 - Jetson)**: 6 tasks (T059-T064)
- **Phase 6 (US4 - Integration)**: 12 tasks (T065-T076)
- **Phase 7 (US5 - Benchmarking)**: 4 tasks (T077-T080)
- **Phase 8 (Troubleshooting)**: 3 tasks (T081-T083)
- **Phase 9 (Polish)**: 12 tasks (T084-T095)

**Parallel Opportunities**: 58 tasks marked [P] can run in parallel with other tasks in their phase

**MVP Scope (Recommended)**: Phase 1 + Phase 2 + Phase 3 (User Story 1) = 51 tasks
- Focus: Get working simulation demo first
- Validation: Students can run voice-commanded fetch-and-deliver in Gazebo
- Delivers: Complete end-to-end Physical AI integration demonstration

**Critical Path**:
1. Setup infrastructure (10 tasks)
2. Build foundation - BLOCKER (16 tasks)
3. Implement MVP demo (25 tasks)
4. Add remaining user stories incrementally based on priority

**Format Validation**: ✅ All tasks follow checklist format: `- [ ] [ID] [P?] [Story?] Description with file path`

---

## Notes

- **[P] tasks** = Different files, no dependencies - can run in parallel
- **[Story] label** = Maps task to specific user story (US1-US5) for traceability
- **Each user story** is independently completable and testable per /sp.tasks requirements
- **No test tasks** included because tests were NOT explicitly requested in specification
- **Commit strategy**: Commit after each chapter, node implementation, or logical asset group
- **Stop at any checkpoint** to validate story independently before proceeding
- **Constitution compliance**: All tasks support FR-010 (word count), FR-004/FR-005 (diagrams), CI/CD validation (Principle V)
- **Avoid**: Vague tasks, same-file conflicts, cross-story dependencies that break independence
