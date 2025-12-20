# Testing Framework Specification

**Feature**: Comprehensive Testing Infrastructure
**Date**: 2025-12-20
**Status**: Draft
**Branch**: `001-project-readiness`

---

## Summary

This specification defines the testing framework for the "Physical AI & Humanoid Robotics" educational platform, establishing testing strategies, tooling, coverage requirements, and CI/CD integration for all platform components (documentation, code examples, RAG chatbot, and integration scenarios).

**Primary Objective**: Implement a multi-layered testing strategy that validates documentation quality, code example correctness, RAG chatbot accuracy, platform performance, and cross-module integration.

**Success Metric**: Achieve >85% test coverage across all components with automated tests running in CI/CD, catching regressions before deployment.

---

## Problem Statement

### Current State
- No testing framework exists (gap analysis: 0% testing infrastructure)
- Code examples not validated for correctness
- Documentation links and word counts not checked
- RAG chatbot accuracy unverified
- No integration tests for cross-module workflows

### Pain Points
1. **Quality Risk**: Broken code examples damage student trust
2. **Regression Risk**: Changes to one module break dependent modules
3. **Performance Risk**: Platform degrades without monitoring
4. **Content Quality**: Technical inaccuracies slip through without validation

### Success Criteria
- ✅ All code examples have passing tests (unit + integration)
- ✅ Documentation quality gates automated (word count, link checking, technical review)
- ✅ RAG chatbot achieves >90% accuracy on validation set
- ✅ Integration tests validate Module 1→5 workflows
- ✅ CI/CD pipeline catches failures before merge
- ✅ Performance budgets enforced (page load <2s, RAG query <2s)

---

## Technical Context

**Language/Version**: Python 3.10+ (pytest), JavaScript ES6+ (Jest), C++ (Google Test), Bash (shellcheck)
**Primary Dependencies**: pytest, Google Test, Jest, Playwright, Lighthouse, pytest-cov, coverage.py
**Testing**: pytest (Python), Google Test (C++), Jest (React), Playwright (E2E), custom validators
**Target Platform**: Ubuntu 22.04 LTS (CI/CD runners), Ubuntu 22.04 (student workstations)

**Constraints**:
- Tests must run in GitHub Actions free tier (2000 minutes/month)
- No GPU required for unit tests (use mocks for Isaac Sim)
- Tests should complete in <10 minutes total (fast feedback)
- Code examples must be testable offline (no API calls in tests)

---

## Testing Pyramid

```
                  ┌─────────────────────┐
                  │   E2E Tests (5%)    │  Playwright, Full User Workflows
                  │   ~10 tests         │  Example: Student completes Module 1→5
                  └─────────────────────┘
                          │
              ┌───────────────────────────┐
              │  Integration Tests (15%)  │  ROS 2 Multi-Node, RAG Pipeline
              │  ~30 tests                │  Example: Module 1 → Module 2 data flow
              └───────────────────────────┘
                          │
          ┌───────────────────────────────────┐
          │      Unit Tests (80%)             │  pytest, Google Test, Jest
          │      ~150 tests                   │  Example: Single function validation
          └───────────────────────────────────┘
```

---

## Testing Strategy by Component

### 1. Documentation Tests

**Tooling**: Custom Python scripts, markdown-link-check, textlint

**Test Cases**:

**T1.1: Word Count Validation**
```python
# tests/docs/test_word_count.py
import pytest
from pathlib import Path
import re

CHAPTER_MIN_WORDS = 700
CHAPTER_MAX_WORDS = 1500

def count_words(markdown_file):
    """Count words excluding front matter, code blocks, and headings"""
    content = Path(markdown_file).read_text()

    # Remove front matter (YAML between ---)
    content = re.sub(r'^---.*?---', '', content, flags=re.DOTALL)

    # Remove code blocks
    content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)

    # Remove inline code
    content = re.sub(r'`[^`]+`', '', content)

    # Remove headings
    content = re.sub(r'^#+\s.*$', '', content, flags=re.MULTILINE)

    # Count words
    words = content.split()
    return len(words)

@pytest.mark.parametrize("chapter", [
    "docs/modules/module-01-ros2-fundamentals/chapter-01-introduction.md",
    "docs/modules/module-01-ros2-fundamentals/chapter-02-pubsub.md",
    # ... all 35 chapters
])
def test_chapter_word_count(chapter):
    """Verify each chapter meets word count requirements (700-1500 words)"""
    word_count = count_words(chapter)

    assert CHAPTER_MIN_WORDS <= word_count <= CHAPTER_MAX_WORDS, \
        f"{chapter} has {word_count} words (expected {CHAPTER_MIN_WORDS}-{CHAPTER_MAX_WORDS})"
```

**T1.2: Link Validation**
```bash
# tests/docs/test_links.sh
#!/bin/bash
# Run markdown-link-check on all docs

find docs/ -name "*.md" -print0 | xargs -0 -n1 markdown-link-check --config .markdown-link-check.json

# .markdown-link-check.json
{
  "ignorePatterns": [
    {
      "pattern": "^http://localhost"
    }
  ],
  "timeout": "10s",
  "retryOn429": true,
  "retryCount": 3
}
```

**T1.3: Technical Accuracy Validation** (Manual Checklist)
```markdown
# tests/docs/technical_review_checklist.md

## Module 1: ROS 2 Fundamentals
- [ ] All ROS 2 commands use `ros2` CLI (not ROS 1 `rosrun`)
- [ ] All examples cite ROS 2 Humble documentation
- [ ] All package names match official ROS 2 packages
- [ ] No deprecated APIs referenced

## Module 2: Gazebo & Unity
- [ ] Gazebo Fortress commands correct (not Gazebo Classic)
- [ ] URDF files conform to ROS 2 standards
- [ ] SDF world files valid

## Module 3: Isaac Sim & VSLAM
- [ ] Isaac Sim version 2023.1+ referenced
- [ ] NVIDIA Isaac ROS package names correct
- [ ] CUDA/TensorRT versions compatible

## Module 4: VLA
- [ ] OpenAI API examples use latest API (not deprecated)
- [ ] Whisper model sizes accurate
- [ ] LangChain patterns follow best practices

## Module 5: Capstone
- [ ] All dependencies from Modules 1-4 integrated
- [ ] Launch files functional
- [ ] Performance metrics realistic
```

---

### 2. Code Example Tests

**Tooling**: pytest (Python), Google Test (C++), ROS 2 launch tests

**Test Cases**:

**T2.1: Python Node Unit Tests**
```python
# examples/module-01-ros2-fundamentals/example-01-pubsub/tests/test_publisher.py
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestPublisher:
    @pytest.fixture(autouse=True)
    def setup_teardown(self):
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_publisher_publishes_message(self):
        """Verify publisher node sends messages on /example_topic"""
        from example_01_pubsub.publisher import ExamplePublisher

        node = ExamplePublisher()
        received_msgs = []

        def callback(msg):
            received_msgs.append(msg.data)

        # Create subscriber to listen
        test_node = Node('test_subscriber')
        subscription = test_node.create_subscription(
            String, '/example_topic', callback, 10
        )

        # Spin for 2 seconds to receive messages
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(test_node)
        executor.spin_once(timeout_sec=2.0)

        assert len(received_msgs) > 0, "No messages received"
        assert "Hello" in received_msgs[0], f"Unexpected message: {received_msgs[0]}"

        node.destroy_node()
        test_node.destroy_node()
```

**T2.2: C++ Node Unit Tests**
```cpp
// examples/module-01-ros2-fundamentals/example-02-services/tests/test_service.cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

class TestAddTwoIntsService : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};

TEST_F(TestAddTwoIntsService, ServiceReturnsCorrectSum) {
    // Create service node (from example)
    auto service_node = std::make_shared<rclcpp::Node>("add_two_ints_server");

    // Create service
    auto service = service_node->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        [](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
            response->sum = request->a + request->b;
        }
    );

    // Create client node
    auto client_node = std::make_shared<rclcpp::Node>("test_client");
    auto client = client_node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    // Wait for service
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

    // Call service
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 5;
    request->b = 7;

    auto future = client->async_send_request(request);

    // Spin until response received
    rclcpp::spin_until_future_complete(client_node, future);

    ASSERT_EQ(future.get()->sum, 12);
}
```

**T2.3: Launch File Tests**
```python
# examples/module-03-ai-robot-brain-isaac/tests/test_perception_launch.py
import pytest
import launch
import launch_ros
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertSequentialStdout

@pytest.mark.launch_test
def generate_test_description():
    """Launch perception nodes and verify they start successfully"""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='module_03_perception',
            executable='yolo_node',
            name='yolo'
        ),
        launch_ros.actions.Node(
            package='module_03_perception',
            executable='depth_estimation_node',
            name='depth'
        ),
        launch_testing.actions.ReadyToTest()
    ])

class TestPerceptionLaunch(unittest.TestCase):
    def test_nodes_start(self, proc_info):
        """Verify both nodes started without errors"""
        proc_info.assertWaitForStartup(timeout=10)

    def test_nodes_publish_topics(self):
        """Verify nodes publish expected topics"""
        # Check /detections topic exists
        topics = subprocess.check_output(['ros2', 'topic', 'list']).decode()
        assert '/detections' in topics
        assert '/depth_cloud' in topics
```

---

### 3. RAG Chatbot Tests

**Tooling**: pytest, FastAPI TestClient, custom accuracy evaluator

**Test Cases**:

**T3.1: RAG Accuracy Validation**
```python
# src/chatbot/tests/test_rag_accuracy.py
import pytest
from fastapi.testclient import TestClient
from src.chatbot.backend.main import app

client = TestClient(app)

# Validation set: 50 question-answer pairs curated by instructors
VALIDATION_SET = [
    {
        "question": "What is the difference between rclpy and rclcpp?",
        "expected_keywords": ["Python", "C++", "rclpy", "rclcpp"],
        "module": "module-01-ros2-fundamentals"
    },
    {
        "question": "How do I create a URDF file?",
        "expected_keywords": ["XML", "robot", "link", "joint"],
        "module": "module-02-digital-twin-gazebo-unity"
    },
    # ... 48 more questions
]

@pytest.mark.parametrize("qa_pair", VALIDATION_SET)
def test_rag_accuracy_on_validation_set(qa_pair):
    """Verify RAG returns relevant answers for known questions"""
    response = client.post("/api/chat", json={
        "message": qa_pair["question"],
        "session_id": "test-validation-session",
        "context": {"current_module": qa_pair["module"]}
    })

    assert response.status_code == 200
    data = response.json()

    # Check if response contains expected keywords
    response_text = data['response'].lower()
    matched_keywords = [kw for kw in qa_pair['expected_keywords'] if kw.lower() in response_text]

    # Require at least 50% keyword match
    match_rate = len(matched_keywords) / len(qa_pair['expected_keywords'])
    assert match_rate >= 0.5, f"Only {match_rate*100:.0f}% keywords matched for: {qa_pair['question']}"

def test_rag_overall_accuracy():
    """Aggregate accuracy across validation set (target: >90%)"""
    correct = 0
    total = len(VALIDATION_SET)

    for qa_pair in VALIDATION_SET:
        response = client.post("/api/chat", json={
            "message": qa_pair["question"],
            "session_id": "test-accuracy-session",
            "context": {"current_module": qa_pair["module"]}
        })

        if response.status_code == 200:
            response_text = response.json()['response'].lower()
            matched_keywords = [kw for kw in qa_pair['expected_keywords'] if kw.lower() in response_text]
            match_rate = len(matched_keywords) / len(qa_pair['expected_keywords'])

            if match_rate >= 0.5:
                correct += 1

    accuracy = correct / total
    assert accuracy >= 0.90, f"RAG accuracy {accuracy*100:.1f}% below target 90%"
```

**T3.2: RAG Latency Tests**
```python
# src/chatbot/tests/test_rag_performance.py
import pytest
import time
from fastapi.testclient import TestClient
from src.chatbot.backend.main import app

client = TestClient(app)

def test_rag_latency_under_2_seconds():
    """Verify RAG query completes in <2 seconds (95th percentile)"""
    latencies = []

    for i in range(20):  # 20 sample queries
        start = time.time()
        response = client.post("/api/chat", json={
            "message": "What is ROS 2?",
            "session_id": f"test-latency-{i}"
        })
        latency = time.time() - start
        latencies.append(latency)

    # Calculate 95th percentile
    latencies.sort()
    p95_latency = latencies[int(len(latencies) * 0.95)]

    assert p95_latency < 2.0, f"P95 latency {p95_latency:.2f}s exceeds 2s budget"
```

**T3.3: RAG Off-Topic Refusal**
```python
# src/chatbot/tests/test_rag_guardrails.py
import pytest
from fastapi.testclient import TestClient
from src.chatbot.backend.main import app

client = TestClient(app)

OFF_TOPIC_QUESTIONS = [
    "What is the weather today?",
    "How do I cook pasta?",
    "Who won the 2024 election?",
    "Recommend a good movie"
]

@pytest.mark.parametrize("question", OFF_TOPIC_QUESTIONS)
def test_rag_refuses_off_topic_questions(question):
    """Verify RAG refuses to answer non-robotics questions"""
    response = client.post("/api/chat", json={
        "message": question,
        "session_id": "test-off-topic"
    })

    assert response.status_code == 200
    response_text = response.json()['response'].lower()

    # Verify refusal patterns
    refusal_patterns = ["robotics", "off-topic", "cannot help", "not relevant", "scope"]
    assert any(pattern in response_text for pattern in refusal_patterns), \
        f"RAG did not refuse off-topic question: {question}"
```

---

### 4. Platform Tests (Docusaurus)

**Tooling**: Jest (React components), Playwright (E2E), Lighthouse (performance)

**Test Cases**:

**T4.1: React Component Tests**
```javascript
// src/components/ChatWidget/ChatWidget.test.js
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ChatWidget from './ChatWidget';

test('renders chat widget with toggle button', () => {
  render(<ChatWidget />);
  const toggleButton = screen.getByRole('button', { name: /chat/i });
  expect(toggleButton).toBeInTheDocument();
});

test('opens and closes chat widget on toggle', () => {
  render(<ChatWidget />);
  const toggleButton = screen.getByRole('button', { name: /chat/i });

  // Initially closed
  expect(screen.queryByRole('textbox')).not.toBeInTheDocument();

  // Open widget
  fireEvent.click(toggleButton);
  expect(screen.getByRole('textbox')).toBeInTheDocument();

  // Close widget
  fireEvent.click(toggleButton);
  expect(screen.queryByRole('textbox')).not.toBeInTheDocument();
});

test('sends message to RAG backend', async () => {
  // Mock fetch
  global.fetch = jest.fn(() =>
    Promise.resolve({
      json: () => Promise.resolve({ response: 'This is a test response' }),
    })
  );

  render(<ChatWidget />);
  const toggleButton = screen.getByRole('button', { name: /chat/i });
  fireEvent.click(toggleButton);

  const input = screen.getByRole('textbox');
  const sendButton = screen.getByRole('button', { name: /send/i });

  fireEvent.change(input, { target: { value: 'What is ROS 2?' } });
  fireEvent.click(sendButton);

  await waitFor(() => {
    expect(screen.getByText(/This is a test response/i)).toBeInTheDocument();
  });

  expect(global.fetch).toHaveBeenCalledWith('/api/chat', expect.anything());
});
```

**T4.2: E2E User Workflows (Playwright)**
```javascript
// tests/e2e/student_workflow.spec.js
const { test, expect } = require('@playwright/test');

test.describe('Student Learning Workflow', () => {
  test('student navigates from Module 1 to Module 5', async ({ page }) => {
    // Navigate to homepage
    await page.goto('http://localhost:3000');

    // Click Module 1
    await page.click('text=Module 1: ROS 2 Fundamentals');
    await expect(page).toHaveURL(/module-01-ros2-fundamentals/);

    // Read Chapter 1
    await page.click('text=Chapter 1: Introduction');
    await expect(page.locator('h1')).toContainText('Introduction');

    // Navigate to Module 2
    await page.click('text=Module 2: Digital Twin');
    await expect(page).toHaveURL(/module-02-digital-twin-gazebo-unity/);

    // Continue to Module 5
    await page.click('text=Module 5: Capstone');
    await expect(page).toHaveURL(/module-05-capstone-autonomous-humanoid/);
  });

  test('student uses RAG chatbot', async ({ page }) => {
    await page.goto('http://localhost:3000');

    // Open chat widget
    await page.click('[aria-label="Open chat"]');

    // Type question
    await page.fill('[placeholder="Ask a question..."]', 'What is ROS 2?');
    await page.click('text=Send');

    // Wait for response
    await expect(page.locator('.chat-message.assistant')).toContainText('ROS 2', { timeout: 5000 });
  });
});
```

**T4.3: Performance Tests (Lighthouse)**
```javascript
// tests/performance/lighthouse.test.js
const lighthouse = require('lighthouse');
const chromeLauncher = require('chrome-launcher');

async function runLighthouse(url) {
  const chrome = await chromeLauncher.launch({ chromeFlags: ['--headless'] });
  const options = {
    logLevel: 'info',
    output: 'json',
    port: chrome.port,
  };

  const runnerResult = await lighthouse(url, options);
  await chrome.kill();

  return runnerResult.lhr;
}

test('homepage meets performance budget', async () => {
  const results = await runLighthouse('http://localhost:3000');

  expect(results.categories.performance.score).toBeGreaterThanOrEqual(0.90); // >90
  expect(results.audits['speed-index'].numericValue).toBeLessThan(2000); // <2s
  expect(results.audits['interactive'].numericValue).toBeLessThan(3000); // <3s
});
```

---

### 5. Integration Tests

**Tooling**: pytest (orchestration), ROS 2 launch tests, Docker Compose

**Test Cases**:

**T5.1: Module 1 → Module 2 Integration**
```python
# tests/integration/test_module1_to_module2.py
import pytest
import rclpy
import subprocess
import time

def test_module1_output_consumed_by_module2():
    """Verify Module 1 RobotState message is consumed by Module 2 Gazebo simulation"""

    # Launch Module 1 publisher
    proc1 = subprocess.Popen([
        'ros2', 'run', 'module_01_pubsub', 'robot_state_publisher'
    ])
    time.sleep(2)

    # Launch Module 2 Gazebo (listens to /robot/state)
    proc2 = subprocess.Popen([
        'ros2', 'launch', 'module_02_gazebo', 'simulation.launch.py'
    ])
    time.sleep(10)  # Wait for Gazebo to load

    # Verify /robot/state topic has subscribers
    result = subprocess.run(
        ['ros2', 'topic', 'info', '/robot/state'],
        capture_output=True,
        text=True
    )

    assert 'Subscription count: 1' in result.stdout or 'Subscription count: 2' in result.stdout

    # Cleanup
    proc1.kill()
    proc2.kill()
```

**T5.2: Full Capstone Integration**
```python
# tests/integration/test_capstone_integration.py
import pytest
import subprocess
import time

def test_capstone_all_nodes_communicate():
    """Verify all capstone nodes (perception, navigation, manipulation, VLA) communicate"""

    # Launch full capstone stack
    proc = subprocess.Popen([
        'ros2', 'launch', 'capstone', 'main.launch.py', 'use_sim_time:=true'
    ])
    time.sleep(15)  # Wait for all nodes to start

    # List all active topics
    result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)

    # Verify expected topics exist
    expected_topics = [
        '/robot/state',           # Module 1
        '/camera/image',          # Module 2 (Gazebo)
        '/detections',            # Module 3 (perception)
        '/nav/goal',              # Module 3 (navigation)
        '/vla/command',           # Module 4 (VLA)
    ]

    for topic in expected_topics:
        assert topic in result.stdout, f"Topic {topic} not found in capstone launch"

    # Cleanup
    proc.kill()
```

---

## CI/CD Integration

### GitHub Actions Workflow

```yaml
# .github/workflows/test.yml
name: Test Suite

on:
  push:
    branches: [master, main, develop]
  pull_request:
    branches: [master, main]

jobs:
  docs-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install pytest markdown-link-check

      - name: Test word counts
        run: pytest tests/docs/test_word_count.py

      - name: Test links
        run: bash tests/docs/test_links.sh

  code-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up ROS 2 Humble
        run: |
          sudo apt update
          sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions

      - name: Build ROS 2 packages
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --packages-select robotics_book_interfaces module_01_pubsub

      - name: Run unit tests
        run: |
          source /opt/ros/humble/setup.bash
          source install/setup.bash
          colcon test
          colcon test-result --verbose

  rag-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install -r src/chatbot/requirements.txt
          pip install pytest pytest-cov

      - name: Run RAG tests
        env:
          QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
          OPENAI_API_KEY: ${{ secrets.OPENAI_API_KEY }}
        run: |
          pytest src/chatbot/tests/ --cov=src/chatbot --cov-report=xml

      - name: Upload coverage
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage.xml

  platform-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '20'

      - name: Install dependencies
        run: npm ci

      - name: Run Jest tests
        run: npm test

      - name: Build site
        run: npm run build

      - name: Run E2E tests
        run: npx playwright test

  integration-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up ROS 2 and dependencies
        run: |
          # ... (install ROS 2, Gazebo, etc.)

      - name: Run integration tests
        run: pytest tests/integration/

  performance-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '20'

      - name: Build site
        run: npm ci && npm run build

      - name: Serve site
        run: npx serve -l 3000 build &

      - name: Run Lighthouse
        run: |
          npm install -g @lhci/cli
          lhci autorun --config=.lighthouserc.json
```

---

## Coverage Requirements

| Component | Coverage Target | Measurement |
|-----------|----------------|-------------|
| Documentation | 100% chapters validated | Word count + link check + manual review |
| Python Code (ROS 2) | >85% line coverage | pytest-cov |
| C++ Code (ROS 2) | >80% line coverage | gcov + lcov |
| JavaScript (React) | >80% line coverage | Jest coverage |
| RAG Chatbot | >90% accuracy | Custom validation set |
| Integration Tests | All critical paths | Manual test case design |

---

## Acceptance Criteria

- ✅ All 226 tasks have associated tests (unit, integration, or manual checklist)
- ✅ CI/CD pipeline runs all tests on every PR
- ✅ Test suite completes in <10 minutes
- ✅ >85% code coverage for Python/C++/JavaScript
- ✅ >90% RAG chatbot accuracy on validation set
- ✅ 100% documentation word count compliance
- ✅ 0 broken links in documentation
- ✅ Lighthouse performance score >90
- ✅ Integration tests validate Module 1→5 workflows

---

## Non-Goals

- ❌ 100% code coverage (diminishing returns)
- ❌ GPU-accelerated tests in CI (too expensive)
- ❌ Real-world hardware tests (Jetson deployment tested manually)
- ❌ Load testing (not required for educational platform)

---

## References

- pytest Documentation: https://docs.pytest.org/
- Google Test: https://google.github.io/googletest/
- Jest: https://jestjs.io/
- Playwright: https://playwright.dev/
- Lighthouse: https://developer.chrome.com/docs/lighthouse/
- ROS 2 Testing: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html
