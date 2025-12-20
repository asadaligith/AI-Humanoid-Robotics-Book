# Integration Testing Guide

## Overview

Integration testing ensures that all course components work together correctly: ROS 2 nodes, simulation environments, VLA pipeline, and the RAG chatbot.

## Testing Levels

```
Unit Tests → Integration Tests → System Tests → E2E Tests
   ↓              ↓                   ↓            ↓
 Functions    Components         Full Stack    User Flows
```

## ROS 2 Integration Testing

### Test Framework Setup

```bash
# Install testing dependencies
sudo apt install -y \
  ros-humble-launch-testing \
  ros-humble-launch-testing-ament-cmake \
  ros-humble-launch-testing-ros

pip3 install pytest pytest-timeout pytest-asyncio
```

### Example: Testing Publisher-Subscriber

```python
# test_pubsub_integration.py
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class TestPubSubIntegration:
    @classmethod
    def setup_class(cls):
        """Initialize ROS 2."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS 2."""
        rclpy.shutdown()

    def test_publisher_subscriber_communication(self):
        """Test that publisher and subscriber can communicate."""

        # Create test nodes
        class TestPublisher(Node):
            def __init__(self):
                super().__init__('test_publisher')
                self.publisher = self.create_publisher(String, 'test_topic', 10)
                self.timer = self.create_timer(0.5, self.publish_message)

            def publish_message(self):
                msg = String()
                msg.data = 'test_message'
                self.publisher.publish(msg)

        class TestSubscriber(Node):
            def __init__(self):
                super().__init__('test_subscriber')
                self.subscription = self.create_subscription(
                    String, 'test_topic', self.callback, 10)
                self.received_messages = []

            def callback(self, msg):
                self.received_messages.append(msg.data)

        # Create nodes
        publisher = TestPublisher()
        subscriber = TestSubscriber()

        # Spin in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        # Wait for messages
        time.sleep(2)

        # Assert messages were received
        assert len(subscriber.received_messages) > 0
        assert subscriber.received_messages[0] == 'test_message'

        # Cleanup
        publisher.destroy_node()
        subscriber.destroy_node()
```

## VLA Pipeline Integration Tests

### Test Whisper → LLM → Action Pipeline

```python
# test_vla_pipeline.py
import pytest
from unittest.mock import Mock, patch
import asyncio

class TestVLAPipeline:
    @pytest.mark.asyncio
    async def test_voice_command_to_action(self):
        """Test complete VLA pipeline."""

        # Mock components
        with patch('whisper.load_model') as mock_whisper, \
             patch('anthropic.Anthropic') as mock_claude:

            # Setup mocks
            mock_whisper.return_value.transcribe.return_value = {
                'text': 'go to the kitchen'
            }

            mock_claude.return_value.messages.create.return_value = Mock(
                content=[Mock(
                    type='tool_use',
                    name='create_robot_plan',
                    input={
                        'goal': 'Navigate to kitchen',
                        'actions': [
                            {'action': 'navigate', 'target': 'kitchen'}
                        ]
                    }
                )]
            )

            # Run pipeline
            from vla_pipeline import VLAPipeline
            pipeline = VLAPipeline()

            result = await pipeline.process_voice_command('audio.wav')

            # Assertions
            assert result['transcription'] == 'go to the kitchen'
            assert len(result['actions']) == 1
            assert result['actions'][0]['action'] == 'navigate'
            assert result['actions'][0]['target'] == 'kitchen'

    def test_error_handling_whisper_failure(self):
        """Test pipeline handles Whisper failure gracefully."""

        with patch('whisper.load_model') as mock_whisper:
            mock_whisper.return_value.transcribe.side_effect = Exception("Whisper error")

            from vla_pipeline import VLAPipeline
            pipeline = VLAPipeline()

            with pytest.raises(Exception) as exc_info:
                pipeline.process_voice_command('audio.wav')

            assert "Whisper error" in str(exc_info.value)
```

## RAG Chatbot Integration Tests

### Test Document Retrieval and Response Generation

```python
# test_rag_integration.py
import pytest
from qdrant_client import QdrantClient
from openai import OpenAI
import anthropic

class TestRAGIntegration:
    @pytest.fixture
    def rag_pipeline(self):
        """Create RAG pipeline for testing."""
        from rag_pipeline import RAGPipeline
        return RAGPipeline(
            qdrant_url=os.getenv('QDRANT_TEST_URL'),
            openai_key=os.getenv('OPENAI_API_KEY'),
            claude_key=os.getenv('ANTHROPIC_API_KEY')
        )

    def test_document_ingestion(self, rag_pipeline):
        """Test document ingestion into vector DB."""

        # Ingest test document
        test_doc = {
            'text': 'ROS 2 uses a publisher-subscriber pattern for communication.',
            'metadata': {'module': '01', 'chapter': '02'}
        }

        doc_id = rag_pipeline.ingest_document(test_doc)

        # Verify document was stored
        assert doc_id is not None

        # Query for document
        results = rag_pipeline.search('publisher subscriber pattern')

        assert len(results) > 0
        assert 'publisher-subscriber' in results[0]['text']

    def test_query_response_generation(self, rag_pipeline):
        """Test complete RAG query flow."""

        # Query chatbot
        response = rag_pipeline.query('How do I create a ROS 2 publisher?')

        # Assertions
        assert response['answer'] is not None
        assert len(response['answer']) > 0
        assert len(response['sources']) > 0
        assert 'create_publisher' in response['answer'].lower()

    def test_context_relevance(self, rag_pipeline):
        """Test that retrieved context is relevant to query."""

        query = "What is SLAM?"
        response = rag_pipeline.query(query)

        # Check that sources are relevant
        for source in response['sources']:
            assert source['relevance_score'] > 0.5
            assert any(keyword in source['text'].lower()
                      for keyword in ['slam', 'localization', 'mapping'])
```

## Simulation Integration Tests

### Test Gazebo + ROS 2 Integration

```python
# test_gazebo_integration.py
import pytest
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist

class TestGazeboIntegration:
    @classmethod
    def setup_class(cls):
        """Launch Gazebo simulation."""
        cls.gazebo_process = subprocess.Popen(
            ['gz', 'sim', 'test_world.sdf', '-s'],  # Headless
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(5)  # Wait for Gazebo to start
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Stop Gazebo simulation."""
        cls.gazebo_process.terminate()
        cls.gazebo_process.wait()
        rclpy.shutdown()

    def test_robot_spawning(self):
        """Test that robot can be spawned in Gazebo."""

        # Spawn robot
        result = subprocess.run(
            ['ros2', 'service', 'call', '/spawn_entity',
             'gazebo_msgs/srv/SpawnEntity',
             '{name: "test_robot", xml: "$(cat test_robot.urdf)"}'],
            capture_output=True,
            text=True
        )

        assert result.returncode == 0
        assert 'success' in result.stdout.lower()

    def test_robot_movement(self):
        """Test robot responds to velocity commands."""

        # Create velocity publisher
        node = rclpy.create_node('test_movement')
        pub = node.create_publisher(Twist, '/cmd_vel', 10)

        # Publish velocity command
        msg = Twist()
        msg.linear.x = 0.5
        pub.publish(msg)

        time.sleep(1)

        # TODO: Verify robot moved (check /odom or /joint_states)

        node.destroy_node()
```

## Continuous Integration Tests

### GitHub Actions Workflow

```yaml
# .github/workflows/integration-tests.yml
name: Integration Tests

on:
  push:
    branches: [main, develop]
  pull_request:

jobs:
  test-ros2:
    runs-on: ubuntu-22.04

    steps:
      - uses: actions/checkout@v3

      - name: Setup ROS 2 Humble
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble

      - name: Install dependencies
        run: |
          sudo apt update
          sudo apt install -y ros-humble-gazebo-ros-pkgs
          pip3 install pytest pytest-asyncio

      - name: Build workspace
        run: |
          source /opt/ros/humble/setup.bash
          cd ros2_ws
          colcon build

      - name: Run integration tests
        run: |
          source /opt/ros/humble/setup.bash
          source ros2_ws/install/setup.bash
          pytest tests/integration/ -v

  test-vla:
    runs-on: ubuntu-22.04

    steps:
      - uses: actions/checkout@v3

      - name: Setup Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install -r chatbot/requirements.txt
          pip install pytest pytest-asyncio pytest-mock

      - name: Run VLA tests
        env:
          ANTHROPIC_API_KEY: ${{ secrets.ANTHROPIC_API_KEY }}
          OPENAI_API_KEY: ${{ secrets.OPENAI_API_KEY }}
        run: |
          pytest tests/vla/ -v

  test-rag:
    runs-on: ubuntu-22.04

    steps:
      - uses: actions/checkout@v3

      - name: Setup Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install -r chatbot/requirements.txt
          pip install pytest pytest-asyncio

      - name: Run RAG tests
        env:
          QDRANT_URL: ${{ secrets.QDRANT_TEST_URL }}
          QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
          DATABASE_URL: ${{ secrets.DATABASE_TEST_URL }}
          ANTHROPIC_API_KEY: ${{ secrets.ANTHROPIC_API_KEY }}
          OPENAI_API_KEY: ${{ secrets.OPENAI_API_KEY }}
        run: |
          pytest tests/rag/ -v
```

## Load Testing

### Test API Performance

```python
# test_load.py
import asyncio
import aiohttp
import time
from statistics import mean, median

async def load_test_chatbot(num_requests=100):
    """Load test the chatbot API."""

    async def send_query(session, query):
        start = time.time()
        async with session.post(
            'http://localhost:8000/chat',
            json={'query': query}
        ) as response:
            await response.json()
            return time.time() - start

    queries = [
        "How do I create a ROS 2 publisher?",
        "What is SLAM?",
        "Explain Nav2 navigation",
    ] * (num_requests // 3)

    async with aiohttp.ClientSession() as session:
        tasks = [send_query(session, q) for q in queries]
        response_times = await asyncio.gather(*tasks)

    # Print statistics
    print(f"Total requests: {num_requests}")
    print(f"Mean response time: {mean(response_times):.2f}s")
    print(f"Median response time: {median(response_times):.2f}s")
    print(f"Min response time: {min(response_times):.2f}s")
    print(f"Max response time: {max(response_times):.2f}s")

# Run load test
asyncio.run(load_test_chatbot(100))
```

## Test Coverage

### Generate Coverage Report

```bash
# Run tests with coverage
pytest --cov=app --cov-report=html tests/

# View coverage report
open htmlcov/index.html

# Target: 80%+ coverage
```

## Best Practices

1. **Isolate Tests**: Each test should be independent
2. **Use Fixtures**: Reuse common setup code
3. **Mock External APIs**: Don't call real APIs in tests
4. **Test Edge Cases**: Not just happy paths
5. **Fast Tests**: Keep test suite under 5 minutes
6. **Meaningful Assertions**: Test behavior, not implementation

## References

- [ROS 2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [pytest Documentation](https://docs.pytest.org/)
- [FastAPI Testing](https://fastapi.tiangolo.com/tutorial/testing/)
