# Example 01: Speech Recognition with OpenAI Whisper

## Learning Objectives

By completing this example, you will:
- Set up OpenAI Whisper for speech-to-text
- Process audio input from microphone
- Integrate Whisper with ROS 2
- Optimize for real-time performance
- Deploy on NVIDIA Jetson devices

## Overview

OpenAI Whisper is a state-of-the-art automatic speech recognition (ASR) model that converts spoken language into text. It's the first component in the Vision-Language-Action pipeline.

**Key Features:**
- Multilingual support (99+ languages)
- Robust to accents and noise
- No fine-tuning required
- Multiple model sizes (tiny to large)

## Prerequisites

- Python 3.8+
- ROS 2 Humble installed
- Microphone or audio input device
- CUDA-capable GPU (optional, for faster inference)
- Completed Modules 01-03

## Installation

### CPU-Only Installation

```bash
# Install Whisper
pip3 install openai-whisper

# Install audio dependencies
sudo apt install -y \
  ffmpeg \
  portaudio19-dev \
  python3-pyaudio

pip3 install pyaudio sounddevice
```

### GPU-Accelerated Installation

```bash
# Install PyTorch with CUDA support
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install Whisper
pip3 install openai-whisper

# Verify CUDA availability
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

### Jetson Installation

```bash
# For NVIDIA Jetson (ARM64)
sudo apt install -y python3-pip libportaudio2 portaudio19-dev

# Install PyTorch for Jetson
wget https://nvidia.box.com/shared/static/ssf2v7pf5i245fk4i0q926hy4imzs2ph.whl -O torch-1.11.0-cp38-cp38-linux_aarch64.whl
pip3 install torch-1.11.0-cp38-cp38-linux_aarch64.whl

# Install Whisper
pip3 install openai-whisper
```

## Whisper Models

| Model  | Parameters | VRAM | Relative Speed | Accuracy |
|--------|------------|------|----------------|----------|
| tiny   | 39M        | ~1GB | 32x            | Good     |
| base   | 74M        | ~1GB | 16x            | Better   |
| small  | 244M       | ~2GB | 6x             | Great    |
| medium | 769M       | ~5GB | 2x             | Excellent|
| large  | 1550M      | ~10GB| 1x             | Best     |

**Recommendation**: Use `base` for real-time applications, `small` for balanced performance.

## Standalone Whisper Usage

### Basic Transcription

```python
#!/usr/bin/env python3
"""
Basic Whisper speech-to-text example
"""
import whisper
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav

# Load model (first run downloads the model)
model = whisper.load_model("base")

# Record audio (5 seconds)
print("Recording for 5 seconds...")
fs = 16000  # Sample rate
duration = 5  # seconds
recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='float32')
sd.wait()
print("Recording complete!")

# Save to temporary file
temp_file = "/tmp/recording.wav"
wav.write(temp_file, fs, recording)

# Transcribe
print("Transcribing...")
result = model.transcribe(temp_file)

print(f"\nTranscription: {result['text']}")
print(f"Language: {result['language']}")
```

### Real-Time Streaming

```python
#!/usr/bin/env python3
"""
Real-time speech recognition with Whisper
"""
import whisper
import sounddevice as sd
import numpy as np
import queue
import threading
import scipy.io.wavfile as wav

class WhisperRealTime:
    def __init__(self, model_name="base"):
        self.model = whisper.load_model(model_name)
        self.audio_queue = queue.Queue()
        self.sample_rate = 16000
        self.chunk_duration = 3  # seconds per chunk
        self.is_listening = False

    def audio_callback(self, indata, frames, time, status):
        """Callback for audio stream."""
        if status:
            print(status)
        self.audio_queue.put(indata.copy())

    def process_audio(self):
        """Process audio chunks from queue."""
        chunk_size = int(self.chunk_duration * self.sample_rate)
        audio_buffer = np.array([], dtype=np.float32)

        while self.is_listening:
            try:
                # Get audio from queue
                chunk = self.audio_queue.get(timeout=0.1)
                audio_buffer = np.append(audio_buffer, chunk.flatten())

                # Process when buffer is full
                if len(audio_buffer) >= chunk_size:
                    # Save chunk to temp file
                    temp_file = "/tmp/chunk.wav"
                    wav.write(temp_file, self.sample_rate, audio_buffer[:chunk_size])

                    # Transcribe
                    result = self.model.transcribe(temp_file, fp16=False)
                    text = result['text'].strip()

                    if text:
                        print(f"You said: {text}")

                    # Keep overlap for continuity
                    overlap = chunk_size // 2
                    audio_buffer = audio_buffer[chunk_size - overlap:]

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error processing audio: {e}")

    def start(self):
        """Start listening."""
        self.is_listening = True

        # Start processing thread
        process_thread = threading.Thread(target=self.process_audio)
        process_thread.start()

        # Start audio stream
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.audio_callback,
            dtype='float32'
        ):
            print("Listening... (Press Ctrl+C to stop)")
            try:
                while self.is_listening:
                    sd.sleep(100)
            except KeyboardInterrupt:
                print("\nStopping...")
                self.is_listening = False

        process_thread.join()

if __name__ == "__main__":
    whisper_rt = WhisperRealTime(model_name="base")
    whisper_rt.start()
```

## ROS 2 Integration

### Whisper ROS 2 Node

```python
#!/usr/bin/env python3
"""
ROS 2 node for Whisper speech recognition
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import threading
import queue

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Parameters
        self.declare_parameter('model_name', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_duration', 3.0)

        model_name = self.get_parameter('model_name').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_duration = self.get_parameter('chunk_duration').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_name}')
        self.model = whisper.load_model(model_name)
        self.get_logger().info('Whisper model loaded!')

        # Publisher for transcribed text
        self.text_pub = self.create_publisher(String, 'voice_command', 10)

        # Audio queue
        self.audio_queue = queue.Queue()
        self.is_listening = False

        # Start audio processing
        self.start_listening()

    def audio_callback(self, indata, frames, time, status):
        """Audio stream callback."""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.audio_queue.put(indata.copy())

    def process_audio(self):
        """Process audio chunks."""
        chunk_size = int(self.chunk_duration * self.sample_rate)
        audio_buffer = np.array([], dtype=np.float32)

        while self.is_listening:
            try:
                chunk = self.audio_queue.get(timeout=0.1)
                audio_buffer = np.append(audio_buffer, chunk.flatten())

                if len(audio_buffer) >= chunk_size:
                    # Transcribe
                    temp_file = "/tmp/whisper_chunk.wav"
                    wav.write(temp_file, self.sample_rate, audio_buffer[:chunk_size])

                    result = self.model.transcribe(
                        temp_file,
                        language=self.language,
                        fp16=False
                    )

                    text = result['text'].strip()

                    if text:
                        self.get_logger().info(f'Transcribed: {text}')

                        # Publish to ROS topic
                        msg = String()
                        msg.data = text
                        self.text_pub.publish(msg)

                    # Keep overlap
                    overlap = chunk_size // 2
                    audio_buffer = audio_buffer[chunk_size - overlap:]

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error: {e}')

    def start_listening(self):
        """Start audio stream."""
        self.is_listening = True

        # Start processing thread
        self.process_thread = threading.Thread(target=self.process_audio)
        self.process_thread.daemon = True
        self.process_thread.start()

        # Start audio stream
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.audio_callback,
            dtype='float32'
        )
        self.stream.start()
        self.get_logger().info('Listening for voice commands...')

    def stop_listening(self):
        """Stop audio stream."""
        self.is_listening = False
        if hasattr(self, 'stream'):
            self.stream.stop()
            self.stream.close()
        if hasattr(self, 'process_thread'):
            self.process_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_listening()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla_system',
            executable='whisper_node',
            name='whisper_node',
            parameters=[{
                'model_name': 'base',
                'language': 'en',
                'sample_rate': 16000,
                'chunk_duration': 3.0,
            }],
            output='screen'
        ),
    ])
```

## Running the Node

```bash
# Terminal 1: Run Whisper node
ros2 run vla_system whisper_node

# Terminal 2: Monitor transcriptions
ros2 topic echo /voice_command

# Test by speaking into microphone
# Output: "go to the kitchen"
```

## Performance Optimization

### GPU Acceleration

```python
# Force CUDA usage
import torch
device = "cuda" if torch.cuda.is_available() else "cpu"
model = whisper.load_model("base", device=device)

# Use FP16 for faster inference
result = model.transcribe(audio_file, fp16=True)
```

### Batch Processing

```python
# Process multiple audio files
audio_files = ["file1.wav", "file2.wav", "file3.wav"]
results = model.transcribe_multiple(audio_files)
```

### Model Quantization (Jetson)

```python
# Use INT8 quantization for Jetson
import torch
model = whisper.load_model("base")
model = torch.quantization.quantize_dynamic(
    model, {torch.nn.Linear}, dtype=torch.qint8
)
```

## Exercises

### Exercise 1: Voice Commands
Create a vocabulary of robot commands and filter Whisper output for valid commands only.

### Exercise 2: Wake Word Detection
Implement wake word detection (e.g., "Hey robot") before activating Whisper.

### Exercise 3: Multilingual Support
Test Whisper with multiple languages and auto-detect language.

### Exercise 4: Noise Filtering
Add pre-processing to filter background noise before transcription.

### Exercise 5: Edge Deployment
Deploy on Raspberry Pi or Jetson with optimized model size.

## Common Issues

### Issue 1: High latency
**Solution**: Use smaller model (tiny/base) or increase chunk_duration.

### Issue 2: Poor accuracy
**Solution**: Use larger model, improve microphone quality, reduce background noise.

### Issue 3: CUDA out of memory
**Solution**: Use smaller model or disable fp16.

### Issue 4: Audio device not found
**Solution**: List devices with `python3 -m sounddevice` and specify device ID.

## Key Takeaways

1. **Model Size Matters**: Balance accuracy vs. speed based on use case
2. **GPU Acceleration**: 10-20x speedup with CUDA
3. **Real-Time Tradeoffs**: Chunk duration affects latency and accuracy
4. **ROS Integration**: Publish transcriptions for downstream processing
5. **Edge Deployment**: Optimize for Jetson with quantization

## Next Steps

- **Example 02**: LLM task planning with Claude API
- **Example 03**: ROS action executor
- **Module 04 Chapter 05**: End-to-end VLA integration

## References

- [OpenAI Whisper](https://github.com/openai/whisper)
- [Whisper Paper](https://arxiv.org/abs/2212.04356)
- [PyAudio Documentation](https://people.csail.mit.edu/hubert/pyaudio/)
- [NVIDIA Jetson AI](https://developer.nvidia.com/embedded/jetson-ai)
