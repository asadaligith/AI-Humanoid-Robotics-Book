---
title: 'Chapter 2: Speech Recognition with Whisper'
description: 'Implement speech-to-text using OpenAI Whisper'
sidebar_position: 2
---

# Chapter 2: Speech Recognition with Whisper

## Learning Objectives

1. **Install** and run Whisper locally
2. **Integrate** Whisper with ROS 2
3. **Optimize** for real-time speech recognition

## Whisper Setup

```bash
pip install openai-whisper
```

## Python Integration

```python
import whisper

model = whisper.load_model("base")
result = model.transcribe("command.wav")
print(result["text"])  # "Pick up the red cube"
```

## ROS 2 Node

Create a node that subscribes to audio and publishes transcribed text.

**Code Example**: See `examples/module-04-vla/example-01-vla-pipeline/whisper_node.py`

**Next**: [Chapter 3: LLM Task Planning](./chapter-03-llm-planning.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 40 minutes
