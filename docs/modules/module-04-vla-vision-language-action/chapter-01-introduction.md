---
title: 'Chapter 1: Introduction to VLA'
description: 'Understand Vision-Language-Action models for embodied AI'
sidebar_position: 1
---

# Chapter 1: Introduction to VLA

## Learning Objectives

1. **Define** Vision-Language-Action (VLA) models
2. **Explain** the role of LLMs in robotics
3. **Compare** VLA approaches (end-to-end vs. modular)

## What is VLA?

VLA models combine:
- **Vision**: Perceive environment (cameras, depth sensors)
- **Language**: Understand commands (NLP, LLMs)
- **Action**: Execute tasks (robot control)

**Example Workflow**:
```
User: "Pick up the red cube"
  → Whisper: Transcribe audio to text
  → LLM: Generate action plan JSON
  → ROS Executor: Navigate, grasp, lift
```

## Modular vs. End-to-End

| Approach | Architecture | Pros | Cons |
|----------|-------------|------|------|
| **Modular** | Whisper + LLM + ROS | Interpretable, debuggable | Complex integration |
| **End-to-End** | Single neural network | Simpler | Black box, needs massive data |

**This course**: Modular approach (industry standard)

**Next**: [Chapter 2: Speech Recognition with Whisper](./chapter-02-whisper.md)

---

**Reading Time**: 15 minutes
