---
title: 'Module 4: Vision-Language-Action (VLA)'
description: 'Integrate voice commands, LLM planning, and robot execution for natural human-robot interaction'
sidebar_position: 4
---

# Module 4: Vision-Language-Action (VLA)

## Module Overview

This module teaches Vision-Language-Action (VLA) pipelines, enabling robots to understand natural language commands and execute complex tasks autonomously.

## Learning Objectives

1. **Implement** speech-to-text using Whisper
2. **Integrate** LLMs (Claude/GPT-4) for task planning
3. **Execute** action plans via ROS 2
4. **Deploy** VLA pipeline on Jetson for real-time operation

## Prerequisites

- **Modules 1-3**: ROS 2, simulation, perception, navigation
- **API Keys**: OpenAI API (Whisper, GPT-4) OR Anthropic Claude API

## Module Structure

| Chapter | Topic | Key Technology | Time |
|---------|-------|----------------|------|
| [Chapter 1](./chapter-01-introduction.md) | VLA Introduction | LLMs in robotics | 30 min |
| [Chapter 2](./chapter-02-whisper.md) | Speech Recognition | OpenAI Whisper | 1.5 hours |
| [Chapter 3](./chapter-03-llm-planning.md) | LLM Task Planning | Claude/GPT-4 | 2 hours |
| [Chapter 4](./chapter-04-ros-executor.md) | ROS Action Executor | ROS 2 actions | 1.5 hours |
| [Chapter 5](./chapter-05-integration.md) | End-to-End Pipeline | Full VLA | 2 hours |
| [Chapter 6](./chapter-06-jetson-deploy.md) | Jetson Deployment | Edge AI | 2 hours |
| [Chapter 7](./chapter-07-summary.md) | Module Summary | N/A | 30 min |

**Total Time**: 10-12 hours

## Real-World Application

VLA enables:
- "Pick up the red cube" → Robot navigates, grasps, delivers
- "Follow me" → Robot tracks and follows human
- "Clean the table" → Robot identifies objects, grasps, places in bin

## Resources

- [Whisper Documentation](https://platform.openai.com/docs/guides/speech-to-text)
- [Anthropic Claude API](https://www.anthropic.com/api)
- [RoboInstructor Paper](https://arxiv.org/abs/2310.07642)

[Start Chapter 1](./chapter-01-introduction.md)
