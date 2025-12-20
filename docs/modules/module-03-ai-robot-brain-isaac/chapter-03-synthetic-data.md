---
title: 'Chapter 3: Synthetic Data Generation'
description: 'Generate labeled training data using Isaac Sim domain randomization'
sidebar_position: 3
---

# Chapter 3: Synthetic Data Generation

## Learning Objectives

1. **Generate** synthetic RGB-D images with automatic labels
2. **Apply** domain randomization for robust perception
3. **Export** datasets in COCO format

## Domain Randomization

Vary simulation parameters to improve real-world generalization:
- **Lighting**: Random sun angle, intensity
- **Textures**: Randomize object materials
- **Poses**: Random object positions/orientations

##Exercise

Generate 100 labeled images of objects in Isaac Sim warehouse scene.

**Code Example**: See `examples/module-03-isaac-brain/example-02-perception/`

**Next**: [Chapter 4: Visual SLAM](./chapter-04-vslam.md)

---

**Reading Time**: 25 minutes
**Hands-On Time**: 50 minutes
