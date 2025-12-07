#!/usr/bin/env python3
"""
Voice Input Node - Capstone Autonomous Humanoid

Captures audio from microphone and transcribes using OpenAI Whisper.
Publishes transcribed text to /voice/transcribed_text topic.

Dependencies:
- openai-whisper
- sounddevice
- numpy
- rclpy

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np
import sounddevice as sd
import os


class VoiceInputNode(Node):
    """ROS 2 node for voice transcription using Whisper"""

    def __init__(self):
        super().__init__('voice_input_node')

        # Declare parameters
        self.declare_parameter('whisper_model', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('duration', 3.0)
        self.declare_parameter('silence_threshold', 0.01)
        self.declare_parameter('device', 'cpu')  # or 'cuda' for GPU

        # Get parameters
        model_size = self.get_parameter('whisper_model').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.duration = self.get_parameter('duration').value
        self.silence_threshold = self.get_parameter('silence_threshold').value
        device = self.get_parameter('device').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size} on {device}')
        self.model = whisper.load_model(model_size, device=device)
        self.get_logger().info(f'Whisper model loaded successfully')

        # Publisher for transcribed text
        self.text_pub = self.create_publisher(
            String,
            '/voice/transcribed_text',
            10
        )

        # Timer for continuous listening (check every 100ms)
        self.listening = False
        self.create_timer(0.1, self.listen_callback)

        self.get_logger().info('Voice Input Node initialized - ready to listen')

    def listen_callback(self):
        """Continuously listen for voice commands"""
        if self.listening:
            return  # Skip if already processing

        try:
            self.listening = True

            # Record audio
            self.get_logger().debug(f'Recording audio for {self.duration}s...')
            audio = sd.rec(
                int(self.duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32'
            )
            sd.wait()  # Wait for recording to finish

            # Check for voice activity (simple energy-based VAD)
            audio_flat = audio.flatten()
            energy = np.sqrt(np.mean(audio_flat**2))

            if energy < self.silence_threshold:
                self.get_logger().debug(f'Silence detected (energy={energy:.4f}), skipping')
                self.listening = False
                return

            # Transcribe audio
            self.get_logger().info(f'Voice detected (energy={energy:.4f}), transcribing...')
            result = self.model.transcribe(
                audio_flat,
                language=self.language,
                fp16=False  # Use FP32 for CPU
            )

            text = result['text'].strip()

            if text:
                self.get_logger().info(f'Transcribed: "{text}"')

                # Publish transcription
                msg = String()
                msg.data = text
                self.text_pub.publish(msg)
            else:
                self.get_logger().warn('Empty transcription result')

        except Exception as e:
            self.get_logger().error(f'Voice input error: {str(e)}')

        finally:
            self.listening = False

    def shutdown(self):
        """Cleanup on node shutdown"""
        self.get_logger().info('Voice Input Node shutting down')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = VoiceInputNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
