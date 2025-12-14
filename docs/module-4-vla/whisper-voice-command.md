---
title: Whisper Voice-to-Command Pipeline
sidebar_position: 2
---

# Whisper Voice-to-Command Pipeline

## Introduction to Voice Command Processing

Voice command processing is a critical component of Vision-Language-Action (VLA) systems, enabling robots to understand and respond to natural human speech. OpenAI's Whisper model represents a significant advancement in automatic speech recognition (ASR), providing robust, multilingual speech-to-text capabilities that can be integrated into robotic systems for natural human-robot interaction.

## Overview of Whisper for Robotics

### Whisper Model Architecture

Whisper is a transformer-based model designed for speech recognition that:
- Handles multiple languages natively
- Shows robustness to accents, background noise, and technical jargon
- Provides both transcription and language identification
- Offers various model sizes for different computational requirements

### Advantages for Robotics
- **Multilingual Support**: Communicate with users in their native language
- **Robustness**: Handles noisy environments common in robotics
- **Real-time Capabilities**: Can be optimized for interactive applications
- **Open Source**: Available for customization and integration

## Setting Up Whisper for Robot Applications

### Installation and Dependencies
```bash
pip install openai-whisper
pip install torch torchvision torchaudio
pip install sounddevice pyaudio  # For audio input
pip install transformers        # For additional NLP processing
```

### Basic Whisper Implementation
```python
import whisper
import torch
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
import tempfile
import os

class WhisperRobotInterface:
    def __init__(self, model_size="base", device="cuda" if torch.cuda.is_available() else "cpu"):
        """
        Initialize Whisper model for robot voice command processing
        """
        self.model_size = model_size
        self.device = device

        # Load Whisper model
        print(f"Loading Whisper {model_size} model on {device}...")
        self.model = whisper.load_model(model_size, device=device)

        # Audio parameters
        self.sample_rate = 16000  # Standard for Whisper
        self.chunk_duration = 1.0  # 1 second chunks for real-time processing
        self.chunk_size = int(self.sample_rate * self.chunk_duration)

        # Command history
        self.command_history = []

    def transcribe_audio(self, audio_data, language="en"):
        """
        Transcribe audio data to text using Whisper
        """
        # Convert audio to the format expected by Whisper
        if isinstance(audio_data, str):
            # If it's a file path
            result = self.model.transcribe(audio_data, language=language)
        else:
            # If it's audio data
            result = self.model.transcribe(audio_data, language=language)

        return result["text"].strip()

    def record_audio_chunk(self, duration=3.0):
        """
        Record a chunk of audio for processing
        """
        print(f"Recording {duration} seconds of audio...")
        audio_data = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()  # Wait for recording to complete
        return audio_data.flatten()

    def continuous_listening(self, callback_func, timeout=5.0):
        """
        Continuously listen for voice commands with timeout
        """
        print("Listening for voice commands...")

        while True:
            try:
                # Record a chunk of audio
                audio_chunk = self.record_audio_chunk(duration=timeout)

                # Transcribe the audio
                transcription = self.transcribe_audio(audio_chunk)

                if transcription:  # If we got a transcription
                    print(f"Heard: {transcription}")

                    # Process the command
                    processed_command = self.process_command(transcription)

                    if processed_command:
                        # Execute callback with the processed command
                        callback_func(processed_command)

            except KeyboardInterrupt:
                print("Stopping voice command processing...")
                break
            except Exception as e:
                print(f"Error in voice processing: {e}")
                continue

    def process_command(self, raw_text):
        """
        Process raw transcribed text into structured command
        """
        # Clean up the text
        cleaned_text = self.clean_transcription(raw_text)

        # Validate if this is actually a command (not just noise)
        if self.is_valid_command(cleaned_text):
            # Add to history
            self.command_history.append({
                'text': cleaned_text,
                'timestamp': torch.datetime.now()
            })

            return cleaned_text

        return None

    def clean_transcription(self, text):
        """
        Clean up Whisper transcription artifacts
        """
        # Remove common Whisper artifacts
        text = text.replace("[BLANK_AUDIO]", "")
        text = text.replace("[NO_SPEECH]", "")
        text = text.strip()

        # Remove extra whitespace
        text = ' '.join(text.split())

        return text

    def is_valid_command(self, text):
        """
        Determine if the transcribed text is a valid command
        """
        if not text:
            return False

        # Check minimum length
        if len(text.strip()) < 3:
            return False

        # Check for common non-command words
        non_command_words = ["um", "uh", "uhh", "like", "you know"]
        if text.lower() in non_command_words:
            return False

        return True
```

## Real-Time Voice Command Processing

### Streaming Audio Processing
```python
import queue
import threading
import time

class StreamingWhisperProcessor:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.is_listening = False

    def audio_callback(self, indata, frames, time, status):
        """Callback for real-time audio input"""
        # Add audio data to queue for processing
        audio_chunk = indata.copy()
        self.audio_queue.put(audio_chunk)

    def start_streaming(self):
        """Start streaming audio processing"""
        self.is_listening = True

        # Start audio input thread
        audio_thread = threading.Thread(target=self._process_audio_stream)
        audio_thread.daemon = True
        audio_thread.start()

        # Start transcription thread
        transcribe_thread = threading.Thread(target=self._transcribe_stream)
        transcribe_thread.daemon = True
        transcribe_thread.start()

    def _process_audio_stream(self):
        """Process incoming audio stream"""
        with sd.InputStream(
            callback=self.audio_callback,
            channels=1,
            samplerate=16000,
            blocksize=8000  # 0.5 second blocks
        ):
            while self.is_listening:
                time.sleep(0.1)

    def _transcribe_stream(self):
        """Transcribe audio chunks in real-time"""
        accumulated_audio = np.array([])

        while self.is_listening:
            try:
                # Get audio chunk from queue
                audio_chunk = self.audio_queue.get(timeout=1.0)

                # Accumulate audio for better transcription
                accumulated_audio = np.concatenate([accumulated_audio, audio_chunk.flatten()])

                # Process every 2 seconds of accumulated audio
                if len(accumulated_audio) >= 16000 * 2:  # 2 seconds at 16kHz
                    # Transcribe the accumulated audio
                    text = self.model.transcribe(accumulated_audio[:16000*2])["text"]

                    if text.strip():
                        self.result_queue.put({
                            'text': text.strip(),
                            'timestamp': time.time()
                        })

                    # Keep remaining audio for next processing
                    accumulated_audio = accumulated_audio[16000*2:]

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Transcription error: {e}")
                continue
```

## Voice Command Interpretation

### Natural Language Understanding
```python
import re
from dataclasses import dataclass
from typing import Optional, List, Dict

@dataclass
class RobotCommand:
    """Structured representation of a robot command"""
    action: str
    parameters: Dict[str, str]
    confidence: float
    raw_text: str

class CommandInterpreter:
    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            'navigation': [
                r'move to (?:the )?(?P<location>\w+)',
                r'go to (?:the )?(?P<location>\w+)',
                r'go (?:to )?(?P<location>\w+)',
                r'navigate to (?:the )?(?P<location>\w+)'
            ],
            'manipulation': [
                r'pick up (?:the )?(?P<object>\w+)',
                r'grab (?:the )?(?P<object>\w+)',
                r'take (?:the )?(?P<object>\w+)',
                r'get (?:the )?(?P<object>\w+)'
            ],
            'interaction': [
                r'say (?:to me )?"(?P<text>[^"]+)"',
                r'tell me (?:about )?(?P<topic>\w+)',
                r'what is (?:the )?(?P<topic>\w+)'
            ],
            'control': [
                r'stop',
                r'pause',
                r'continue',
                r'wait',
                r'follow me'
            ]
        }

        # Location mappings
        self.location_map = {
            'kitchen': 'kitchen_waypoint',
            'living room': 'living_room_waypoint',
            'bedroom': 'bedroom_waypoint',
            'office': 'office_waypoint',
            'dining room': 'dining_room_waypoint'
        }

        # Object mappings
        self.object_map = {
            'bottle': 'bottle_1',
            'cup': 'cup_1',
            'book': 'book_1',
            'phone': 'phone_1'
        }

    def interpret_command(self, text: str) -> Optional[RobotCommand]:
        """Interpret natural language command into structured format"""
        text_lower = text.lower().strip()

        # Check each command type
        for action_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower, re.IGNORECASE)
                if match:
                    params = match.groupdict()

                    # Map parameters to robot-specific identifiers
                    params = self._map_parameters(params, action_type)

                    # Calculate confidence based on match quality
                    confidence = self._calculate_confidence(text, pattern, match)

                    return RobotCommand(
                        action=action_type,
                        parameters=params,
                        confidence=confidence,
                        raw_text=text
                    )

        return None

    def _map_parameters(self, params: Dict[str, str], action_type: str) -> Dict[str, str]:
        """Map natural language parameters to robot-specific identifiers"""
        mapped_params = {}

        for key, value in params.items():
            if action_type == 'navigation' and key == 'location':
                # Map location names to robot waypoints
                mapped_params[key] = self.location_map.get(value, value)
            elif action_type == 'manipulation' and key == 'object':
                # Map object names to robot object identifiers
                mapped_params[key] = self.object_map.get(value, value)
            else:
                mapped_params[key] = value

        return mapped_params

    def _calculate_confidence(self, text: str, pattern: str, match) -> float:
        """Calculate confidence score for command interpretation"""
        # Base confidence on pattern match quality
        base_confidence = 0.8 if match else 0.0

        # Adjust based on text length (longer text might be more specific)
        length_factor = min(len(text) / 100.0, 0.2)  # Max 0.2 for length

        # Adjust based on pattern specificity
        pattern_specificity = 0.1 if '?' in pattern else 0.0  # Optional groups

        confidence = base_confidence + length_factor - pattern_specificity
        return min(confidence, 1.0)  # Cap at 1.0
```

## Integration with ROS 2

### ROS 2 Voice Command Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Initialize Whisper interface
        self.whisper_interface = WhisperRobotInterface()
        self.command_interpreter = CommandInterpreter()

        # ROS publishers and subscribers
        self.command_pub = self.create_publisher(String, 'robot_commands', 10)
        self.speech_pub = self.create_publisher(String, 'speech_output', 10)

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer for continuous listening
        self.listen_timer = self.create_timer(0.1, self.check_for_commands)

        # Command queue
        self.command_queue = []

        self.get_logger().info('Voice Command Node initialized')

    def check_for_commands(self):
        """Check for new voice commands"""
        # This would integrate with the streaming processor
        # For now, we'll simulate command detection
        pass

    def process_voice_command(self, command_text: str):
        """Process a voice command and execute appropriate action"""
        self.get_logger().info(f'Processing command: {command_text}')

        # Interpret the command
        robot_command = self.command_interpreter.interpret_command(command_text)

        if robot_command:
            self.get_logger().info(f'Interpreted command: {robot_command.action} with params {robot_command.parameters}')

            # Execute based on command type
            if robot_command.action == 'navigation':
                self.execute_navigation(robot_command.parameters)
            elif robot_command.action == 'manipulation':
                self.execute_manipulation(robot_command.parameters)
            elif robot_command.action == 'interaction':
                self.execute_interaction(robot_command.parameters)
            else:
                self.speak_response(f"I don't know how to {command_text}")
        else:
            self.speak_response(f"I didn't understand '{command_text}'. Can you repeat?")

    def execute_navigation(self, params: Dict[str, str]):
        """Execute navigation command"""
        location = params.get('location')

        if location:
            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            # Set pose based on location (this would come from a map)
            if location == 'kitchen_waypoint':
                goal_msg.pose.pose.position.x = 1.0
                goal_msg.pose.pose.position.y = 2.0
                goal_msg.pose.pose.orientation.w = 1.0
            elif location == 'living_room_waypoint':
                goal_msg.pose.pose.position.x = -1.0
                goal_msg.pose.pose.position.y = -1.0
                goal_msg.pose.pose.orientation.w = 1.0
            # Add more locations as needed

            # Send navigation goal
            self.nav_client.wait_for_server()
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self.navigation_done_callback)

            self.speak_response(f"Going to the {location.replace('_waypoint', '')}")
        else:
            self.speak_response("I need to know where to go")

    def execute_interaction(self, params: Dict[str, str]):
        """Execute interaction command"""
        if 'text' in params:
            # Robot says something
            response_msg = String()
            response_msg.data = params['text']
            self.speech_pub.publish(response_msg)
        elif 'topic' in params:
            # Robot provides information about a topic
            topic = params['topic']
            response = self.get_topic_info(topic)
            self.speak_response(response)

    def speak_response(self, text: str):
        """Publish speech response"""
        response_msg = String()
        response_msg.data = text
        self.speech_pub.publish(response_msg)
        self.get_logger().info(f'Robot says: {text}')

    def get_topic_info(self, topic: str) -> str:
        """Get information about a topic"""
        # This would integrate with knowledge base
        topic_responses = {
            'weather': 'The weather is sunny with a high of 75 degrees.',
            'time': f'The current time is {self.get_clock().now().seconds} seconds.',
            'robot': 'I am a helpful robot designed to assist with daily tasks.'
        }
        return topic_responses.get(topic, f"I don't have information about {topic}.")

    def navigation_done_callback(self, future):
        """Handle navigation completion"""
        goal_handle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.speak_response("I have reached my destination")
        else:
            self.speak_response("I couldn't reach the destination")
```

## Advanced Voice Processing Features

### Wake Word Detection
```python
import numpy as np
from scipy import signal

class WakeWordDetector:
    def __init__(self, wake_words=["robot", "hey robot", "assistant"]):
        self.wake_words = wake_words
        self.audio_buffer = np.array([])
        self.buffer_size = 16000 * 2  # 2 seconds of audio

    def detect_wake_word(self, audio_chunk):
        """Detect if wake word is present in audio chunk"""
        # Add new audio to buffer
        self.audio_buffer = np.concatenate([self.audio_buffer, audio_chunk])

        # Keep only recent audio
        if len(self.audio_buffer) > self.buffer_size:
            self.audio_buffer = self.audio_buffer[-self.buffer_size:]

        # For real implementation, you'd use a dedicated wake word detection model
        # This is a simplified version for demonstration
        return self._simple_wake_word_detection()

    def _simple_wake_word_detection(self):
        """Simple wake word detection (in practice, use specialized models)"""
        # This would actually use audio feature extraction and ML models
        # For now, we'll return True to indicate wake word detected
        return True  # Simplified for example
```

### Noise Reduction and Audio Enhancement
```python
from scipy import signal
import webrtcvad  # WebRTC Voice Activity Detection

class AudioPreprocessor:
    def __init__(self):
        # Initialize WebRTC VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Aggressiveness mode (0-3)

        # Audio parameters
        self.sample_rate = 16000
        self.frame_duration = 30  # ms
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000)

    def preprocess_audio(self, audio_data):
        """Preprocess audio for better Whisper performance"""
        # Apply noise reduction
        cleaned_audio = self._reduce_noise(audio_data)

        # Normalize audio levels
        normalized_audio = self._normalize_audio(cleaned_audio)

        # Detect voice activity
        voice_detected = self._detect_voice_activity(normalized_audio)

        return normalized_audio if voice_detected else None

    def _reduce_noise(self, audio_data):
        """Apply basic noise reduction"""
        # Apply a simple high-pass filter to remove low-frequency noise
        b, a = signal.butter(4, 100 / (self.sample_rate / 2), btype='high')
        filtered_audio = signal.filtfilt(b, a, audio_data)
        return filtered_audio

    def _normalize_audio(self, audio_data):
        """Normalize audio to consistent level"""
        # Calculate RMS and normalize
        rms = np.sqrt(np.mean(audio_data**2))
        target_rms = 0.05  # Target RMS level

        if rms > 0:
            normalization_factor = target_rms / rms
            normalized_audio = audio_data * normalization_factor
            # Clip to prevent overflow
            normalized_audio = np.clip(normalized_audio, -1.0, 1.0)
        else:
            normalized_audio = audio_data

        return normalized_audio

    def _detect_voice_activity(self, audio_data):
        """Detect if voice is present in audio"""
        # WebRTC VAD works on 10, 20, or 30ms frames
        # Convert to appropriate format
        audio_int16 = (audio_data * 32767).astype(np.int16)

        # Check voice activity in chunks
        voice_frames = 0
        total_frames = 0

        for i in range(0, len(audio_int16), self.frame_size):
            frame = audio_int16[i:i + self.frame_size]
            if len(frame) == self.frame_size:
                is_speech = self.vad.is_speech(
                    frame.tobytes(),
                    self.sample_rate
                )
                if is_speech:
                    voice_frames += 1
                total_frames += 1

        # Consider voice active if more than 30% of frames have speech
        return (voice_frames / max(total_frames, 1)) > 0.3 if total_frames > 0 else False
```

## Performance Optimization

### Model Quantization for Edge Deployment
```python
def optimize_whisper_for_edge(model_path, output_path):
    """Optimize Whisper model for edge deployment"""
    import onnx
    import onnxruntime as ort
    from onnxruntime.quantization import quantize_dynamic, QuantType

    # Load the model (this is conceptual - actual Whisper ONNX conversion is complex)
    # In practice, you'd use tools like optimum for Whisper optimization

    # Example of quantization (simplified)
    quantized_model_path = output_path + "_quantized.onnx"
    quantize_dynamic(
        model_input=model_path,
        model_output=quantized_model_path,
        op_types_to_quantize=['MatMul', 'Add'],
        per_channel=True,
        reduce_range=True,
        weight_type=QuantType.QUInt8
    )

    return quantized_model_path

def create_optimized_pipeline():
    """Create optimized Whisper pipeline for robotics"""
    # Use smaller model for real-time applications
    model_size = "tiny"  # or "base" depending on requirements

    # Load model with optimizations
    model = whisper.load_model(
        model_size,
        device="cuda" if torch.cuda.is_available() else "cpu",
        download_root="./models"
    )

    # Set model to evaluation mode
    model.eval()

    return model
```

## Practical Exercise: Voice-Controlled Robot

Create a complete voice command system with:

1. **Audio Input**: Real-time audio capture and preprocessing
2. **Whisper Integration**: Speech-to-text conversion
3. **Command Interpretation**: Natural language understanding
4. **ROS Integration**: Command execution through ROS 2
5. **Feedback System**: Audio/visual feedback to user
6. **Error Handling**: Robust error recovery

This exercise will provide hands-on experience with building a complete voice command pipeline for robotics applications.

## Troubleshooting Common Issues

### Audio Quality Issues
- **Background Noise**: Use noise reduction and voice activity detection
- **Audio Clipping**: Normalize audio levels before processing
- **Sampling Rate Mismatch**: Ensure audio is at 16kHz as expected by Whisper

### Recognition Problems
- **Accents**: Train or fine-tune models for specific accents
- **Domain Adaptation**: Add domain-specific vocabulary and phrases
- **Real-time Constraints**: Balance accuracy with processing speed

### Integration Challenges
- **Latency**: Optimize for real-time response requirements
- **Synchronization**: Coordinate audio, vision, and action systems
- **Resource Management**: Balance computational requirements

## Summary

The Whisper voice-to-command pipeline enables natural human-robot interaction through:
- **Robust Speech Recognition**: Handling diverse accents and noisy environments
- **Real-time Processing**: Optimized for interactive applications
- **Natural Language Understanding**: Interpreting commands in context
- **ROS Integration**: Seamless integration with robotic systems
- **Adaptive Processing**: Handling various audio conditions

This foundation enables robots to understand and respond to natural human speech, making them more accessible and intuitive to interact with.