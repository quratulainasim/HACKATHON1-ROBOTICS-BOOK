# Data Model: Physical AI Robotics Book

## Module
**Description**: A major section of the book covering a specific aspect of Physical AI
**Attributes**:
- title: String (e.g., "The Robotic Nervous System (ROS 2)")
- topics: List<String> (specific topics covered in the module)
- duration: Number (estimated weeks to complete)
- learning_outcomes: List<String> (skills/knowledge to acquire)
- prerequisites: List<String> (required knowledge before starting)

## Learning Outcome
**Description**: A measurable skill or knowledge point that students should acquire
**Attributes**:
- description: String (what the student should be able to do)
- module: Reference to Module (which module this outcome belongs to)
- difficulty: Enum (beginner, intermediate, advanced)
- assessment_method: String (how to verify the outcome is met)

## Weekly Breakdown
**Description**: Structured progression of topics across weeks
**Attributes**:
- week_number: Number (1-13)
- topics: List<String> (topics to cover in the week)
- learning_outcomes: List<Reference to Learning Outcome>
- activities: List<String> (practical exercises for the week)
- estimated_hours: Number (time required for week completion)

## Capstone Project
**Description**: Comprehensive final project integrating all concepts from the book
**Attributes**:
- title: String (e.g., "Autonomous Humanoid Robot")
- description: String (overview of the project)
- requirements: List<String> (technical requirements)
- modules_integrated: List<Reference to Module> (which modules are used)
- assessment_criteria: List<String> (how the project will be evaluated)
- timeline: String (duration for project completion)

## Topic
**Description**: Individual subject or concept covered within a module
**Attributes**:
- title: String (name of the topic)
- module: Reference to Module (which module contains this topic)
- content_type: Enum (theory, practical, example, exercise)
- duration: Number (estimated time to cover the topic)
- dependencies: List<Reference to Topic> (prerequisite topics)