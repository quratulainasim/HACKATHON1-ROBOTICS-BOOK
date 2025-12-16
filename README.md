# Physical AI Robotics Book & Authentication Integration    

This repository contains two comprehensive projects:

1. **Physical AI Robotics Book** - A complete educational resource on Physical AI and Humanoid Robotics, structured into four modules covering ROS 2, simulation environments, NVIDIA Isaac, and Vision-Language-Action systems.

2. **Authentication Integration System** - A secure authentication system using Better Auth and Neon PostgreSQL, designed to work with Docusaurus frontends.

Both projects are complete and ready for deployment and implementation.

## Physical AI Robotics Book Overview

The Physical AI Robotics Book provides a comprehensive understanding of Physical AI and humanoid robotics. The book is structured into four modules that build upon each other to provide both theoretical knowledge and practical implementation skills, culminating in a capstone project implementing an autonomous humanoid robot.

### Book Modules

1. **Module 1: The Robotic Nervous System (ROS 2)** - Covers ROS 2 fundamentals, Nodes, Topics, Services, Python integration, and URDF for humanoid robots
2. **Module 2: The Digital Twin (Gazebo & Unity)** - Focuses on simulation environments, physics modeling, and sensor simulation
3. **Module 3: The AI-Robot Brain (NVIDIA Isaac)** - Explores advanced simulation, perception systems, and navigation
4. **Module 4: Vision-Language-Action (VLA)** - Integrates AI systems with robotics, including natural language processing

## Authentication Integration System Overview

A complete authentication system designed for Docusaurus-based documentation sites with:

- Backend-forward architecture using Better Auth
- Neon PostgreSQL for secure user data storage
- Secure API endpoints for frontend consumption
- Session management with JWT and refresh tokens
- OAuth provider integration capabilities
- Comprehensive security measures (rate limiting, CSRF protection, etc.)

## Prerequisites

### For Robotics Book:
- Basic programming knowledge (Python preferred)
- Understanding of fundamental robotics concepts
- Access to a computer capable of running robotics simulation software
- Familiarity with command-line tools

### For Authentication System:
- Node.js 18+
- npm or yarn package manager
- Neon PostgreSQL account
- Git for version control

## Technical Setup

### Required Software for Robotics Book:
- ROS 2 (Humble Hawksbill or later recommended)
- Gazebo Garden or later
- Unity (Personal or Pro license)
- NVIDIA Isaac Sim (requires Isaac ROS assets)
- Python 3.8 or later

### Required Software for Authentication:
- Node.js 18+
- Neon PostgreSQL database
- Git for version control

### Recommended Development Environment:
- Ubuntu 22.04 LTS (for ROS 2 compatibility) or Windows with WSL2
- VS Code with appropriate extensions
- Docker for consistent environment setup (optional but recommended)

## Repository Structure

```
docs/                       # Docusaurus documentation (book content + auth docs)
├── intro/                  # Physical AI introduction
├── module-1-ros2/          # ROS 2 fundamentals
├── module-2-simulation/    # Simulation environments
├── module-3-nvidia-isaac/  # NVIDIA Isaac content
├── module-4-vla/           # Vision-Language-Action systems
├── weekly-breakdown/       # Structured weekly progression
├── capstone-project/       # Comprehensive final project
├── conclusion/             # Summary and next steps
└── auth/                   # Authentication system documentation

specs/                      # Feature specifications
├── 001-physical-ai-book/   # Robotics book specification
└── 002-auth-integration/   # Authentication system specification

src/                        # Docusaurus source code
static/                     # Static assets
history/                    # Prompt history records
```

## Features

### Robotics Book Features:
- **Complete Documentation**: All modules with detailed explanations and code examples
- **Practical Exercises**: Hands-on exercises for each concept
- **Knowledge Checks**: Assessments to validate understanding
- **Capstone Project**: Implementation of an autonomous humanoid robot
- **Weekly Breakdown**: Structured 13-week curriculum
- **Professional Styling**: Animated gradients and interactive elements

### Authentication System Features:
- **Secure Architecture**: Backend-forward design with no secrets exposed to frontend
- **API Integration**: Clean endpoints for Docusaurus frontend consumption
- **Database Security**: Neon PostgreSQL with encrypted data storage
- **Session Management**: JWT tokens with refresh rotation
- **OAuth Ready**: Extensible for Google, GitHub, and other providers
- **Security First**: Rate limiting, CSRF protection, and audit logging

## Getting Started

### For the Robotics Book:
1. Navigate to the `docs/` directory to access all book content
2. Follow the weekly breakdown structure to progress through the material
3. Complete the practical exercises and capstone project

### For the Authentication System:
1. Set up your development environment with Node.js and Neon PostgreSQL
2. Review the specification in `specs/002-auth-integration/`
3. Follow the quickstart guide to implement the authentication system

## License

This content is made available under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
