import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Book sidebar for the Physical AI Robotics Book
  bookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro/physical-ai-overview'],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/intro',
        'module-1-ros2/nodes-topics-services',
        'module-1-ros2/python-ros-integration',
        'module-1-ros2/urdf-humanoid-robots',
        'module-1-ros2/middleware-real-time-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-simulation/intro',
        'module-2-simulation/physics-simulation',
        'module-2-simulation/unity-rendering',
        'module-2-simulation/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-nvidia-isaac/intro',
        'module-3-nvidia-isaac/isaac-sim',
        'module-3-nvidia-isaac/synthetic-data-perception',
        'module-3-nvidia-isaac/navigation-planning',
        'module-3-nvidia-isaac/isaac-sim-practical-exercises',
        'module-3-nvidia-isaac/knowledge-checks',
        'module-3-nvidia-isaac/summary-next-steps',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/intro',
        'module-4-vla/whisper-voice-command',
        'module-4-vla/llm-task-planning',
        'module-4-vla/ros-action-generation',
        'module-4-vla/vla-practical-exercises',
      ],
    },
    {
      type: 'category',
      label: 'Weekly Breakdown',
      items: ['weekly-breakdown/week-1-2', 'weekly-breakdown/week-3-4', 'weekly-breakdown/week-5-8', 'weekly-breakdown/week-9-13'],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone-project/intro', 'capstone-project/requirements', 'capstone-project/implementation', 'capstone-project/implementation-details'],
    },
    {
      type: 'category',
      label: 'Conclusion',
      items: ['conclusion/summary', 'conclusion/next-steps'],
    },
    {
      type: 'doc',
      id: 'README',
    },
  ],
};

export default sidebars;
