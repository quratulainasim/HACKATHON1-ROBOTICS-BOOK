---
title: Python-ROS Integration with rclpy
sidebar_position: 3
---

# Python-ROS Integration with rclpy

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API for creating ROS 2 nodes, publishing and subscribing to topics, making service calls, and more. rclpy is built on top of the ROS Client Library (rcl) and the underlying middleware (RMW).

## Setting Up Python-ROS Environment

Before we dive into rclpy, let's understand how to set up your Python environment for ROS 2 development.

### Installation and Setup

1. **Install ROS 2**: Follow the official installation guide for your platform
2. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution
   ```
3. **Verify installation**:
   ```bash
   python3 -c "import rclpy; print('rclpy imported successfully')"
   ```

### Creating a Python Package for ROS 2

ROS 2 Python packages follow the standard Python package structure with additional ROS-specific files:

```
my_robot_package/
├── package.xml          # Package manifest
├── setup.py             # Python setup script
├── setup.cfg            # Installation instructions
├── my_robot_package/    # Python module
│   ├── __init__.py
│   └── nodes/
│       ├── __init__.py
│       └── my_node.py
└── test/
    └── test_my_node.py
```

## Core rclpy Concepts

### Initializing and Shutting Down

Every ROS 2 Python program must initialize rclpy before creating nodes and must properly shut down when finished:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create and use nodes here
    node = MyNode()

    # Keep the node running
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Node Class

The most common pattern is to create a class that inherits from `rclpy.node.Node`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyRobotNode(Node):
    def __init__(self):
        # Initialize the parent class with a node name
        super().__init__('my_robot_node')

        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)

        # Create a subscriber
        self.subscription = self.create_subscription(
            String,
            'command',
            self.command_callback,
            10)

        # Create a timer
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('MyRobotNode initialized')

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Process the command here

    def timer_callback(self):
        msg = String()
        msg.data = 'Robot is running'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced rclpy Features

### Parameters

Nodes can use parameters to make their behavior configurable:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('safety_enabled', True)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_enabled = self.get_parameter('safety_enabled').value

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Robot: {self.robot_name}, Max Speed: {self.max_speed}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_speed' and param.type_ == Parameter.Type.DOUBLE:
                if param.value > 5.0:
                    return SetParametersResult(successful=False, reason='Max speed too high')
        return SetParametersResult(successful=True)

from rclpy.parameter_service import SetParametersResult
```

### Timers and Callbacks

Timers allow you to execute code at regular intervals:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create a publisher
        self.publisher = self.create_publisher(Float64, 'sensor_data', 10)

        # Create a timer that calls a callback every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.counter = 0.0

    def timer_callback(self):
        msg = Float64()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 0.1
        self.get_logger().info(f'Published: {msg.data}')
```

### Executors

Executors manage the execution of callbacks. The default executor is SingleThreadedExecutor, but you can also use MultiThreadedExecutor:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

class ExecutorNode(Node):
    def __init__(self):
        super().__init__('executor_node')
        self.publisher = self.create_publisher(String, 'data', 10)

    def publish_data(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)

def main():
    rclpy.init()

    node1 = ExecutorNode()
    node2 = ExecutorNode()

    # Create a multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

## Best Practices for Python-ROS Integration

### Error Handling

Always implement proper error handling in your ROS 2 nodes:

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        try:
            self.declare_parameter('critical_param', 'default')
            self.critical_value = self.get_parameter('critical_param').value
        except ParameterNotDeclaredException:
            self.get_logger().error('Critical parameter not declared')
            raise

        # Always handle potential exceptions in callbacks
        self.subscription = self.create_subscription(
            String,
            'input',
            self.safe_callback,
            10)

    def safe_callback(self, msg):
        try:
            # Process message
            processed = self.process_message(msg)
            self.publish_result(processed)
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
```

### Resource Management

Properly manage resources to prevent memory leaks:

```python
class ResourceNode(Node):
    def __init__(self):
        super().__init__('resource_node')
        self.create_timer(1.0, self.cleanup_callback)
        self.resources = []

    def destroy_node(self):
        # Clean up resources before destroying the node
        for resource in self.resources:
            resource.cleanup()
        super().destroy_node()

    def cleanup_callback(self):
        # Periodic cleanup of temporary resources
        # Implement cleanup logic here
        pass
```

## Practical Exercise: Robot Controller Node

Create a Python node that:

1. Subscribes to velocity commands
2. Publishes motor control messages
3. Uses parameters for robot configuration
4. Implements proper error handling
5. Uses timers for periodic status updates

This exercise will help you integrate all the concepts we've covered in this section.

## Common Pitfalls and Troubleshooting

### Import Issues

Make sure to source your ROS 2 environment before running Python scripts:

```bash
source /opt/ros/humble/setup.bash
python3 my_script.py
```

### Node Lifecycle

Always follow the proper initialization and shutdown sequence to prevent resource leaks and ensure proper cleanup.

### Threading Considerations

Be careful when using threading with rclpy, as some operations are not thread-safe. Use the appropriate executor for multi-threaded scenarios.

## Summary

In this section, we've covered:

- Setting up Python-ROS integration with rclpy
- Creating nodes with proper initialization and shutdown
- Working with parameters, timers, and executors
- Best practices for robust Python-ROS applications
- Common pitfalls and how to avoid them

Understanding rclpy is essential for developing ROS 2 applications in Python, which is one of the most popular languages for robotics development.