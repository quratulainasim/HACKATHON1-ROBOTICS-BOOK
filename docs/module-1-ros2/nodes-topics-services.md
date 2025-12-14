---
title: ROS 2 Nodes, Topics, and Services
sidebar_position: 2
---

# ROS 2 Nodes, Topics, and Services

## Understanding ROS 2 Architecture

ROS 2 (Robot Operating System 2) is not an operating system but rather a middleware framework that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### Core Architecture Components

The ROS 2 architecture is built around several core concepts:

- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous goal-oriented communication
- **Parameters**: Configuration values that can be changed at runtime

## Nodes: The Building Blocks

Nodes are the fundamental building blocks of a ROS 2 system. Each node is a process that performs a specific task and communicates with other nodes through topics, services, or actions.

### Creating a Node

A ROS 2 node is typically implemented as a class that inherits from `rclpy.Node`. Here's a basic example:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal_node')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    # Spin to keep the node alive
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Best Practices

- Each node should have a single, well-defined responsibility
- Use meaningful names for nodes to improve system understanding
- Include proper logging for debugging and monitoring
- Implement proper cleanup in the node destructor

## Topics: Publisher-Subscriber Communication

Topics enable asynchronous, many-to-many communication through a publish-subscribe pattern. Publishers send messages to a topic, and subscribers receive messages from that topic.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topic Characteristics

- **Asynchronous**: Publishers and subscribers don't need to be active simultaneously
- **Many-to-many**: Multiple publishers can send to the same topic, and multiple subscribers can receive from it
- **Decoupled**: Publishers and subscribers are unaware of each other's existence
- **Typed**: Messages have specific types defined in `.msg` files

## Services: Request-Response Communication

Services provide synchronous, request-response communication between nodes. A client sends a request to a service, and the service sends back a response.

### Service Definition

Services are defined in `.srv` files that specify the request and response message types:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Service Server Example

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Characteristics

- **Synchronous**: The client waits for a response from the service
- **One-to-one**: Each service request is handled by one service server
- **Request-response**: Clear input and output defined in the service interface
- **Blocking**: The client is blocked until the service responds

## Quality of Service (QoS) Settings

ROS 2 provides Quality of Service (QoS) settings to control how messages are delivered between nodes. QoS settings include:

- **Reliability**: Whether messages are delivered reliably or best-effort
- **Durability**: Whether late-joining subscribers receive the last message
- **History**: How many messages to store
- **Deadline**: Maximum time between consecutive messages

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Create a QoS profile for reliable communication
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# Use the QoS profile when creating a publisher
publisher = node.create_publisher(String, 'topic', qos_profile)
```

## Practical Exercise: Temperature Monitoring System

Let's create a simple temperature monitoring system to practice what we've learned:

1. Create a temperature sensor node that publishes temperature readings
2. Create a temperature display node that subscribes to temperature readings
3. Create a temperature alert service that checks if temperature is within safe limits

This exercise will help you understand how nodes, topics, and services work together in a real-world scenario.

## Summary

In this section, we've covered:

- The fundamental concepts of ROS 2 architecture
- How to create and use nodes for computation
- The publish-subscribe pattern for asynchronous communication
- The request-response pattern for synchronous communication
- Quality of Service settings for controlling message delivery

Understanding these concepts is crucial for developing any ROS 2-based robot system, as they form the foundation of all robot communication.