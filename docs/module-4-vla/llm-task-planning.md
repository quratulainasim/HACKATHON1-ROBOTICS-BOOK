---
title: LLM-Powered Task Planning for Robotics
sidebar_position: 3
---

# LLM-Powered Task Planning for Robotics

## Introduction to LLM-Based Task Planning

Large Language Models (LLMs) have revolutionized the field of robotics by enabling sophisticated task planning capabilities that were previously impossible with traditional rule-based systems. LLMs bring several key advantages to robotic task planning:

- **Natural Language Understanding**: Robots can interpret complex, natural language commands
- **Common-Sense Reasoning**: Access to vast amounts of world knowledge for planning
- **Adaptive Planning**: Ability to handle novel situations and adapt to changes
- **Hierarchical Planning**: Decomposition of complex tasks into manageable subtasks

## The Role of LLMs in Robotics

### Traditional vs. LLM-Based Planning

**Traditional Planning Approaches:**
- Rule-based systems with predefined behaviors
- Limited to pre-programmed tasks
- Require extensive manual programming for new capabilities
- Struggle with ambiguity and novel situations

**LLM-Based Planning:**
- Natural language interface for task specification
- Access to common-sense knowledge and reasoning
- Ability to generalize to new tasks and situations
- Flexible decomposition of complex goals

### Integration Architecture

LLMs serve as the "cognitive layer" in robotic systems, bridging high-level task understanding with low-level execution:

```
Natural Language Command
         ↓
    LLM Task Planner
         ↓
  Task Decomposition
         ↓
   Action Sequences
         ↓
   Robot Execution
```

## LLM Task Planning Architecture

### Core Components

```python
import openai
import json
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

class TaskStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class RobotAction:
    """Represents a single robot action"""
    action_type: str
    parameters: Dict[str, Any]
    description: str
    preconditions: List[str]
    effects: List[str]

@dataclass
class TaskPlan:
    """Represents a complete task plan"""
    task_id: str
    original_goal: str
    subtasks: List[RobotAction]
    status: TaskStatus
    current_step: int = 0

class LLMBasedTaskPlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        self.client = openai.OpenAI(api_key=api_key)
        self.model = model
        self.robot_capabilities = self._define_robot_capabilities()
        self.current_plan = None

    def _define_robot_capabilities(self) -> Dict[str, Any]:
        """Define what the robot can do"""
        return {
            "navigation": {
                "actions": ["move_to", "go_to", "navigate_to"],
                "parameters": ["location", "waypoint"],
                "description": "Move robot to specified location"
            },
            "manipulation": {
                "actions": ["pick_up", "grasp", "place", "release"],
                "parameters": ["object", "location"],
                "description": "Manipulate objects in the environment"
            },
            "interaction": {
                "actions": ["speak", "listen", "ask"],
                "parameters": ["text", "question"],
                "description": "Interact with humans"
            },
            "perception": {
                "actions": ["detect", "identify", "locate"],
                "parameters": ["object_type", "location"],
                "description": "Perceive and identify objects"
            }
        }

    def create_task_plan(self, goal: str) -> Optional[TaskPlan]:
        """Create a task plan from a natural language goal"""
        try:
            # Define the planning prompt
            prompt = self._create_planning_prompt(goal)

            # Call the LLM
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,  # Lower temperature for more consistent planning
                response_format={"type": "json_object"}
            )

            # Parse the response
            plan_data = json.loads(response.choices[0].message.content)

            # Create task plan
            task_plan = TaskPlan(
                task_id=f"task_{len(str(goal))}_{hash(goal) % 10000}",
                original_goal=goal,
                subtasks=self._parse_actions(plan_data["actions"]),
                status=TaskStatus.PENDING
            )

            self.current_plan = task_plan
            return task_plan

        except Exception as e:
            print(f"Error creating task plan: {e}")
            return None

    def _create_planning_prompt(self, goal: str) -> str:
        """Create the prompt for task planning"""
        return f"""
        Given the following goal: "{goal}"

        Please break this down into a sequence of specific robot actions from the following capabilities:

        {json.dumps(self.robot_capabilities, indent=2)}

        Return your response as a JSON object with the following structure:
        {{
            "actions": [
                {{
                    "action_type": "string",
                    "parameters": {{"param_name": "param_value"}},
                    "description": "brief description of what this action does"
                }}
            ],
            "reasoning": "brief explanation of your planning approach"
        }}

        Ensure that:
        1. Each action is specific and executable
        2. Actions are in logical sequence
        3. All necessary preconditions are met before each action
        4. The sequence achieves the stated goal
        """

    def _get_system_prompt(self) -> str:
        """System prompt to guide the LLM behavior"""
        return """
        You are an expert robotic task planner. Your job is to decompose high-level goals into specific, executable robot actions.
        The robot has capabilities for navigation, manipulation, interaction, and perception.
        Always return valid JSON with properly structured actions that the robot can execute.
        Consider the physical constraints and safety requirements when planning.
        """

    def _parse_actions(self, action_list: List[Dict]) -> List[RobotAction]:
        """Parse LLM response into RobotAction objects"""
        actions = []
        for action_data in action_list:
            action = RobotAction(
                action_type=action_data["action_type"],
                parameters=action_data.get("parameters", {}),
                description=action_data.get("description", ""),
                preconditions=[],  # Would be populated based on context
                effects=[]  # Would be populated based on context
            )
            actions.append(action)
        return actions
```

## Advanced Task Planning Techniques

### Hierarchical Task Planning

```python
class HierarchicalTaskPlanner(LLMBasedTaskPlanner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.task_library = self._initialize_task_library()

    def _initialize_task_library(self) -> Dict[str, Any]:
        """Initialize library of common tasks and their decompositions"""
        return {
            "get_object_from_room": {
                "description": "Navigate to a room, identify an object, and bring it back",
                "subtasks": [
                    {"action_type": "navigate_to", "parameters": {"location": "{room}"}},
                    {"action_type": "identify", "parameters": {"object": "{object}"}},
                    {"action_type": "grasp", "parameters": {"object": "{object}"}},
                    {"action_type": "navigate_to", "parameters": {"location": "{destination}"}},
                    {"action_type": "release", "parameters": {"object": "{object}"}}
                ]
            },
            "room_cleanup": {
                "description": "Pick up all objects of specified types from a room",
                "subtasks": [
                    {"action_type": "navigate_to", "parameters": {"location": "{room}"}},
                    {"action_type": "detect", "parameters": {"object_type": "{object_types}"}},
                    {"action_type": "grasp", "parameters": {"object": "{object}"}},
                    {"action_type": "navigate_to", "parameters": {"location": "{disposal_location}"}},
                    {"action_type": "release", "parameters": {"object": "{object}"}}
                ]
            }
        }

    def create_hierarchical_plan(self, goal: str) -> Optional[TaskPlan]:
        """Create a plan using hierarchical decomposition"""
        # First, check if the goal matches any known task patterns
        known_task = self._match_known_task(goal)

        if known_task:
            # Use the known task decomposition
            return self._create_plan_from_known_task(goal, known_task)
        else:
            # Fall back to LLM-based planning
            return self.create_task_plan(goal)

    def _match_known_task(self, goal: str) -> Optional[Dict]:
        """Match the goal to known task patterns"""
        # This could use more sophisticated pattern matching
        goal_lower = goal.lower()

        for task_name, task_info in self.task_library.items():
            if any(keyword in goal_lower for keyword in task_info.get("keywords", [])):
                return task_info

        return None

    def _create_plan_from_known_task(self, goal: str, task_info: Dict) -> TaskPlan:
        """Create a plan from a known task template"""
        # Extract parameters from the goal
        parameters = self._extract_parameters(goal, task_info)

        # Instantiate the template with specific parameters
        subtasks = []
        for template_action in task_info["subtasks"]:
            action = template_action.copy()
            # Replace placeholders with actual values
            for param_name, param_value in parameters.items():
                if isinstance(action["parameters"], dict):
                    for key, value in action["parameters"].items():
                        if isinstance(value, str) and f"{{{param_name}}}" in value:
                            action["parameters"][key] = value.replace(f"{{{param_name}}}", param_value)
            subtasks.append(RobotAction(
                action_type=action["action_type"],
                parameters=action["parameters"],
                description=f"Execute {action['action_type']} with {action['parameters']}",
                preconditions=[],
                effects=[]
            ))

        return TaskPlan(
            task_id=f"htask_{hash(goal) % 10000}",
            original_goal=goal,
            subtasks=subtasks,
            status=TaskStatus.PENDING
        )

    def _extract_parameters(self, goal: str, task_info: Dict) -> Dict[str, str]:
        """Extract parameters from the goal text"""
        # Simple parameter extraction - in practice, this would be more sophisticated
        parameters = {}

        # Look for common patterns like "get X from Y" or "clean up Z"
        import re

        # Extract object
        object_match = re.search(r'(?:get|bring|fetch|take|pick up)\s+(?:the\s+)?(\w+)', goal, re.IGNORECASE)
        if object_match:
            parameters["object"] = object_match.group(1)

        # Extract room/location
        location_match = re.search(r'(?:from|in|at)\s+(?:the\s+)?(\w+)', goal, re.IGNORECASE)
        if location_match:
            parameters["room"] = location_match.group(1)

        # Extract destination
        to_match = re.search(r'(?:to|toward)\s+(?:the\s+)?(\w+)', goal, re.IGNORECASE)
        if to_match:
            parameters["destination"] = to_match.group(1)

        return parameters
```

## Context-Aware Planning

### World State Integration

```python
class ContextAwarePlanner(HierarchicalTaskPlanner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.world_state = {}
        self.location_map = {}  # Maps location names to coordinates
        self.object_map = {}    # Maps object names to locations

    def update_world_state(self, sensor_data: Dict[str, Any]):
        """Update the planner's understanding of the world"""
        # Update location map from navigation system
        if "locations" in sensor_data:
            self.location_map.update(sensor_data["locations"])

        # Update object map from perception system
        if "objects" in sensor_data:
            for obj in sensor_data["objects"]:
                self.object_map[obj["name"]] = obj["location"]

        # Update other relevant state information
        self.world_state.update(sensor_data)

    def create_context_aware_plan(self, goal: str) -> Optional[TaskPlan]:
        """Create a plan that considers current world state"""
        # Include world state in the planning prompt
        context_info = self._get_context_info()

        try:
            prompt = self._create_contextual_planning_prompt(goal, context_info)

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_contextual_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            plan_data = json.loads(response.choices[0].message.content)

            # Enhance actions with context-specific information
            enhanced_actions = self._enhance_actions_with_context(
                plan_data["actions"], context_info
            )

            task_plan = TaskPlan(
                task_id=f"ctx_task_{hash(goal) % 10000}",
                original_goal=goal,
                subtasks=self._parse_actions(enhanced_actions),
                status=TaskStatus.PENDING
            )

            self.current_plan = task_plan
            return task_plan

        except Exception as e:
            print(f"Error creating contextual task plan: {e}")
            return self.create_task_plan(goal)  # Fallback to basic planning

    def _get_context_info(self) -> Dict[str, Any]:
        """Get current context information"""
        return {
            "current_location": self.world_state.get("robot_location", "unknown"),
            "available_objects": list(self.object_map.keys()),
            "known_locations": list(self.location_map.keys()),
            "robot_battery": self.world_state.get("battery_level", 100),
            "current_time": self.world_state.get("timestamp", "unknown")
        }

    def _create_contextual_planning_prompt(self, goal: str, context: Dict) -> str:
        """Create planning prompt with context information"""
        return f"""
        Goal: {goal}

        Current Context:
        - Robot Location: {context['current_location']}
        - Available Objects: {', '.join(context['available_objects'])}
        - Known Locations: {', '.join(context['known_locations'])}
        - Battery Level: {context['robot_battery']}%
        - Current Time: {context['current_time']}

        Robot Capabilities:
        {json.dumps(self.robot_capabilities, indent=2)}

        Given the goal and current context, create a task plan that:
        1. Takes into account the robot's current location
        2. Uses known locations for navigation
        3. Considers available objects
        4. Accounts for battery level (avoid long journeys if battery is low)
        5. Follows the same JSON structure as before

        Consider the most efficient path given the current state.
        """

    def _get_contextual_system_prompt(self) -> str:
        """System prompt that emphasizes context awareness"""
        return """
        You are an expert robotic task planner that considers the current state of the world.
        Always take into account the robot's current location, available objects, known locations,
        battery level, and other contextual information when creating plans.
        Create efficient plans that consider the current state to minimize unnecessary actions.
        """

    def _enhance_actions_with_context(self, actions: List[Dict], context: Dict) -> List[Dict]:
        """Enhance actions with context-specific information"""
        enhanced_actions = []

        for action in actions:
            enhanced_action = action.copy()

            # If it's a navigation action, add specific coordinates
            if action["action_type"] in ["move_to", "go_to", "navigate_to", "navigate"]:
                location = action["parameters"].get("location")
                if location and location in self.location_map:
                    enhanced_action["parameters"]["coordinates"] = self.location_map[location]

            # If it's an object manipulation action, add location information
            if action["action_type"] in ["pick_up", "grasp", "identify", "locate"]:
                obj = action["parameters"].get("object")
                if obj and obj in self.object_map:
                    enhanced_action["parameters"]["object_location"] = self.object_map[obj]

            enhanced_actions.append(enhanced_action)

        return enhanced_actions
```

## Integration with ROS 2

### ROS 2 Task Planning Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class LLMTaskPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_task_planning_node')

        # Initialize the LLM-based planner
        api_key = self.declare_parameter('openai_api_key', '').get_parameter_value().string_value
        if not api_key:
            self.get_logger().error('OpenAI API key not provided')
            return

        self.planner = ContextAwarePlanner(api_key=api_key)

        # Publishers and subscribers
        self.task_goal_sub = self.create_subscription(
            String,
            'task_goals',
            self.task_goal_callback,
            10
        )

        self.action_status_pub = self.create_publisher(
            String,
            'action_status',
            10
        )

        self.plan_pub = self.create_publisher(
            String,
            'task_plan',
            10
        )

        # Action clients for robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, ManipulationAction, 'manipulation')

        # World state subscribers
        self.world_state_sub = self.create_subscription(
            String,
            'world_state',
            self.world_state_callback,
            10
        )

        # Task execution timer
        self.task_execution_timer = self.create_timer(
            0.1,  # 10 Hz
            self.execute_next_action,
            callback_group=ReentrantCallbackGroup()
        )

        self.current_task_plan = None
        self.task_execution_active = False

        self.get_logger().info('LLM Task Planning Node initialized')

    def task_goal_callback(self, msg: String):
        """Handle new task goals"""
        goal = msg.data
        self.get_logger().info(f'Received task goal: {goal}')

        # Create a new task plan
        task_plan = self.planner.create_context_aware_plan(goal)

        if task_plan:
            self.current_task_plan = task_plan
            self.task_execution_active = True

            # Publish the plan
            plan_msg = String()
            plan_msg.data = json.dumps({
                'task_id': task_plan.task_id,
                'original_goal': task_plan.original_goal,
                'actions': [action.description for action in task_plan.subtasks]
            })
            self.plan_pub.publish(plan_msg)

            self.get_logger().info(f'Created plan with {len(task_plan.subtasks)} actions')
        else:
            self.get_logger().error('Failed to create task plan')

    def world_state_callback(self, msg: String):
        """Update world state from sensor data"""
        try:
            sensor_data = json.loads(msg.data)
            self.planner.update_world_state(sensor_data)
            self.get_logger().debug('Updated world state')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid world state message')

    def execute_next_action(self):
        """Execute the next action in the current plan"""
        if not self.current_task_plan or not self.task_execution_active:
            return

        if self.current_task_plan.current_step >= len(self.current_task_plan.subtasks):
            # Task plan completed
            self.task_execution_active = False
            self.current_task_plan.status = TaskStatus.COMPLETED
            self.get_logger().info('Task plan completed successfully')
            return

        current_action = self.current_task_plan.subtasks[self.current_task_plan.current_step]

        # Execute based on action type
        if current_action.action_type in ['move_to', 'go_to', 'navigate_to', 'navigate']:
            self.execute_navigation_action(current_action)
        elif current_action.action_type in ['pick_up', 'grasp', 'place', 'release']:
            self.execute_manipulation_action(current_action)
        elif current_action.action_type in ['speak', 'listen', 'ask']:
            self.execute_interaction_action(current_action)
        else:
            self.get_logger().warn(f'Unknown action type: {current_action.action_type}')
            self.current_task_plan.current_step += 1

    def execute_navigation_action(self, action: RobotAction):
        """Execute navigation action"""
        location = action.parameters.get('location')

        if not location:
            self.get_logger().error('Navigation action missing location parameter')
            self.current_task_plan.current_step += 1
            return

        # Look up coordinates if available
        coordinates = action.parameters.get('coordinates')
        if not coordinates:
            # Try to get from location map
            coordinates = self.planner.location_map.get(location)

        if coordinates:
            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            goal_msg.pose.pose.position.x = coordinates['x']
            goal_msg.pose.pose.position.y = coordinates['y']
            goal_msg.pose.pose.orientation.w = coordinates.get('w', 1.0)

            # Send navigation goal
            self.nav_client.wait_for_server()
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self.navigation_done_callback)

            self.publish_action_status(f"Navigating to {location}")
        else:
            self.get_logger().error(f'Unknown location: {location}')
            self.current_task_plan.current_step += 1

    def execute_manipulation_action(self, action: RobotAction):
        """Execute manipulation action"""
        obj = action.parameters.get('object')

        if not obj:
            self.get_logger().error('Manipulation action missing object parameter')
            self.current_task_plan.current_step += 1
            return

        # Create manipulation goal
        goal_msg = ManipulationAction.Goal()
        goal_msg.object_name = obj
        goal_msg.action_type = action.action_type

        # Add location information if available
        if 'object_location' in action.parameters:
            goal_msg.object_pose.header.frame_id = 'map'
            goal_msg.object_pose.pose.position.x = action.parameters['object_location']['x']
            goal_msg.object_pose.pose.position.y = action.parameters['object_location']['y']

        # Send manipulation goal
        self.manipulation_client.wait_for_server()
        future = self.manipulation_client.send_goal_async(goal_msg)
        future.add_done_callback(self.manipulation_done_callback)

        self.publish_action_status(f"Manipulating {obj}")

    def execute_interaction_action(self, action: RobotAction):
        """Execute interaction action"""
        text = action.parameters.get('text', '')

        # Publish speech command
        speech_msg = String()
        speech_msg.data = text
        self.speech_pub.publish(speech_msg)

        self.publish_action_status(f"Speaking: {text}")

        # Move to next action immediately for interaction
        self.current_task_plan.current_step += 1

    def navigation_done_callback(self, future):
        """Handle navigation completion"""
        goal_handle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation completed successfully')
            self.current_task_plan.current_step += 1
        else:
            self.get_logger().error('Navigation failed')
            self.current_task_plan.status = TaskStatus.FAILED
            self.task_execution_active = False

    def manipulation_done_callback(self, future):
        """Handle manipulation completion"""
        goal_handle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Manipulation completed successfully')
            self.current_task_plan.current_step += 1
        else:
            self.get_logger().error('Manipulation failed')
            self.current_task_plan.status = TaskStatus.FAILED
            self.task_execution_active = False

    def publish_action_status(self, status: str):
        """Publish action execution status"""
        status_msg = String()
        status_msg.data = status
        self.action_status_pub.publish(status_msg)
```

## Error Handling and Recovery

### Adaptive Planning with Error Recovery

```python
class AdaptiveTaskPlanner(ContextAwarePlanner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.execution_history = []
        self.error_recovery_strategies = self._define_recovery_strategies()

    def _define_recovery_strategies(self) -> Dict[str, Any]:
        """Define strategies for handling different types of errors"""
        return {
            "navigation_failure": {
                "retry": True,
                "alternative_route": True,
                "ask_for_help": True
            },
            "object_not_found": {
                "expand_search": True,
                "ask_for_location": True,
                "skip_action": True
            },
            "grasp_failure": {
                "retry_grasp": True,
            },
            "communication_failure": {
                "retry": True,
                "timeout": True
            }
        }

    def handle_execution_error(self, error_type: str, error_details: Dict[str, Any]) -> bool:
        """Handle execution errors and attempt recovery"""
        strategy = self.error_recovery_strategies.get(error_type)

        if not strategy:
            self.get_logger().error(f'No recovery strategy for error type: {error_type}')
            return False

        # Implement recovery based on strategy
        if strategy.get("retry") and error_details.get("retry_count", 0) < 3:
            error_details["retry_count"] = error_details.get("retry_count", 0) + 1
            return self.retry_current_action(error_details)

        elif strategy.get("alternative_route") and error_type == "navigation_failure":
            return self.find_alternative_route(error_details)

        elif strategy.get("ask_for_help"):
            return self.request_human_assistance(error_type, error_details)

        elif strategy.get("skip_action"):
            return self.skip_current_action()

        return False

    def retry_current_action(self, error_details: Dict[str, Any]) -> bool:
        """Retry the current action"""
        if self.current_plan and self.current_plan.current_step < len(self.current_plan.subtasks):
            self.get_logger().info(f'Retrying action: {error_details.get("action", "unknown")}')
            # Reset action status and allow re-execution
            return True
        return False

    def find_alternative_route(self, error_details: Dict[str, Any]) -> bool:
        """Find an alternative route for navigation"""
        # This would integrate with path planning algorithms
        self.get_logger().info('Looking for alternative route...')

        # For now, return True to indicate recovery attempt
        return True

    def request_human_assistance(self, error_type: str, error_details: Dict[str, Any]) -> bool:
        """Request human assistance for error resolution"""
        assistance_msg = self.generate_assistance_request(error_type, error_details)

        # Publish request for help
        self.publish_assistance_request(assistance_msg)

        # Wait for human response (in real system, this would be asynchronous)
        return True

    def generate_assistance_request(self, error_type: str, error_details: Dict[str, Any]) -> str:
        """Generate a natural language request for human assistance"""
        if error_type == "object_not_found":
            obj = error_details.get("object", "unknown")
            location = error_details.get("location", "unknown")
            return f"I couldn't find the {obj} in the {location}. Can you help me locate it?"

        elif error_type == "navigation_failure":
            destination = error_details.get("destination", "unknown")
            return f"I'm having trouble reaching the {destination}. Can you guide me there?"

        return f"I encountered a problem: {error_details.get('message', 'unknown issue')}. Can you help?"

    def publish_assistance_request(self, message: str):
        """Publish assistance request"""
        # In a real system, this would publish to a human-robot interaction interface
        print(f"Assistance Request: {message}")
```

## Performance Optimization

### Caching and Planning Efficiency

```python
from functools import lru_cache
import hashlib

class OptimizedTaskPlanner(AdaptiveTaskPlanner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.plan_cache = {}
        self.max_cache_size = 100

    @lru_cache(maxsize=50)
    def get_cached_plan(self, goal_hash: str) -> Optional[TaskPlan]:
        """Get plan from cache if available"""
        return self.plan_cache.get(goal_hash)

    def create_task_plan(self, goal: str) -> Optional[TaskPlan]:
        """Create task plan with caching"""
        # Create hash of the goal for caching
        goal_hash = hashlib.md5(goal.encode()).hexdigest()

        # Check cache first
        cached_plan = self.get_cached_plan(goal_hash)
        if cached_plan:
            self.get_logger().info('Using cached plan')
            return cached_plan

        # Create new plan
        new_plan = super().create_task_plan(goal)

        # Cache the plan if successful
        if new_plan:
            self.plan_cache[goal_hash] = new_plan

            # Maintain cache size
            if len(self.plan_cache) > self.max_cache_size:
                # Remove oldest entries (this is simplified)
                oldest_key = next(iter(self.plan_cache))
                del self.plan_cache[oldest_key]

        return new_plan

    def _get_system_prompt(self) -> str:
        """Optimized system prompt for faster planning"""
        return """
        You are a robotic task planner. Create efficient plans by:
        1. Using minimal but sufficient actions
        2. Considering the robot's current state
        3. Planning the most direct route to achieve goals
        4. Always returning valid JSON
        """

    def optimize_plan(self, plan: TaskPlan) -> TaskPlan:
        """Optimize an existing plan"""
        # Remove redundant actions
        optimized_subtasks = self._remove_redundant_actions(plan.subtasks)

        # Optimize action sequence
        optimized_subtasks = self._optimize_action_sequence(optimized_subtasks)

        # Create new plan with optimized subtasks
        optimized_plan = TaskPlan(
            task_id=f"opt_{plan.task_id}",
            original_goal=plan.original_goal,
            subtasks=optimized_subtasks,
            status=TaskStatus.PENDING
        )

        return optimized_plan

    def _remove_redundant_actions(self, subtasks: List[RobotAction]) -> List[RobotAction]:
        """Remove redundant or duplicate actions"""
        optimized = []
        for action in subtasks:
            # Check if this action is redundant
            if not self._is_redundant_action(action, optimized):
                optimized.append(action)

        return optimized

    def _is_redundant_action(self, action: RobotAction, existing_actions: List[RobotAction]) -> bool:
        """Check if an action is redundant given existing actions"""
        # Example: if we just navigated to a location, don't navigate there again immediately
        if (action.action_type in ['move_to', 'go_to', 'navigate_to'] and
            len(existing_actions) > 0 and
            existing_actions[-1].action_type in ['move_to', 'go_to', 'navigate_to'] and
            action.parameters.get('location') == existing_actions[-1].parameters.get('location')):
            return True

        return False
```

## Practical Exercise: Complete VLA Task Planning System

Create a complete task planning system with:

1. **LLM Integration**: OpenAI API integration for planning
2. **Context Awareness**: World state integration
3. **ROS Communication**: Message passing with ROS 2
4. **Error Handling**: Recovery from execution failures
5. **Optimization**: Plan caching and optimization
6. **Testing**: Validation with simulated robot

This exercise will provide hands-on experience with building a complete LLM-powered task planning system for robotics.

## Evaluation Metrics

### Plan Quality Assessment

```python
class PlanEvaluator:
    def __init__(self):
        self.metrics = {
            'success_rate': 0.0,
            'efficiency': 0.0,
            'adaptability': 0.0,
            'safety': 0.0
        }

    def evaluate_plan(self, plan: TaskPlan, execution_results: Dict[str, Any]) -> Dict[str, float]:
        """Evaluate plan quality based on execution results"""
        metrics = {}

        # Success rate: Did the plan achieve the goal?
        metrics['success_rate'] = 1.0 if plan.status == TaskStatus.COMPLETED else 0.0

        # Efficiency: Actions per goal complexity
        goal_complexity = len(plan.original_goal.split())  # Simplified
        action_efficiency = goal_complexity / max(len(plan.subtasks), 1)
        metrics['efficiency'] = min(action_efficiency, 1.0)

        # Adaptability: How well did it handle changes?
        metrics['adaptability'] = execution_results.get('adaptability_score', 0.5)

        # Safety: Avoided dangerous situations
        metrics['safety'] = execution_results.get('safety_score', 0.8)

        return metrics

    def get_average_metrics(self) -> Dict[str, float]:
        """Get average metrics across multiple executions"""
        # Implementation would track metrics over time
        return self.metrics
```

## Summary

LLM-powered task planning provides robots with:
- **Natural Language Interface**: Understanding complex, natural commands
- **Common-Sense Reasoning**: Access to world knowledge for planning
- **Adaptive Behavior**: Handling novel situations and changes
- **Hierarchical Planning**: Decomposition of complex tasks
- **Context Awareness**: Planning based on current world state
- **Error Recovery**: Handling and recovering from execution failures

This approach enables robots to perform complex tasks by understanding natural language goals and generating appropriate action sequences, significantly improving the flexibility and usability of robotic systems.