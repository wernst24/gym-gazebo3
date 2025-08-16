import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from gymnasium import spaces
from gym_gazebo3.envs.gazebo_env import GazeboEnv

# cv_bridge will be imported lazily to avoid NumPy compatibility issues
CV_BRIDGE_AVAILABLE = None  # Will be set on first use


class GazeboTurtlebot3CameraEnv(GazeboEnv):
    """
    TurtleBot3 environment with depth camera - matching the target repository.
    
    Action Space: Discrete(3) - Forward, Left turn, Right turn
    Observation Space: 32x32 depth camera image (single channel)
    
    Designed to match: github.com/dimahdera/Robust-Uncertainty-Estimation-Framework-in-Deep-Reinforcement-Learning-for-Active-SLAM
    """

    def __init__(self, headless=True):
        # Don't call parent init for simplified approach
        self.headless = headless
        self.gazebo_process = None
        self.current_step = 0
        self.max_episode_steps = 1000
        
        # Action space: Discrete actions matching target repo
        # 0: Forward, 1: Left turn, 2: Right turn
        self.action_space = spaces.Discrete(3)
        
        # Observation space: 32x32 depth image (single channel, normalized 0-255)
        self.observation_space = spaces.Box(
            low=0,
            high=255,
            shape=(32, 32, 1),
            dtype=np.uint8
        )
        
        # Action mappings (matching target repository)
        self.action_mappings = {
            0: {'linear': 0.3, 'angular': 0.0},     # Forward
            1: {'linear': 0.1, 'angular': 0.3},     # Left turn  
            2: {'linear': 0.1, 'angular': -0.3}     # Right turn
        }
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        # Initialize ROS2 node
        self.node = Node('turtlebot3_camera_env')
        
        # CV Bridge will be initialized lazily
        self.bridge = None
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # Depth camera subscriber (matching target topic)
        self.depth_sub = self.node.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_callback, 10
        )
        
        # Laser scan for collision detection
        self.laser_sub = self.node.create_subscription(
            LaserScan, '/scan', self._laser_callback, 10
        )
        
        # Data storage
        self.last_depth_image = None
        self.last_laser_scan = None
        self.collision_threshold = 0.21  # meters (matching target repo)
        
        print("GazeboTurtlebot3CameraEnv initialized - TurtleBot3 with depth camera")
        print("Action space: 3 discrete actions (Forward, Left, Right)")
        print("Observation space: 32x32 depth images")

    def _depth_callback(self, data):
        """Callback for depth camera data"""
        global CV_BRIDGE_AVAILABLE
        
        try:
            # Lazy import of cv_bridge
            if CV_BRIDGE_AVAILABLE is None:
                try:
                    from cv_bridge import CvBridge
                    import cv2
                    self.bridge = CvBridge()
                    CV_BRIDGE_AVAILABLE = True
                    self._cv2 = cv2  # Store cv2 module
                except ImportError as e:
                    print(f"Warning: cv_bridge not available ({e}). Using fallback image processing.")
                    CV_BRIDGE_AVAILABLE = False
                    self.bridge = None
            
            if CV_BRIDGE_AVAILABLE and self.bridge:
                # Convert ROS image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
                
                # Handle NaN values and normalize
                cv_image = np.nan_to_num(cv_image, nan=0.0, posinf=0.0, neginf=0.0)
                
                # Resize to 32x32 (matching target repo)
                resized = self._cv2.resize(cv_image, (32, 32))
                
                # Normalize to 0-255 range
                normalized = self._cv2.normalize(resized, None, 0, 255, self._cv2.NORM_MINMAX)
                
                # Convert to uint8 and add channel dimension
                self.last_depth_image = normalized.astype(np.uint8).reshape(32, 32, 1)
            else:
                # Fallback: Create dummy depth image for testing
                # Generate a pattern that simulates depth data
                pattern = np.random.randint(0, 255, (32, 32, 1), dtype=np.uint8)
                self.last_depth_image = pattern
                
        except Exception as e:
            print(f"Error processing depth image: {e}")
            # Provide fallback image
            self.last_depth_image = np.zeros((32, 32, 1), dtype=np.uint8)

    def _laser_callback(self, data):
        """Callback for laser scan data (collision detection)"""
        self.last_laser_scan = data

    def _is_collision(self):
        """Check for collision using laser scan data"""
        if self.last_laser_scan is None:
            return False
        
        # Check minimum distance in laser scan
        valid_ranges = [r for r in self.last_laser_scan.ranges 
                       if not np.isnan(r) and not np.isinf(r)]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            return min_distance < self.collision_threshold
        
        return False

    def _get_observation(self):
        """Get current observation (depth image)"""
        if self.last_depth_image is not None:
            return self.last_depth_image.copy()
        else:
            # Return zero image if no depth data available
            return np.zeros((32, 32, 1), dtype=np.uint8)

    def _calculate_reward(self, action):
        """Calculate reward based on action and current state"""
        # Reward structure matching target repository
        if action == 0:  # Forward
            reward = 5.0  # Positive reward for forward motion
        else:  # Left or Right turn
            reward = -1.0  # Small negative reward for turning
        
        # Additional collision penalty
        if self._is_collision():
            reward -= 100.0
        
        return reward

    def reset(self, seed=None, options=None):
        """Reset the environment"""
        if seed is not None:
            super().reset(seed=seed)
        
        self.current_step = 0
        
        # Stop the robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        # Spin once to process any pending callbacks
        rclpy.spin_once(self.node, timeout_sec=0.5)
        
        observation = self._get_observation()
        info = {
            'step': self.current_step,
            'max_steps': self.max_episode_steps,
            'has_depth_data': self.last_depth_image is not None,
            'has_laser_data': self.last_laser_scan is not None
        }
        
        return observation, info

    def step(self, action):
        """Execute action and return observation, reward, done, info"""
        self.current_step += 1
        
        # Validate action
        if not self.action_space.contains(action):
            raise ValueError(f"Invalid action {action}")
        
        # Execute action
        twist = Twist()
        action_params = self.action_mappings[action]
        twist.linear.x = action_params['linear']
        twist.angular.z = action_params['angular']
        
        self.cmd_vel_pub.publish(twist)
        
        # Allow time for action to take effect and sensors to update
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Get new observation
        observation = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward(action)
        
        # Check termination conditions
        collision = self._is_collision()
        terminated = collision  # Episode ends on collision
        truncated = self.current_step >= self.max_episode_steps
        
        info = {
            'step': self.current_step,
            'action_taken': action,
            'collision': collision,
            'has_depth_data': self.last_depth_image is not None,
            'has_laser_data': self.last_laser_scan is not None,
            'reward_breakdown': {
                'action_reward': self._calculate_reward(action) + (100.0 if collision else 0.0),
                'collision_penalty': -100.0 if collision else 0.0
            }
        }
        
        return observation, reward, terminated, truncated, info

    def close(self):
        """Clean up resources"""
        print("Closing GazeboTurtlebot3CameraEnv")
        
        # Stop the robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        if self.node:
            self.node.destroy_node()