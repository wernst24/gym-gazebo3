import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from gymnasium import spaces
from gym_gazebo3.envs.gazebo_env import GazeboEnv

class GazeboEmptyEnvSimple(GazeboEnv):
    """
    Simplified empty world environment that connects to an existing Gazebo instance
    
    This version assumes Gazebo is already running externally and just connects to it
    via ROS2 topics, without managing the Gazebo process lifecycle.
    """

    def __init__(self, headless=True):
        # Don't call parent init as we're not managing Gazebo process
        self.headless = headless
        self.gazebo_process = None  # No process management
        self.current_step = 0
        self.max_episode_steps = 200
        
        # Action space: dummy discrete actions, mimicking straight, left, right
        self.action_space = spaces.Discrete(n=3)
        
        # Observation space: dummy Box, mimicking 32x32 depth camera
        self.observation_space = spaces.Box(
            low=0,
            high=255,
            shape=(32, 32, 1),
            dtype=np.uint8
        )
        
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Initialize ROS2 node
        self.node = Node('empty_env_simple')
        
        # Publishers and subscribers for ROS2 topics
        self.cmd_vel_pub = self.node.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        self.scan_sub = self.node.create_subscription(
            Image, '/raw_image', self._scan_callback, 10
        )
        
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        
        # Data storage
        self.last_scan = None
        self.last_odom = None
        
        print("GazeboEmptyEnvSimple initialized - expecting external Gazebo instance")

    def _scan_callback(self, data):
        """Callback for image/scan data"""
        self.last_scan = data

    def _odom_callback(self, data):
        """Callback for odometry data"""
        self.last_odom = data

    def reset(self, seed=None, options=None):
        """Reset the environment (no Gazebo restart)"""
        if seed is not None:
            super().reset(seed=seed)
        
        self.current_step = 0
        
        # Generate random observation since we're not using real Gazebo data yet
        observation = self.observation_space.sample()
        info = {
            'step': self.current_step, 
            'max_steps': self.max_episode_steps,
            'gazebo_external': True
        }
        return observation, info

    def step(self, action):
        """Execute action and return observation, reward, done, info"""
        self.current_step += 1
        
        # Publish action as velocity command (basic mapping)
        twist = Twist()
        if action == 0:  # straight
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        elif action == 1:  # left
            twist.linear.x = 0.2
            twist.angular.z = 0.5
        elif action == 2:  # right
            twist.linear.x = 0.2
            twist.angular.z = -0.5
        
        self.cmd_vel_pub.publish(twist)
        
        # Spin once to process callbacks
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Generate dummy observation and reward
        observation = self.observation_space.sample()
        reward = np.random.uniform(-1.0, 1.0)
        terminated = False
        truncated = self.current_step >= self.max_episode_steps
        info = {
            'step': self.current_step,
            'action_taken': action,
            'has_scan_data': self.last_scan is not None,
            'has_odom_data': self.last_odom is not None
        }
        
        return observation, reward, terminated, truncated, info

    def close(self):
        """Clean up resources (no Gazebo shutdown)"""
        print("Closing GazeboEmptyEnvSimple")
        if self.node:
            self.node.destroy_node()
        # Note: We don't shutdown rclpy as other nodes might be using it