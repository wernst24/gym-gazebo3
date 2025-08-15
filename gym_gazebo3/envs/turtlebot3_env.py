import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gymnasium import spaces
from gym_gazebo3.envs.gazebo_env import GazeboEnv


class GazeboTurtlebot3Env(GazeboEnv):
    """
    Turtlebot3 navigation environment using ROS2 Humble and Gazebo Classic 11
    """
    
    def __init__(self, world_file="turtlebot3_world.world", headless=True):
        launch_file = "turtlebot3_gazebo_launch.py"
        super(GazeboTurtlebot3Env, self).__init__(launch_file, world_file, headless)
        
        # Action space: [linear_x, angular_z]
        self.action_space = spaces.Box(
            low=np.array([-0.5, -1.5]),
            high=np.array([0.5, 1.5]),
            dtype=np.float32
        )
        
        # Observation space: LIDAR readings
        self.observation_space = spaces.Box(
            low=0.0,
            high=10.0,
            shape=(360,),
            dtype=np.float32
        )
        
        # Initialize ROS2 node
        self.node = Node('turtlebot3_env')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        self.scan_sub = self.node.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )
        
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        
        # State variables
        self.scan_data = None
        self.odom_data = None
        self.goal_position = np.array([5.0, 5.0])
        
    def _scan_callback(self, msg):
        """Callback for LIDAR scan data"""
        self.scan_data = np.array(msg.ranges)
        # Replace inf values with max range
        self.scan_data = np.where(
            np.isinf(self.scan_data), 
            msg.range_max, 
            self.scan_data
        )
        
    def _odom_callback(self, msg):
        """Callback for odometry data"""
        self.odom_data = msg
        
    def _get_observation(self):
        """Get current LIDAR observation"""
        if self.scan_data is not None:
            return self.scan_data.astype(np.float32)
        else:
            return np.zeros(360, dtype=np.float32)
            
    def _compute_reward(self):
        """Compute reward based on distance to goal and obstacle avoidance"""
        reward = 0.0
        
        if self.odom_data is None or self.scan_data is None:
            return reward
            
        # Distance to goal reward
        current_pos = np.array([
            self.odom_data.pose.pose.position.x,
            self.odom_data.pose.pose.position.y
        ])
        
        distance_to_goal = np.linalg.norm(current_pos - self.goal_position)
        reward += -distance_to_goal * 0.1
        
        # Collision penalty
        min_distance = np.min(self.scan_data)
        if min_distance < 0.2:
            reward -= 10.0
            
        # Goal reached reward
        if distance_to_goal < 0.5:
            reward += 100.0
            
        return reward
        
    def step(self, action):
        """Execute action and return observation, reward, done, info"""
        # Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = float(action[0])
        cmd_msg.angular.z = float(action[1])
        self.cmd_vel_pub.publish(cmd_msg)
        
        # Spin ROS2 node to process callbacks
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Get observation
        obs = self._get_observation()
        
        # Compute reward
        reward = self._compute_reward()
        
        # Check if episode is done
        done = False
        if self.odom_data is not None:
            current_pos = np.array([
                self.odom_data.pose.pose.position.x,
                self.odom_data.pose.pose.position.y
            ])
            distance_to_goal = np.linalg.norm(current_pos - self.goal_position)
            
            # Episode done if goal reached or collision
            if distance_to_goal < 0.5 or np.min(self.scan_data) < 0.2:
                done = True
                
        info = {}
        
        return obs, reward, done, False, info
        
    def reset(self, seed=None, options=None):
        """Reset the environment"""
        obs, info = super().reset(seed, options)
        
        # Reset state variables
        self.scan_data = None
        self.odom_data = None
        
        # Randomize goal position if specified
        if options and 'randomize_goal' in options:
            self.goal_position = np.random.uniform(-5, 5, size=2)
            
        # Wait for initial data
        timeout = time.time() + 10
        while (self.scan_data is None or self.odom_data is None) and time.time() < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        return self._get_observation(), info