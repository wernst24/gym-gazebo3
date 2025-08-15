import os
import subprocess
import time
import signal
import psutil
import rclpy
from rclpy.node import Node
import gymnasium as gym
from gymnasium import spaces
import numpy as np


class GazeboEnv(gym.Env):
    """
    Base class for Gazebo environments using ROS2 Humble
    """
    
    def __init__(self, launch_file, world_file=None, headless=True):
        super(GazeboEnv, self).__init__()
        
        self.launch_file = launch_file
        self.world_file = world_file
        self.headless = headless
        self.gazebo_process = None
        self.ros_master_process = None
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        # Default action and observation spaces (to be overridden)
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(
            low=0, high=255, shape=(64, 64, 3), dtype=np.uint8
        )
        
    def _launch_gazebo(self):
        """Launch Gazebo with the specified world file"""
        try:
            env = os.environ.copy()
            if self.headless:
                env['DISPLAY'] = ':1'
            
            launch_cmd = ['ros2', 'launch', self.launch_file]
            if self.world_file:
                launch_cmd.extend(['world:=' + self.world_file])
            
            self.gazebo_process = subprocess.Popen(
                launch_cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Wait for Gazebo to start
            time.sleep(5)
            
        except Exception as e:
            print(f"Error launching Gazebo: {e}")
            
    def _kill_gazebo(self):
        """Terminate Gazebo and related processes"""
        try:
            # Kill gazebo processes
            for proc in psutil.process_iter(['pid', 'name']):
                if 'gazebo' in proc.info['name'].lower():
                    try:
                        proc.kill()
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
            
            if self.gazebo_process:
                self.gazebo_process.terminate()
                self.gazebo_process.wait(timeout=5)
                
        except Exception as e:
            print(f"Error killing Gazebo: {e}")
            
    def reset(self, seed=None, options=None):
        """Reset the environment"""
        if seed is not None:
            np.random.seed(seed)
            
        self._kill_gazebo()
        time.sleep(2)
        self._launch_gazebo()
        
        # Return initial observation and info
        obs = self._get_observation()
        info = {}
        return obs, info
        
    def step(self, action):
        """Execute action and return observation, reward, done, info"""
        # To be implemented by subclasses
        raise NotImplementedError
        
    def _get_observation(self):
        """Get current observation from sensors"""
        # To be implemented by subclasses
        raise NotImplementedError
        
    def _compute_reward(self):
        """Compute reward for current state"""
        # To be implemented by subclasses
        raise NotImplementedError
        
    def render(self, mode='human'):
        """Render the environment"""
        pass
        
    def close(self):
        """Clean up resources"""
        self._kill_gazebo()
        if rclpy.ok():
            rclpy.shutdown()