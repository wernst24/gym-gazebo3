"""
gym-gazebo3: A reinforcement learning toolkit for ROS2 Humble and Gazebo Classic 11
"""

import gymnasium as gym
from gymnasium.envs.registration import register

from gym_gazebo3.envs.gazebo_env import GazeboEnv

__version__ = "0.1.0"

# Register environments
register(
    id='GazeboTurtlebot3-v0',
    entry_point='gym_gazebo3.envs:GazeboTurtlebot3Env',
    max_episode_steps=1000,
)

register(
    id='GazeboPioneer3AT-v0',
    entry_point='gym_gazebo3.envs:GazeboPioneer3ATEnv',
    max_episode_steps=1000,
)

# Simplified environment that connects to external Gazebo
register(
    id='GazeboEmptySimple-v0',
    entry_point='gym_gazebo3.envs:GazeboEmptyEnvSimple',
    max_episode_steps=200,
)

# TurtleBot3 with depth camera (matching target repository)
register(
    id='GazeboTurtlebot3Camera-v0',
    entry_point='gym_gazebo3.envs:GazeboTurtlebot3CameraEnv',
    max_episode_steps=1000,
)