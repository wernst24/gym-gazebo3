import numpy as np
import gymnasium as gym
from gymnasium.spaces import Box
from collections import deque


class FrameStackWrapper(gym.Wrapper):
    """
    Wrapper to stack multiple consecutive observations
    Useful for providing temporal information to the agent
    """
    
    def __init__(self, env, num_stack=4):
        super().__init__(env)
        
        self.num_stack = num_stack
        self.frames = deque(maxlen=num_stack)
        
        # Update observation space
        obs_shape = env.observation_space.shape
        if len(obs_shape) == 1:
            # For 1D observations (like LIDAR), stack along new dimension
            new_shape = (num_stack, obs_shape[0])
        else:
            # For multi-dimensional observations, stack along first dimension
            new_shape = (num_stack,) + obs_shape
            
        self.observation_space = Box(
            low=env.observation_space.low.min(),
            high=env.observation_space.high.max(),
            shape=new_shape,
            dtype=env.observation_space.dtype
        )
        
    def reset(self, **kwargs):
        """Reset environment and initialize frame stack"""
        observation, info = self.env.reset(**kwargs)
        
        # Fill frame stack with initial observation
        for _ in range(self.num_stack):
            self.frames.append(observation)
            
        return self._get_stacked_observation(), info
        
    def step(self, action):
        """Take step and update frame stack"""
        observation, reward, terminated, truncated, info = self.env.step(action)
        
        self.frames.append(observation)
        
        return self._get_stacked_observation(), reward, terminated, truncated, info
        
    def _get_stacked_observation(self):
        """Get stacked observation from frame buffer"""
        return np.array(self.frames, dtype=self.observation_space.dtype)