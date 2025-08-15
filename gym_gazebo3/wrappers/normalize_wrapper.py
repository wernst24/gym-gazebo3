import numpy as np
import gymnasium as gym
from gymnasium.spaces import Box


class NormalizeObservationWrapper(gym.ObservationWrapper):
    """
    Wrapper to normalize LIDAR observations to [0, 1] range
    """
    
    def __init__(self, env, min_range=0.0, max_range=30.0):
        super().__init__(env)
        
        self.min_range = min_range
        self.max_range = max_range
        
        # Update observation space to reflect normalization
        if isinstance(env.observation_space, Box):
            self.observation_space = Box(
                low=0.0,
                high=1.0,
                shape=env.observation_space.shape,
                dtype=np.float32
            )
        else:
            raise ValueError("NormalizeObservationWrapper only supports Box observation spaces")
            
    def observation(self, observation):
        """Normalize observation to [0, 1] range"""
        # Clip values to expected range
        clipped_obs = np.clip(observation, self.min_range, self.max_range)
        
        # Normalize to [0, 1]
        normalized_obs = (clipped_obs - self.min_range) / (self.max_range - self.min_range)
        
        return normalized_obs.astype(np.float32)