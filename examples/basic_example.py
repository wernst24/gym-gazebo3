#!/usr/bin/env python3
"""
Basic example showing how to use gym-gazebo3 environments
"""

import gymnasium as gym
import numpy as np
import gym_gazebo3
from gym_gazebo3.wrappers import NormalizeObservationWrapper
import time


def random_agent_example():
    """Example using a random agent"""
    print("=== Random Agent Example ===")
    
    # Create environment
    env = gym.make('GazeboPioneer3AT-v0', headless=True)
    env = NormalizeObservationWrapper(env)
    
    # Run episodes
    for episode in range(3):
        obs, info = env.reset()
        episode_reward = 0
        step = 0
        done = False
        
        print(f"\nEpisode {episode + 1}")
        print(f"Initial observation shape: {obs.shape}")
        
        while not done and step < 100:
            # Random action
            action = env.action_space.sample()
            
            # Take step
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            step += 1
            done = terminated or truncated
            
            if step % 20 == 0:
                print(f"Step {step}: reward={reward:.3f}, total_reward={episode_reward:.3f}")
                if 'distance_to_goal' in info:
                    print(f"  Distance to goal: {info['distance_to_goal']:.2f}")
        
        print(f"Episode {episode + 1} finished:")
        print(f"  Total steps: {step}")
        print(f"  Total reward: {episode_reward:.3f}")
        print(f"  Goal reached: {info.get('goal_reached', False)}")
        print(f"  Collision: {info.get('collision', False)}")
    
    env.close()


def simple_policy_example():
    """Example using a simple wall-following policy"""
    print("\n=== Simple Policy Example ===")
    
    # Create environment
    env = gym.make('GazeboPioneer3AT-v0', headless=True)
    env = NormalizeObservationWrapper(env)
    
    def simple_policy(observation):
        """Simple wall-following policy"""
        # Get front and side distances
        front_distances = observation[25:35]  # Front sector
        left_distances = observation[45:55]   # Left sector
        right_distances = observation[5:15]   # Right sector
        
        front_min = np.min(front_distances)
        left_min = np.min(left_distances)
        right_min = np.min(right_distances)
        
        # Simple logic
        if front_min < 0.3:  # Obstacle in front
            if left_min > right_min:
                return np.array([0.0, 1.0])  # Turn left
            else:
                return np.array([0.0, -1.0])  # Turn right
        elif left_min < 0.5:  # Too close to left wall
            return np.array([0.3, -0.5])  # Move forward and turn right
        elif left_min > 1.0:  # Too far from left wall
            return np.array([0.3, 0.5])  # Move forward and turn left
        else:
            return np.array([0.5, 0.0])  # Move straight
    
    # Run episodes
    for episode in range(2):
        obs, info = env.reset()
        episode_reward = 0
        step = 0
        done = False
        
        print(f"\nEpisode {episode + 1}")
        
        while not done and step < 200:
            # Apply simple policy
            action = simple_policy(obs)
            
            # Take step
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            step += 1
            done = terminated or truncated
            
            if step % 50 == 0:
                print(f"Step {step}: reward={reward:.3f}, total_reward={episode_reward:.3f}")
        
        print(f"Episode {episode + 1} finished:")
        print(f"  Total steps: {step}")
        print(f"  Total reward: {episode_reward:.3f}")
        print(f"  Goal reached: {info.get('goal_reached', False)}")
        print(f"  Collision: {info.get('collision', False)}")
    
    env.close()


def environment_info_example():
    """Example showing environment information"""
    print("\n=== Environment Information ===")
    
    # Create environment
    env = gym.make('GazeboPioneer3AT-v0', headless=True)
    
    print(f"Environment: {env.spec.id}")
    print(f"Action space: {env.action_space}")
    print(f"Observation space: {env.observation_space}")
    print(f"Max episode steps: {env.spec.max_episode_steps}")
    
    # Reset and get initial observation
    obs, info = env.reset()
    print(f"Initial observation shape: {obs.shape}")
    print(f"Observation range: [{np.min(obs):.2f}, {np.max(obs):.2f}]")
    print(f"Sample action: {env.action_space.sample()}")
    
    env.close()


if __name__ == '__main__':
    print("gym-gazebo3 Basic Examples")
    print("=" * 40)
    
    try:
        environment_info_example()
        random_agent_example()
        simple_policy_example()
        
        print("\n=== Examples completed successfully! ===")
        
    except Exception as e:
        print(f"Error running examples: {e}")
        import traceback
        traceback.print_exc()