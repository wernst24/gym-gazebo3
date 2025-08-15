#!/usr/bin/env python3

import argparse
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO, SAC, TD3
import gym_gazebo3
from gym_gazebo3.wrappers import NormalizeObservationWrapper
import time


def main():
    parser = argparse.ArgumentParser(description='Test trained RL agent')
    parser.add_argument('--env-id', type=str, default='GazeboPioneer3AT-v0',
                       help='Environment ID')
    parser.add_argument('--model-path', type=str, required=True,
                       help='Path to trained model')
    parser.add_argument('--algorithm', type=str, default='PPO', choices=['PPO', 'SAC', 'TD3'],
                       help='RL algorithm used')
    parser.add_argument('--episodes', type=int, default=10,
                       help='Number of test episodes')
    parser.add_argument('--render', action='store_true',
                       help='Render environment')
    parser.add_argument('--normalize', action='store_true',
                       help='Normalize observations')
    parser.add_argument('--deterministic', action='store_true',
                       help='Use deterministic policy')
    
    args = parser.parse_args()
    
    # Create environment
    env_kwargs = {'headless': not args.render}
    env = gym.make(args.env_id, **env_kwargs)
    
    if args.normalize:
        env = NormalizeObservationWrapper(env)
    
    # Load trained model
    if args.algorithm == 'PPO':
        model = PPO.load(args.model_path)
    elif args.algorithm == 'SAC':
        model = SAC.load(args.model_path)
    elif args.algorithm == 'TD3':
        model = TD3.load(args.model_path)
    else:
        raise ValueError(f"Unknown algorithm: {args.algorithm}")
    
    # Test the agent
    episode_rewards = []
    episode_lengths = []
    success_count = 0
    
    for episode in range(args.episodes):
        obs, info = env.reset()
        episode_reward = 0
        episode_length = 0
        done = False
        
        print(f"Episode {episode + 1}/{args.episodes}")
        
        while not done:
            action, _ = model.predict(obs, deterministic=args.deterministic)
            obs, reward, terminated, truncated, info = env.step(action)
            
            episode_reward += reward
            episode_length += 1
            done = terminated or truncated
            
            if args.render:
                time.sleep(0.1)  # Slow down for visualization
            
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        
        # Check if goal was reached
        if 'goal_reached' in info and info['goal_reached']:
            success_count += 1
            
        print(f"  Reward: {episode_reward:.2f}, Length: {episode_length}")
        if 'distance_to_goal' in info:
            print(f"  Final distance to goal: {info['distance_to_goal']:.2f}")
        if 'collision' in info:
            print(f"  Collision: {info['collision']}")
    
    # Print statistics
    print("\n=== Test Results ===")
    print(f"Episodes: {args.episodes}")
    print(f"Average reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"Average length: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}")
    print(f"Success rate: {success_count}/{args.episodes} ({100*success_count/args.episodes:.1f}%)")
    print(f"Best reward: {np.max(episode_rewards):.2f}")
    print(f"Worst reward: {np.min(episode_rewards):.2f}")
    
    env.close()


if __name__ == '__main__':
    main()