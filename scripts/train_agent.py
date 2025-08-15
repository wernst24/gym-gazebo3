#!/usr/bin/env python3

import argparse
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnNoModelImprovement
from stable_baselines3.common.monitor import Monitor
import gym_gazebo3
from gym_gazebo3.wrappers import NormalizeObservationWrapper
import os


def main():
    parser = argparse.ArgumentParser(description='Train RL agent for gym-gazebo3 environments')
    parser.add_argument('--env-id', type=str, default='GazeboPioneer3AT-v0',
                       help='Environment ID')
    parser.add_argument('--algorithm', type=str, default='PPO', choices=['PPO', 'SAC', 'TD3'],
                       help='RL algorithm to use')
    parser.add_argument('--total-timesteps', type=int, default=100000,
                       help='Total training timesteps')
    parser.add_argument('--save-path', type=str, default='./models/',
                       help='Path to save trained models')
    parser.add_argument('--tensorboard-log', type=str, default='./logs/',
                       help='Tensorboard log directory')
    parser.add_argument('--eval-freq', type=int, default=10000,
                       help='Evaluation frequency')
    parser.add_argument('--headless', action='store_true',
                       help='Run in headless mode')
    parser.add_argument('--normalize', action='store_true',
                       help='Normalize observations')
    
    args = parser.parse_args()
    
    # Create directories
    os.makedirs(args.save_path, exist_ok=True)
    os.makedirs(args.tensorboard_log, exist_ok=True)
    
    # Create environment
    env_kwargs = {'headless': args.headless}
    
    def make_env():
        env = gym.make(args.env_id, **env_kwargs)
        if args.normalize:
            env = NormalizeObservationWrapper(env)
        env = Monitor(env)
        return env
    
    # Create training environment
    train_env = make_vec_env(make_env, n_envs=1)
    
    # Create evaluation environment
    eval_env = make_env()
    
    # Initialize algorithm
    if args.algorithm == 'PPO':
        model = PPO(
            'MlpPolicy',
            train_env,
            verbose=1,
            tensorboard_log=args.tensorboard_log,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
        )
    elif args.algorithm == 'SAC':
        model = SAC(
            'MlpPolicy',
            train_env,
            verbose=1,
            tensorboard_log=args.tensorboard_log,
            learning_rate=3e-4,
            buffer_size=100000,
            learning_starts=1000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
        )
    elif args.algorithm == 'TD3':
        model = TD3(
            'MlpPolicy',
            train_env,
            verbose=1,
            tensorboard_log=args.tensorboard_log,
            learning_rate=3e-4,
            buffer_size=100000,
            learning_starts=1000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
        )
    
    # Setup callbacks
    stop_callback = StopTrainingOnNoModelImprovement(
        max_no_improvement_evals=5,
        min_evals=10,
        verbose=1
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=args.save_path,
        log_path=args.save_path,
        eval_freq=args.eval_freq,
        deterministic=True,
        render=False,
        callback_after_eval=stop_callback,
        verbose=1
    )
    
    # Train the agent
    print(f"Training {args.algorithm} on {args.env_id} for {args.total_timesteps} timesteps")
    model.learn(
        total_timesteps=args.total_timesteps,
        callback=eval_callback,
        progress_bar=True
    )
    
    # Save final model
    final_model_path = os.path.join(args.save_path, f'{args.algorithm}_{args.env_id}_final')
    model.save(final_model_path)
    print(f"Final model saved to {final_model_path}")
    
    # Close environments
    train_env.close()
    eval_env.close()


if __name__ == '__main__':
    main()