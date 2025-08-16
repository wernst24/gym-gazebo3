#!/usr/bin/env python3

"""
Test script for the simplified Gazebo environment that connects to external Gazebo.

This approach assumes:
1. Gazebo is already running externally (manual launch)
2. The environment just connects via ROS2 topics
3. No subprocess management of Gazebo

Usage:
    # First, manually start Gazebo in another terminal:
    source /opt/ros/kilted/setup.bash
    export GZ_SIM_RESOURCE_PATH="$HOME/.gazebo/models:$GZ_SIM_RESOURCE_PATH"
    ros2 launch launch/empty_gazebo_launch.py headless:=true
    
    # Then run this test:
    python3 examples/test_simple_env.py
"""

import gymnasium as gym
import gym_gazebo3
import time

def main():
    print("=== Testing Simplified Gazebo Environment ===")
    print("This assumes Gazebo is already running externally!")
    print()
    
    try:
        # Create the simplified environment
        print("Creating GazeboEmptySimple-v0 environment...")
        env = gym.make('GazeboEmptySimple-v0', headless=True)
        print(f"✓ Environment created successfully")
        print(f"Action space: {env.action_space}")
        print(f"Observation space: {env.observation_space}")
        print()
        
        # Test environment reset and steps
        print("Testing environment...")
        observation, info = env.reset()
        print(f"✓ Environment reset - observation shape: {observation.shape}")
        print(f"Info: {info}")
        print()
        
        # Run a few steps
        print("Running 5 test steps...")
        for step in range(5):
            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)
            print(f"Step {step+1}: action={action}, reward={reward:.3f}, "
                  f"terminated={terminated}, truncated={truncated}")
            print(f"  Info: {info}")
            
            if terminated or truncated:
                print("Episode ended, resetting...")
                observation, info = env.reset()
            
            time.sleep(0.5)
        
        print()
        print("✅ Simple environment test completed successfully!")
        print()
        print("Notes:")
        print("- Commands are being published to /cmd_vel")
        print("- Listening for data on /raw_image and /odom") 
        print("- If you see 'has_scan_data: False', check that Gazebo topics match")
        
    except Exception as e:
        print(f"❌ Error testing environment: {e}")
        print()
        print("Make sure:")
        print("1. Gazebo is running externally")
        print("2. ROS2 environment is sourced") 
        print("3. Workspace is built and sourced")
        
    finally:
        if 'env' in locals():
            env.close()
            print("Environment closed")

if __name__ == "__main__":
    main()