# gym-gazebo3

A reinforcement learning toolkit for ROS2 Humble and Gazebo Classic 11, providing gymnasium-compatible environments for robotic simulation and training.

## Overview

gym-gazebo3 is the successor to the original gym-gazebo & gym-gazebo2 packages, updated to work with:
- **ROS2 Humble** instead of ROS1
- **Gymnasium** instead of OpenAI Gym
- **Gazebo Classic 11** for robust simulation

## Features

- 🤖 Multiple robot environments (TurtleBot3, Pioneer 3AT)
- 🎯 Gymnasium-compatible interface
- 🚀 ROS2 Humble integration
- 🎮 Configurable training parameters
- 📈 Tensorboard logging
- 🔧 Customizable observation/action spaces
- 🎭 Environment wrappers for preprocessing

## Installation

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Classic 11
- Python 3.8+

### Install Dependencies

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install Gazebo Classic
sudo apt install gazebo11 ros-humble-gazebo-ros-pkgs

# Install Python dependencies
pip install -e .
```

### Build the Package

```bash
# In your ROS2 workspace
colcon build --packages-select gym_gazebo3
source install/setup.bash
```

## Quick Start

### Basic Usage

```python
import gymnasium as gym
import gym_gazebo3

# Create environment
env = gym.make('GazeboPioneer3AT-v0', headless=True)

# Reset environment
obs, info = env.reset()

# Take random actions
for _ in range(100):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    
    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

### Training an Agent

```bash
# Train PPO agent on Pioneer 3AT environment
python scripts/train_agent.py \
    --env-id GazeboPioneer3AT-v0 \
    --algorithm PPO \
    --total-timesteps 100000 \
    --headless
```

### Testing a Trained Agent

```bash
# Test trained agent
python scripts/test_agent.py \
    --env-id GazeboPioneer3AT-v0 \
    --model-path ./models/best_model \
    --algorithm PPO \
    --episodes 10
```

### Visualizing Training

```bash
# Start Tensorboard
python scripts/visualize_training.py --logdir ./logs/ --open-browser
```

## Available Environments

### GazeboPioneer3AT-v0
- **Robot**: Pioneer 3AT with 180° LIDAR (61 readings)
- **Action Space**: Continuous [linear_velocity, angular_velocity]
- **Observation Space**: LIDAR readings [0, 30] meters
- **Goal**: Navigate to target position while avoiding obstacles

### GazeboTurtlebot3-v0
- **Robot**: TurtleBot3 Burger with 360° LIDAR
- **Action Space**: Continuous [linear_velocity, angular_velocity]  
- **Observation Space**: LIDAR readings [0, 10] meters
- **Goal**: Navigate to target position while avoiding obstacles

## Project Structure

```
gym-gazebo3/
├── gym_gazebo3/           # Main package
│   ├── envs/             # Environment implementations
│   ├── utils/            # Utility functions
│   └── wrappers/         # Environment wrappers
├── launch/               # ROS2 launch files
├── worlds/               # Gazebo world files
├── models/               # Robot URDF models
├── config/               # Configuration files
├── scripts/              # Training/testing scripts
└── examples/             # Example usage
```

## Configuration

Training parameters can be configured via YAML files:

```yaml
# config/training_config.yaml
environment:
  env_id: "GazeboPioneer3AT-v0"
  headless: true
  normalize_observations: true

training:
  algorithm: "PPO"
  total_timesteps: 100000
  
algorithms:
  PPO:
    learning_rate: 3e-4
    n_steps: 2048
    batch_size: 64
```

## Environment Wrappers

### NormalizeObservationWrapper
Normalizes LIDAR observations to [0, 1] range:

```python
from gym_gazebo3.wrappers import NormalizeObservationWrapper

env = gym.make('GazeboPioneer3AT-v0')
env = NormalizeObservationWrapper(env, min_range=0.0, max_range=30.0)
```

### FrameStackWrapper
Stacks multiple consecutive observations:

```python
from gym_gazebo3.wrappers import FrameStackWrapper

env = gym.make('GazeboPioneer3AT-v0')
env = FrameStackWrapper(env, num_stack=4)
```

## Examples

See the `examples/` directory for complete usage examples:

- `basic_example.py`: Basic environment usage
- More examples coming soon...

## Acknowledgments

- Original [gym-gazebo](https://github.com/erlerobot/gym-gazebo) project
- [uq-in-slam](https://github.com/wernst24/uq-in-slam) implementation reference
- ROS2 and Gazebo communities
- Stable Baselines3 team

## Citation

If you use gym-gazebo3 in your research, please cite:

```bibtex
@software{gym_gazebo3,
  title={gym-gazebo3: Reinforcement Learning Toolkit for ROS2 and Gazebo},
  author={Liam Ernst},
  year={2025},
  url={https://github.com/wernst24/gym-gazebo3}
}
```