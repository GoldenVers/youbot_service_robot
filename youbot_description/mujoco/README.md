# YouBot MuJoCo Model

This folder contains the YouBot robot model in MuJoCo (MJCF) format.

## Files

- `youbot.xml` - Main robot model file (MJCF format)
- `view_mujoco.py` - Simple viewer script to visualize the robot
- `demo_arm.py` - Demo script showing arm movement
- `assets/` - Folder for mesh files (optional, currently using primitives)

## Installation

First, install MuJoCo Python bindings:

```bash
pip install mujoco
```

## Usage

### View the model:
```bash
cd ~/youbot/src/youbot_description/mujoco
python3 view_mujoco.py
```

### Run the arm demo:
```bash
cd ~/youbot/src/youbot_description/mujoco
python3 demo_arm.py
```

## Viewer Controls

- **Mouse drag**: Rotate camera
- **Scroll**: Zoom in/out
- **Double click**: Center on object
- **Space**: Pause/Resume simulation
- **Backspace**: Reset simulation
- **Esc**: Quit

## Model Details

### Joints
- `wheel_joint_fl/fr/bl/br` - Wheel joints (continuous)
- `arm_joint_1` to `arm_joint_5` - Arm joints (revolute)
- `gripper_finger_joint_l/r` - Gripper finger joints (prismatic)

### Actuators
- Wheel actuators: Velocity control
- Arm actuators: Position control
- Gripper actuators: Position control

### Sensors
- Joint position sensors for all arm joints
- Joint velocity sensors for all arm joints
- Gripper finger position sensors
- End-effector position and orientation sensors

## Using with DRL (Gymnasium)

For Deep Reinforcement Learning, you can create a Gymnasium environment:

```python
import gymnasium as gym
import mujoco
import numpy as np

class YouBotEnv(gym.Env):
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path('youbot.xml')
        self.data = mujoco.MjData(self.model)
        
        # Define observation and action spaces
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(self.model.nq + self.model.nv,), dtype=np.float64
        )
        self.action_space = gym.spaces.Box(
            low=-1, high=1, 
            shape=(self.model.nu,), dtype=np.float64
        )
    
    def step(self, action):
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)
        obs = self._get_obs()
        reward = self._compute_reward()
        done = False
        return obs, reward, done, False, {}
    
    def reset(self, seed=None):
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs(), {}
    
    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel])
    
    def _compute_reward(self):
        # Define your reward function here
        return 0.0
```

## ROS2 Integration

For ROS2 integration, consider using:
- `mujoco_ros2` package
- Custom node that publishes joint states and subscribes to commands
