#!/usr/bin/env python3
"""
MuJoCo viewer script for YouBot robot.
This script loads and visualizes the YouBot model in MuJoCo.

Usage:
    python3 view_mujoco.py

Controls:
    - Mouse: Rotate/pan/zoom camera
    - Space: Pause/unpause simulation
    - Backspace: Reset simulation
    - Tab: Toggle visualization options
    - Right click: Context menu
"""

import mujoco
import mujoco.viewer
import numpy as np
import os

def main():
    # Get the path to the model file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, 'youbot.xml')
    
    print(f"Loading MuJoCo model from: {model_path}")
    
    # Load the model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("\n=== YouBot MuJoCo Model Info ===")
    print(f"Number of bodies: {model.nbody}")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of actuators: {model.nu}")
    print(f"Number of sensors: {model.nsensor}")
    print(f"Timestep: {model.opt.timestep} seconds")
    
    print("\n=== Joint Names ===")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"  Joint {i}: {joint_name}")
    
    print("\n=== Actuator Names ===")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  Actuator {i}: {actuator_name}")
    
    print("\n=== Starting MuJoCo Viewer ===")
    print("Controls:")
    print("  - Mouse drag: Rotate camera")
    print("  - Scroll: Zoom")
    print("  - Double click: Center on object")
    print("  - Space: Pause/Resume")
    print("  - Backspace: Reset")
    print("  - Esc: Quit")
    
    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set initial camera position
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.0
        viewer.cam.lookat[:] = [0, 0, 0.3]
        
        while viewer.is_running():
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()

if __name__ == "__main__":
    main()
