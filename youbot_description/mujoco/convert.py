#!/usr/bin/env python3
"""
URDF to MuJoCo Converter for YouBot

This script converts the YouBot URDF to MuJoCo format by:
1. Replacing package:// URIs with absolute paths
2. Converting DAE mesh references to STL (MuJoCo doesn't support DAE)
3. Removing Gazebo-specific tags that MuJoCo doesn't understand
4. Adding MuJoCo-specific compiler directives

NOTE: MuJoCo only supports STL and OBJ mesh formats, not DAE/COLLADA.
      This script will try to use _convex.stl files for collisions,
      and will create simple primitive shapes for visuals where DAE files
      would have been used.
"""

import mujoco
import os
import re
import shutil

# Paths
PACKAGE_DIR = "/home/youssef/youbot/src/youbot_description"
URDF_PATH = os.path.join(PACKAGE_DIR, "urdf", "youbot.urdf")
MESHES_DIR = os.path.join(PACKAGE_DIR, "meshes")
MUJOCO_DIR = os.path.join(PACKAGE_DIR, "mujoco")
# Put temp URDF in urdf folder so relative paths work
TEMP_URDF = os.path.join(PACKAGE_DIR, "urdf", "youbot_mujoco.urdf")
OUTPUT_MJCF = os.path.join(MUJOCO_DIR, "youbot.xml")


def find_stl_alternative(dae_path):
    """Try to find an STL alternative for a DAE mesh file."""
    # First try direct replacement .dae -> .stl (now all should exist)
    stl_path = dae_path.replace('.dae', '.stl')
    if os.path.exists(stl_path):
        return stl_path
    
    # Try _convex.stl version
    stl_convex = dae_path.replace('.dae', '_convex.stl')
    if os.path.exists(stl_convex):
        return stl_convex
    
    return None


def convert_urdf_to_mujoco():
    """Convert URDF to MuJoCo-compatible format."""
    
    # Read the original URDF
    with open(URDF_PATH, 'r') as f:
        urdf_content = f.read()
    
    # 1. Replace package:// URIs with ABSOLUTE paths
    urdf_content = urdf_content.replace(
        'package://youbot_description/meshes/',
        MESHES_DIR + '/'
    )
    
    # 2. Convert DAE references to STL where possible
    # Find all mesh filenames
    mesh_pattern = r'<mesh\s+filename="([^"]+\.dae)"'
    
    def replace_dae_with_stl(match):
        dae_path = match.group(1)
        stl_path = find_stl_alternative(dae_path)
        if stl_path:
            print(f"  Replacing: {os.path.basename(dae_path)} -> {os.path.basename(stl_path)}")
            return f'<mesh filename="{stl_path}"'
        else:
            # No STL found, will need to use primitive shapes
            print(f"  WARNING: No STL found for: {os.path.basename(dae_path)}")
            return match.group(0)  # Keep original (will fail)
    
    print("\nConverting mesh references:")
    urdf_content = re.sub(mesh_pattern, replace_dae_with_stl, urdf_content)
    
    # 3. Remove all <gazebo> tags and their contents
    urdf_content = re.sub(r'<gazebo[^>]*>.*?</gazebo>', '', urdf_content, flags=re.DOTALL)
    
    # 4. Remove <sensor> tags inside links
    urdf_content = re.sub(r'<sensor[^>]*>.*?</sensor>', '', urdf_content, flags=re.DOTALL)
    
    # 5. Remove type="laser" from links
    urdf_content = re.sub(r'<link\s+name="([^"]+)"\s+type="[^"]*"', r'<link name="\1"', urdf_content)
    
    # 6. Remove <mimic> tags
    urdf_content = re.sub(r'<mimic[^/]*/>', '', urdf_content)
    
    # 7. Remove scale attribute from mesh tags
    urdf_content = re.sub(r'<mesh\s+filename="([^"]+)"\s+scale="[^"]*"', r'<mesh filename="\1"', urdf_content)
    
    # 8. Add MuJoCo compiler directive 
    mujoco_header = '''
  <mujoco>
    <compiler balanceinertia="true" discardvisual="false" strippath="false"/>
  </mujoco>
'''
    
    # Insert after <robot ...> tag
    urdf_content = re.sub(
        r'(<robot[^>]*>)',
        r'\1' + mujoco_header,
        urdf_content,
        count=1
    )
    
    # Write the modified URDF
    with open(TEMP_URDF, 'w') as f:
        f.write(urdf_content)
    
    print(f"\nCreated temporary URDF: {TEMP_URDF}")
    
    # Try to load with MuJoCo
    try:
        model = mujoco.MjModel.from_xml_path(TEMP_URDF)
        data = mujoco.MjData(model)
        print("✓ Successfully loaded URDF into MuJoCo!")
        print(f"  - Bodies: {model.nbody}")
        print(f"  - Joints: {model.njnt}")
        print(f"  - Actuators: {model.nu}")
        print(f"  - Geometries: {model.ngeom}")
        
        # Save as MJCF
        mujoco.mj_saveLastXML(OUTPUT_MJCF, model)
        print(f"✓ Saved MuJoCo model to: {OUTPUT_MJCF}")
        
        return model, data
        
    except Exception as e:
        print(f"✗ Failed to load URDF: {e}")
        print("\nDebug: Check the temp URDF file for issues:")
        print(f"  {TEMP_URDF}")
        return None, None


def view_model(model, data):
    """Launch the MuJoCo viewer to visualize the model."""
    if model is None:
        print("No model to view!")
        return
    
    try:
        import mujoco.viewer
        print("\nLaunching MuJoCo viewer...")
        print("Controls: Mouse drag=rotate, Scroll=zoom, Space=pause, Esc=quit")
        mujoco.viewer.launch(model, data)
    except Exception as e:
        print(f"Could not launch viewer: {e}")


if __name__ == "__main__":
    print("=" * 60)
    print("YouBot URDF to MuJoCo Converter")
    print("=" * 60)
    
    model, data = convert_urdf_to_mujoco()
    
    if model is not None:
        # Ask user if they want to view the model
        response = input("\nWould you like to view the model? [y/N]: ")
        if response.lower() == 'y':
            view_model(model, data)
    else:
        print("\n" + "=" * 60)
        print("CONVERSION FAILED")
        print("=" * 60)
        print("""
MuJoCo does NOT support DAE (COLLADA) mesh format.
You need to convert your meshes to STL or OBJ format.

Options:
1. Use Blender to batch convert DAE -> STL:
   - Open Blender
   - Import DAE file
   - Export as STL

2. Use online converter: https://www.meshconvert.com/

3. Use command line tool 'assimp':
   sudo apt install assimp-utils
   assimp export input.dae output.stl

After converting meshes, run this script again.
""")


def convert_urdf_to_mujoco():
    """Convert URDF to MuJoCo-compatible format."""
    
    # Read the original URDF
    with open(URDF_PATH, 'r') as f:
        urdf_content = f.read()
    
    # 1. Replace package:// URIs with ABSOLUTE paths (most reliable)
    urdf_content = urdf_content.replace(
        'package://youbot_description/meshes/',
        '/home/youssef/youbot/src/youbot_description/meshes/'
    )
    
    # 2. Remove all <gazebo> tags and their contents (MuJoCo doesn't understand them)
    urdf_content = re.sub(r'<gazebo[^>]*>.*?</gazebo>', '', urdf_content, flags=re.DOTALL)
    
    # 3. Remove <sensor> tags inside links (not valid URDF, Gazebo-specific)
    urdf_content = re.sub(r'<sensor[^>]*>.*?</sensor>', '', urdf_content, flags=re.DOTALL)
    
    # 4. Remove type="laser" from links (not valid URDF attribute)
    urdf_content = re.sub(r'<link\s+name="([^"]+)"\s+type="[^"]*"', r'<link name="\1"', urdf_content)
    
    # 5. Remove <mimic> tags (MuJoCo handles this differently)
    urdf_content = re.sub(r'<mimic[^/]*/>', '', urdf_content)
    
    # 6. Remove scale attribute from mesh tags (MuJoCo doesn't support it in URDF)
    urdf_content = re.sub(r'<mesh\s+filename="([^"]+)"\s+scale="[^"]*"', r'<mesh filename="\1"', urdf_content)
    
    # 7. Add MuJoCo compiler directive 
    # strippath="false" prevents MuJoCo from stripping the path from mesh filenames
    mujoco_header = '''
  <mujoco>
    <compiler balanceinertia="true" discardvisual="false" strippath="false"/>
  </mujoco>
'''
    
    # Insert after <robot ...> tag
    urdf_content = re.sub(
        r'(<robot[^>]*>)',
        r'\1' + mujoco_header,
        urdf_content,
        count=1
    )
    
    # Write the modified URDF to urdf folder (so relative paths work)
    with open(TEMP_URDF, 'w') as f:
        f.write(urdf_content)
    
    print(f"Created temporary URDF: {TEMP_URDF}")
    
    # Try to load with MuJoCo
    try:
        model = mujoco.MjModel.from_xml_path(TEMP_URDF)
        data = mujoco.MjData(model)
        print("✓ Successfully loaded URDF into MuJoCo!")
        print(f"  - Bodies: {model.nbody}")
        print(f"  - Joints: {model.njnt}")
        print(f"  - Actuators: {model.nu}")
        print(f"  - Geometries: {model.ngeom}")
        
        # Save as MJCF
        mujoco.mj_saveLastXML(OUTPUT_MJCF, model)
        print(f"✓ Saved MuJoCo model to: {OUTPUT_MJCF}")
        
        return model, data
        
    except Exception as e:
        print(f"✗ Failed to load URDF: {e}")
        print("\nDebug: Check the temp URDF file for issues:")
        print(f"  {TEMP_URDF}")
        return None, None


def view_model(model, data):
    """Launch the MuJoCo viewer to visualize the model."""
    if model is None:
        print("No model to view!")
        return
    
    try:
        import mujoco.viewer
        print("\nLaunching MuJoCo viewer...")
        print("Controls: Mouse drag=rotate, Scroll=zoom, Space=pause, Esc=quit")
        mujoco.viewer.launch(model, data)
    except Exception as e:
        print(f"Could not launch viewer: {e}")


if __name__ == "__main__":
    print("=" * 60)
    print("YouBot URDF to MuJoCo Converter")
    print("=" * 60)
    
    model, data = convert_urdf_to_mujoco()
    
    if model is not None:
        # Ask user if they want to view the model
        response = input("\nWould you like to view the model? [y/N]: ")
        if response.lower() == 'y':
            view_model(model, data)
