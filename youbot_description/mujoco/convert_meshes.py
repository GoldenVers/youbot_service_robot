#!/usr/bin/env python3
"""
Convert all DAE mesh files to STL for MuJoCo compatibility.

MuJoCo doesn't support DAE (COLLADA) format, only STL and OBJ.
This script converts all .dae files in the meshes folder to .stl
"""

import os
import sys

try:
    import trimesh
except ImportError:
    print("ERROR: trimesh not installed. Run: pip install trimesh")
    sys.exit(1)

# For COLLADA support
try:
    import collada
except ImportError:
    print("Installing pycollada for DAE support...")
    os.system("pip install pycollada")
    import collada

MESHES_DIR = "/home/youssef/youbot/src/youbot_description/meshes"


def convert_dae_to_stl(dae_path, stl_path):
    """Convert a single DAE file to STL."""
    try:
        # Load the mesh
        mesh = trimesh.load(dae_path, force='mesh')
        
        # Export as STL
        mesh.export(stl_path, file_type='stl')
        return True
    except Exception as e:
        print(f"  ERROR: {e}")
        return False


def main():
    print("=" * 60)
    print("DAE to STL Mesh Converter for MuJoCo")
    print("=" * 60)
    print(f"\nScanning: {MESHES_DIR}\n")
    
    # Find all DAE files
    dae_files = []
    for root, dirs, files in os.walk(MESHES_DIR):
        for file in files:
            if file.endswith('.dae'):
                dae_path = os.path.join(root, file)
                stl_path = dae_path.replace('.dae', '.stl')
                
                # Skip if STL already exists
                if os.path.exists(stl_path):
                    print(f"  SKIP (exists): {file}")
                    continue
                    
                dae_files.append((dae_path, stl_path))
    
    if not dae_files:
        print("No DAE files to convert (all already have STL versions)")
        return
    
    print(f"Found {len(dae_files)} DAE files to convert:\n")
    
    success = 0
    failed = 0
    
    for dae_path, stl_path in dae_files:
        rel_path = os.path.relpath(dae_path, MESHES_DIR)
        print(f"Converting: {rel_path}")
        
        if convert_dae_to_stl(dae_path, stl_path):
            print(f"  -> {os.path.basename(stl_path)} ✓")
            success += 1
        else:
            print(f"  -> FAILED ✗")
            failed += 1
    
    print("\n" + "=" * 60)
    print(f"Conversion complete: {success} succeeded, {failed} failed")
    print("=" * 60)
    
    if failed > 0:
        print("\nSome conversions failed. You may need to manually convert")
        print("those files using Blender or another 3D tool.")


if __name__ == "__main__":
    main()
