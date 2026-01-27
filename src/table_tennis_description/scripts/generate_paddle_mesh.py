#!/usr/bin/env python3
"""
Generate a realistic table tennis paddle STL mesh.
ITTF regulations: Blade must be at least 85% wood, 
typically 15.7cm × 15.2cm × 6.5mm (with rubber sheets).
"""

import numpy as np
from stl import mesh

def create_paddle_mesh():
    """Create a realistic table tennis paddle mesh."""
    
    # Paddle dimensions (meters)
    blade_width = 0.157  # 15.7cm
    blade_height = 0.152  # 15.2cm
    blade_thickness = 0.0065  # 6.5mm (includes rubber)
    handle_length = 0.10  # 10cm
    handle_width = 0.025  # 2.5cm at base
    handle_tip_width = 0.022  # 2.2cm at tip
    handle_thickness = 0.020  # 2cm
    
    vertices = []
    faces = []
    
    # Blade (rounded rectangular shape)
    blade_corner_radius = 0.01
    blade_segments = 16
    
    # Create blade vertices (top and bottom surfaces)
    # Top surface (z = blade_thickness/2)
    z_top = blade_thickness / 2
    z_bottom = -blade_thickness / 2
    
    # Main blade rectangle with rounded corners
    hw = blade_width / 2
    hh = blade_height / 2
    
    # Blade outline points (creating rounded rectangle)
    blade_points_2d = []
    
    # Top edge
    for i in range(blade_segments // 4):
        angle = np.pi/2 + i * (np.pi/2) / (blade_segments // 4)
        x = (hw - blade_corner_radius) + blade_corner_radius * np.cos(angle)
        y = (hh - blade_corner_radius) + blade_corner_radius * np.sin(angle)
        blade_points_2d.append([x, y])
    
    # Right edge
    for i in range(blade_segments // 4):
        angle = 0 + i * (np.pi/2) / (blade_segments // 4)
        x = (hw - blade_corner_radius) + blade_corner_radius * np.cos(angle)
        y = -(hh - blade_corner_radius) + blade_corner_radius * np.sin(angle)
        blade_points_2d.append([x, y])
    
    # Bottom edge (where handle connects)
    for i in range(blade_segments // 4):
        angle = -np.pi/2 + i * (np.pi/2) / (blade_segments // 4)
        x = -(hw - blade_corner_radius) + blade_corner_radius * np.cos(angle)
        y = -(hh - blade_corner_radius) + blade_corner_radius * np.sin(angle)
        blade_points_2d.append([x, y])
    
    # Left edge
    for i in range(blade_segments // 4):
        angle = np.pi + i * (np.pi/2) / (blade_segments // 4)
        x = -(hw - blade_corner_radius) + blade_corner_radius * np.cos(angle)
        y = (hh - blade_corner_radius) + blade_corner_radius * np.sin(angle)
        blade_points_2d.append([x, y])
    
    blade_points_2d = np.array(blade_points_2d)
    
    # Create top and bottom blade surfaces
    num_blade_points = len(blade_points_2d)
    
    # Add vertices for blade
    for point in blade_points_2d:
        vertices.append([point[0], point[1], z_top])  # Top surface
    
    for point in blade_points_2d:
        vertices.append([point[0], point[1], z_bottom])  # Bottom surface
    
    # Create blade faces (top surface)
    for i in range(num_blade_points - 2):
        faces.append([0, i + 1, i + 2])
    
    # Create blade faces (bottom surface - reversed winding)
    offset = num_blade_points
    for i in range(num_blade_points - 2):
        faces.append([offset, offset + i + 2, offset + i + 1])
    
    # Create side faces connecting top and bottom
    for i in range(num_blade_points):
        next_i = (i + 1) % num_blade_points
        # Two triangles per side face
        faces.append([i, next_i, offset + i])
        faces.append([next_i, offset + next_i, offset + i])
    
    # Handle
    handle_start_y = -hh + 0.005  # Start slightly into blade
    handle_end_y = handle_start_y - handle_length
    
    # Handle cross-section points (tapered rectangular)
    handle_segments_y = 20
    
    for seg in range(handle_segments_y + 1):
        t = seg / handle_segments_y
        y = handle_start_y - t * handle_length
        
        # Taper handle width
        w = handle_width + t * (handle_tip_width - handle_width)
        h = handle_thickness * (1.0 - 0.2 * t)  # Slight thickness taper
        
        # Four corners of handle cross-section
        handle_base_idx = len(vertices)
        vertices.append([-w/2, y, h/2])
        vertices.append([w/2, y, h/2])
        vertices.append([w/2, y, -h/2])
        vertices.append([-w/2, y, -h/2])
        
        # Connect to previous segment
        if seg > 0:
            prev_base = handle_base_idx - 4
            curr_base = handle_base_idx
            
            # Four side faces
            for i in range(4):
                next_i = (i + 1) % 4
                faces.append([prev_base + i, curr_base + i, prev_base + next_i])
                faces.append([curr_base + i, curr_base + next_i, prev_base + next_i])
    
    # Cap handle end
    end_cap_base = len(vertices) - 4
    faces.append([end_cap_base, end_cap_base + 1, end_cap_base + 2])
    faces.append([end_cap_base, end_cap_base + 2, end_cap_base + 3])
    
    # Convert to numpy arrays
    vertices = np.array(vertices, dtype=np.float32)
    faces = np.array(faces, dtype=np.int32)
    
    # Create mesh
    paddle_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            paddle_mesh.vectors[i][j] = vertices[face[j], :]
    
    return paddle_mesh


if __name__ == "__main__":
    print("Generating table tennis paddle mesh...")
    paddle = create_paddle_mesh()
    
    # Save to STL
    output_path = "/home/bhavya-shah/Projects/ros2_ws/src/table_tennis_description/meshes/paddle/paddle.stl"
    paddle.save(output_path)
    print(f"Paddle mesh saved to: {output_path}")
    print(f"Vertices: {len(paddle.vectors) * 3}, Faces: {len(paddle.vectors)}")
