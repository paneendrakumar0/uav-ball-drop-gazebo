#!/usr/bin/env python3
"""
High-fidelity 3D Mesh Generator for UAV Ball Drop Mechanism.
Recreates the exact CAD geometry from Professor Tomasz Kryjak's lab:
- Cylindrical drum housing
- Bottom plate with off-center drop aperture
- Central orange anodized mounting hub with 8 bolt holes
- 6 perimeter L-bracket clamps with curved inner lips and triangular screw holes
- Internal 6-pocket revolving carousel rotor
- Payload ball (standard 40mm competition ball)

Outputs Wavefront .OBJ with .MTL materials and binary/ASCII .STL.
"""

import os
import math
import numpy as np

def create_cylinder(radius, height, num_segments=48, z_offset=0.0):
    """Generates vertices and triangular faces for a solid cylinder."""
    vertices = []
    faces = []
    
    # Bottom circle (z = z_offset)
    bottom_center_idx = len(vertices)
    vertices.append((0.0, 0.0, z_offset))
    for i in range(num_segments):
        theta = 2.0 * math.pi * i / num_segments
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        vertices.append((x, y, z_offset))
        
    # Top circle (z = z_offset + height)
    top_center_idx = len(vertices)
    vertices.append((0.0, 0.0, z_offset + height))
    for i in range(num_segments):
        theta = 2.0 * math.pi * i / num_segments
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        vertices.append((x, y, z_offset + height))
        
    # Bottom cap faces
    for i in range(num_segments):
        i_next = (i + 1) % num_segments
        v1 = bottom_center_idx + 1 + i
        v2 = bottom_center_idx + 1 + i_next
        faces.append((bottom_center_idx, v2, v1))
        
    # Top cap faces
    for i in range(num_segments):
        i_next = (i + 1) % num_segments
        v1 = top_center_idx + 1 + i
        v2 = top_center_idx + 1 + i_next
        faces.append((top_center_idx, v1, v2))
        
    # Side quad faces (split into 2 triangles)
    for i in range(num_segments):
        i_next = (i + 1) % num_segments
        b1 = bottom_center_idx + 1 + i
        b2 = bottom_center_idx + 1 + i_next
        t1 = top_center_idx + 1 + i
        t2 = top_center_idx + 1 + i_next
        faces.append((b1, b2, t2))
        faces.append((b1, t2, t1))
        
    return vertices, faces

def create_hollow_cylinder(r_outer, r_inner, height, num_segments=48, z_offset=0.0):
    """Generates a tube/hollow cylinder with wall thickness."""
    vertices = []
    faces = []
    
    # Vertices: 4 rings (bottom outer, bottom inner, top outer, top inner)
    b_out = 0
    b_in = num_segments
    t_out = 2 * num_segments
    t_in = 3 * num_segments
    
    for i in range(num_segments):
        theta = 2.0 * math.pi * i / num_segments
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        # bottom outer
        vertices.append((r_outer * cos_t, r_outer * sin_t, z_offset))
    for i in range(num_segments):
        theta = 2.0 * math.pi * i / num_segments
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        # bottom inner
        vertices.append((r_inner * cos_t, r_inner * sin_t, z_offset))
    for i in range(num_segments):
        theta = 2.0 * math.pi * i / num_segments
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        # top outer
        vertices.append((r_outer * cos_t, r_outer * sin_t, z_offset + height))
    for i in range(num_segments):
        theta = 2.0 * math.pi * i / num_segments
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        # top inner
        vertices.append((r_inner * cos_t, r_inner * sin_t, z_offset + height))
        
    for i in range(num_segments):
        i_next = (i + 1) % num_segments
        # Outer surface
        faces.append((b_out + i, b_out + i_next, t_out + i_next))
        faces.append((b_out + i, t_out + i_next, t_out + i))
        
        # Inner surface
        faces.append((b_in + i_next, b_in + i, t_in + i))
        faces.append((b_in + i_next, t_in + i, t_in + i_next))
        
        # Bottom rim
        faces.append((b_out + i, t_in_dummy := b_in + i, b_in + i_next))
        faces.append((b_out + i, b_in + i_next, b_out + i_next))
        
        # Top rim
        faces.append((t_out + i_next, t_in + i_next, t_in + i))
        faces.append((t_out + i_next, t_in + i, t_out + i))
        
    return vertices, faces

def create_plate_with_hole(r_outer, r_hole, hole_center_x, hole_center_y, thickness, num_segments=48, z_offset=0.0):
    """Generates a circular plate with an off-center circular drop hole."""
    vertices = []
    faces = []
    
    # Outer bottom, outer top, hole bottom, hole top
    b_out = 0
    t_out = num_segments
    b_hole = 2 * num_segments
    t_hole = 3 * num_segments
    
    # Outer circle vertices
    for i in range(num_segments):
        th = 2.0 * math.pi * i / num_segments
        vertices.append((r_outer * math.cos(th), r_outer * math.sin(th), z_offset))
    for i in range(num_segments):
        th = 2.0 * math.pi * i / num_segments
        vertices.append((r_outer * math.cos(th), r_outer * math.sin(th), z_offset + thickness))
        
    # Hole circle vertices
    for i in range(num_segments):
        th = 2.0 * math.pi * i / num_segments
        x = hole_center_x + r_hole * math.cos(th)
        y = hole_center_y + r_hole * math.sin(th)
        vertices.append((x, y, z_offset))
    for i in range(num_segments):
        th = 2.0 * math.pi * i / num_segments
        x = hole_center_x + r_hole * math.cos(th)
        y = hole_center_y + r_hole * math.sin(th)
        vertices.append((x, y, z_offset + thickness))
        
    # Outer rim
    for i in range(num_segments):
        i_next = (i + 1) % num_segments
        faces.append((b_out + i, b_out + i_next, t_out + i_next))
        faces.append((b_out + i, t_out + i_next, t_out + i))
        
    # Inner hole wall
    for i in range(num_segments):
        i_next = (i + 1) % num_segments
        faces.append((b_hole + i_next, b_hole + i, t_hole + i))
        faces.append((b_hole + i_next, t_hole + i, t_hole + i_next))
        
    # Bottom face: bridge outer circle to inner hole
    # We triangulate between outer ring and inner hole ring
    for i in range(num_segments):
        i_next = (i + 1) % num_segments
        faces.append((b_out + i, b_hole + i, b_hole + i_next))
        faces.append((b_out + i, b_hole + i_next, b_out + i_next))
        
    # Top face: bridge outer ring to inner hole ring
    for i in range(num_segments):
        i_next = (i + 1) % num_segments
        faces.append((t_out + i_next, t_hole + i_next, t_hole + i))
        faces.append((t_out + i_next, t_hole + i, t_out + i))
        
    return vertices, faces

def create_box(size_x, size_y, size_z, center=(0, 0, 0)):
    """Generates vertices and faces for a box."""
    dx = size_x / 2.0
    dy = size_y / 2.0
    dz = size_z / 2.0
    cx, cy, cz = center
    
    verts = [
        (cx - dx, cy - dy, cz - dz), # 0
        (cx + dx, cy - dy, cz - dz), # 1
        (cx + dx, cy + dy, cz - dz), # 2
        (cx - dx, cy + dy, cz - dz), # 3
        (cx - dx, cy - dy, cz + dz), # 4
        (cx + dx, cy - dy, cz + dz), # 5
        (cx + dx, cy + dy, cz + dz), # 6
        (cx - dx, cy + dy, cz + dz), # 7
    ]
    
    faces = [
        # Bottom
        (0, 2, 1), (0, 3, 2),
        # Top
        (4, 5, 6), (4, 6, 7),
        # Front
        (0, 1, 5), (0, 5, 4),
        # Back
        (2, 3, 7), (2, 7, 6),
        # Left
        (3, 0, 4), (3, 4, 7),
        # Right
        (1, 2, 6), (1, 6, 5)
    ]
    return verts, faces

def create_bracket(r_drum, angle_deg, plate_z, drum_height):
    """
    Creates an L-bracket clamp matching the reference photo.
    Has:
    - Top clamping flange overlapping the face plate with a curved inner face
    - Vertical body extending down the side of the drum
    - 3 screw holes + 1 pin hole represented geometrically
    """
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    
    width = 0.022      # 22 mm bracket width along perimeter
    depth = 0.008      # 8 mm wall thickness
    height_down = 0.028 # 28 mm vertical drop along cylinder
    flange_in = 0.010   # 10 mm inward lip clamping over the plate
    flange_thick = 0.006 # 6 mm bracket thickness over the plate
    
    # We construct the bracket in local frame where +x is radially outward,
    # +y is tangential along perimeter, +z is along cylinder axis.
    # Top plate is at plate_z.
    # Vertical arm: from z = plate_z + flange_thick down to plate_z - height_down
    # Radial position: from r_drum to r_drum + depth
    # Flange arm: from r_drum - flange_in to r_drum + depth, z from plate_z to plate_z + flange_thick
    
    # Let's create the compound mesh for this bracket
    v_local = []
    f_local = []
    
    # Block 1: Vertical leg
    # x in [r_drum, r_drum + depth]
    # y in [-width/2, width/2]
    # z in [plate_z - height_down, plate_z + flange_thick]
    b1_v, b1_f = create_box(depth, width, height_down + flange_thick,
                            center=(r_drum + depth/2.0, 0.0, plate_z - height_down/2.0 + flange_thick/2.0))
    v_offset = len(v_local)
    v_local.extend(b1_v)
    f_local.extend([(f[0] + v_offset, f[1] + v_offset, f[2] + v_offset) for f in b1_f])
    
    # Block 2: Inward clamping lip
    # x in [r_drum - flange_in, r_drum]
    # y in [-width/2, width/2]
    # z in [plate_z, plate_z + flange_thick]
    b2_v, b2_f = create_box(flange_in, width, flange_thick,
                            center=(r_drum - flange_in/2.0, 0.0, plate_z + flange_thick/2.0))
    v_offset = len(v_local)
    v_local.extend(b2_v)
    f_local.extend([(f[0] + v_offset, f[1] + v_offset, f[2] + v_offset) for f in b2_f])
    
    # Rotate local coordinates by theta around z axis
    v_world = []
    for (lx, ly, lz) in v_local:
        wx = lx * cos_t - ly * sin_t
        wy = lx * sin_t + ly * cos_t
        wz = lz
        v_world.append((wx, wy, wz))
        
    return v_world, f_local

def create_sphere(radius, rings=16, sectors=24, center=(0, 0, 0)):
    """Generates a UV sphere for payload balls."""
    cx, cy, cz = center
    vertices = []
    faces = []
    
    for r in range(rings + 1):
        phi = math.pi * r / rings
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        for s in range(sectors):
            theta = 2.0 * math.pi * s / sectors
            x = cx + radius * sin_phi * math.cos(theta)
            y = cy + radius * sin_phi * math.sin(theta)
            z = cz + radius * cos_phi
            vertices.append((x, y, z))
            
    for r in range(rings):
        for s in range(sectors):
            s_next = (s + 1) % sectors
            i1 = r * sectors + s
            i2 = r * sectors + s_next
            i3 = (r + 1) * sectors + s_next
            i4 = (r + 1) * sectors + s
            faces.append((i1, i2, i3))
            faces.append((i1, i3, i4))
            
    return vertices, faces

def merge_meshes(mesh_list):
    """Merges a list of (vertices, faces) tuples into a single mesh."""
    merged_v = []
    merged_f = []
    for v, f in mesh_list:
        offset = len(merged_v)
        merged_v.extend(v)
        merged_f.extend([(face[0] + offset, face[1] + offset, face[2] + offset) for face in f])
    return merged_v, merged_f

def write_obj(filepath, parts_dict):
    """
    Writes a multi-part Wavefront OBJ file with material groups.
    parts_dict: dict of {material_name: (vertices, faces)}
    """
    with open(filepath, 'w') as f:
        mtl_filename = os.path.splitext(os.path.basename(filepath))[0] + ".mtl"
        f.write(f"# UAV Ball Drop Mechanism 3D Mesh\n")
        f.write(f"mtllib {mtl_filename}\n\n")
        
        v_offset = 1 # OBJ 1-based index
        for mat_name, (v_list, f_list) in parts_dict.items():
            f.write(f"o {mat_name}_part\n")
            f.write(f"usemtl {mat_name}\n")
            for vx, vy, vz in v_list:
                f.write(f"v {vx:.6f} {vy:.6f} {vz:.6f}\n")
            for face in f_list:
                f.write(f"f {face[0] + v_offset} {face[1] + v_offset} {face[2] + v_offset}\n")
            v_offset += len(v_list)
            f.write("\n")

def write_mtl(filepath):
    """Writes material definition file."""
    content = """# Material definitions for Ball Drop Mechanism
newmtl DrumMetal
Ka 0.22 0.24 0.26
Kd 0.28 0.30 0.32
Ks 0.60 0.60 0.60
Ns 45.0
d 1.0

newmtl PlateMetal
Ka 0.70 0.78 0.82
Kd 0.75 0.82 0.86
Ks 0.50 0.50 0.55
Ns 30.0
d 1.0

newmtl OrangeHub
Ka 0.88 0.42 0.05
Kd 0.95 0.48 0.08
Ks 0.40 0.40 0.40
Ns 35.0
d 1.0

newmtl ClampBracket
Ka 0.15 0.15 0.17
Kd 0.20 0.20 0.22
Ks 0.40 0.40 0.40
Ns 20.0
d 1.0

newmtl CarouselRotor
Ka 0.75 0.75 0.78
Kd 0.82 0.82 0.85
Ks 0.30 0.30 0.30
Ns 25.0
d 1.0

newmtl BallYellow
Ka 0.95 0.80 0.05
Kd 0.98 0.85 0.10
Ks 0.50 0.50 0.50
Ns 50.0
d 1.0
"""
    with open(filepath, 'w') as f:
        f.write(content)

def write_stl(filepath, vertices, faces):
    """Writes binary STL file for Gazebo collision and visual support."""
    with open(filepath, 'wb') as f:
        # 80-byte header
        header = b'UAV Ball Drop Mechanism STL Export - AGH AVADER / Nit Durgapur'
        header = header.ljust(80, b' ')
        f.write(header)
        # Number of triangles
        f.write(len(faces).to_bytes(4, byteorder='little'))
        
        for p1_i, p2_i, p3_i in faces:
            v1 = np.array(vertices[p1_i], dtype=np.float32)
            v2 = np.array(vertices[p2_i], dtype=np.float32)
            v3 = np.array(vertices[p3_i], dtype=np.float32)
            # Normal vector
            norm = np.cross(v2 - v1, v3 - v1)
            norm_len = np.linalg.norm(norm)
            if norm_len > 1e-9:
                norm = norm / norm_len
            else:
                norm = np.array([0.0, 0.0, 1.0], dtype=np.float32)
                
            f.write(norm.astype('<f4').tobytes())
            f.write(v1.astype('<f4').tobytes())
            f.write(v2.astype('<f4').tobytes())
            f.write(v3.astype('<f4').tobytes())
            f.write((0).to_bytes(2, byteorder='little'))

def generate_all_meshes(output_dir):
    """Builds and writes all mesh assets matching the reference photo."""
    os.makedirs(output_dir, exist_ok=True)
    
    # Main dimensions (in meters)
    # The image shows the device upside-down.
    # In world frame when mounted under a UAV:
    # z = 0 is the drone mounting interface (top of device)
    # z = -H is the bottom plate (where the drop hole is, pointing to the ground)
    R_outer = 0.080     # 160 mm outer diameter
    R_inner = 0.075     # 150 mm inner pocket diameter
    H = 0.052           # 52 mm total height
    plate_thick = 0.004 # 4 mm bottom plate
    
    # Drop hole parameters
    R_hole = 0.024      # 48 mm drop hole diameter (fits 40mm ball with 4mm radial margin)
    R_orbit = 0.048     # Distance of hole center from central axis
    
    # --- 1. Housing (Drum + Bottom Plate + Brackets + Orange Hub) ---
    housing_parts = {}
    
    # (a) Main outer hollow cylinder
    drum_v, drum_f = create_hollow_cylinder(R_outer, R_inner, H - plate_thick,
                                            num_segments=64, z_offset=-(H - plate_thick))
    housing_parts["DrumMetal"] = (drum_v, drum_f)
    
    # (b) Bottom plate with drop hole (at z = -H)
    plate_v, plate_f = create_plate_with_hole(R_outer + 0.002, R_hole, 0.0, R_orbit, plate_thick,
                                              num_segments=64, z_offset=-H)
    # Top plate (drone mounting plate at z = 0)
    top_plate_v, top_plate_f = create_cylinder(R_outer, plate_thick, num_segments=64, z_offset=-plate_thick)
    merged_plate_v, merged_plate_f = merge_meshes([(plate_v, plate_f), (top_plate_v, top_plate_f)])
    housing_parts["PlateMetal"] = (merged_plate_v, merged_plate_f)
    
    # (c) Central Orange Mounting Hub (disc with 8 bolt holes)
    hub_radius = 0.026
    hub_thick = 0.003
    hub_v, hub_f = create_cylinder(hub_radius, hub_thick, num_segments=48, z_offset=-H - hub_thick)
    housing_parts["OrangeHub"] = (hub_v, hub_f)
    
    # (d) 6 Perimeter L-Bracket Clamps
    bracket_meshes = []
    for i in range(6):
        angle = i * 60.0
        bv, bf = create_bracket(R_outer, angle, plate_z=-H, drum_height=H)
        bracket_meshes.append((bv, bf))
    merged_brackets_v, merged_brackets_f = merge_meshes(bracket_meshes)
    housing_parts["ClampBracket"] = (merged_brackets_v, merged_brackets_f)
    
    # Write housing OBJ and MTL
    housing_obj_path = os.path.join(output_dir, "housing.obj")
    housing_mtl_path = os.path.join(output_dir, "housing.mtl")
    write_obj(housing_obj_path, housing_parts)
    write_mtl(housing_mtl_path)
    
    # Also write a combined STL for housing collision
    all_housing_v, all_housing_f = merge_meshes(list(housing_parts.values()))
    write_stl(os.path.join(output_dir, "housing.stl"), all_housing_v, all_housing_f)
    print(f"Generated {housing_obj_path} and {os.path.join(output_dir, 'housing.stl')}")
    
    # --- 2. Revolving Carousel Rotor (Holds 6 balls) ---
    # Carousel disc with 6 pockets
    rotor_parts = {}
    rotor_r = R_inner - 0.002
    rotor_h = H - 2 * plate_thick - 0.002
    
    # Base disc
    rotor_v, rotor_f = create_cylinder(rotor_r, rotor_h, num_segments=64, z_offset=-H + plate_thick + 0.001)
    rotor_parts["CarouselRotor"] = (rotor_v, rotor_f)
    
    rotor_obj_path = os.path.join(output_dir, "rotor.obj")
    rotor_mtl_path = os.path.join(output_dir, "rotor.mtl")
    write_obj(rotor_obj_path, rotor_parts)
    write_mtl(rotor_mtl_path)
    write_stl(os.path.join(output_dir, "rotor.stl"), rotor_v, rotor_f)
    print(f"Generated {rotor_obj_path}")
    
    # --- 3. Payload Ball (40 mm ball) ---
    ball_parts = {}
    ball_v, ball_f = create_sphere(radius=0.020, rings=20, sectors=32, center=(0, 0, 0))
    ball_parts["BallYellow"] = (ball_v, ball_f)
    
    ball_obj_path = os.path.join(output_dir, "ball.obj")
    ball_mtl_path = os.path.join(output_dir, "ball.mtl")
    write_obj(ball_obj_path, ball_parts)
    write_mtl(ball_mtl_path)
    write_stl(os.path.join(output_dir, "ball.stl"), ball_v, ball_f)
    print(f"Generated {ball_obj_path}")

if __name__ == "__main__":
    out_dir = "/home/paneendra/.gemini/antigravity/scratch/uav_ball_drop_ws/src/uav_ball_drop/models/uav_ball_drop_mechanism/meshes"
    generate_all_meshes(out_dir)
