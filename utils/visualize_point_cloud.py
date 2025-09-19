import numpy as np
import plotly.graph_objects as go
import argparse
import os

def read_kitti_bin(bin_path):
    """Reads a KITTI .bin file and returns a numpy array of points."""
    if not os.path.isfile(bin_path):
        raise FileNotFoundError(f"Error: .bin file not found at {bin_path}")
    point_cloud = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return point_cloud

def read_kitti_label(label_path):
    """Reads a KITTI label .txt file and returns a list of objects."""
    if not os.path.isfile(label_path):
        raise FileNotFoundError(f"Error: .txt label file not found at {label_path}")
    objects = []
    with open(label_path, 'r') as f:
        for line in f.readlines():
            parts = line.strip().split(' ')
            obj_data = {
                'type': parts[0],
                'dimensions': [float(p) for p in parts[8:11]], # h, w, l
                'location': [float(p) for p in parts[11:14]], # x, y, z
                'rotation_y': float(parts[14])
            }
            objects.append(obj_data)
    return objects

def compute_box_corners(center, dimensions, yaw):
    """Computes the 8 corners of a 3D bounding box."""
    h, w, l = dimensions
    x, y, z = center
    R = np.array([
        [np.cos(yaw), 0, np.sin(yaw)],
        [0, 1, 0],
        [-np.sin(yaw), 0, np.cos(yaw)]
    ])
    x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
    
    # --- FIX IS HERE ---
    # The box should be drawn from the bottom 'y' coordinate UPWARDS by height 'h'.
    # Original line: y_corners = [0, 0, 0, 0, -h, -h, -h, -h] (Incorrectly drew downwards)
    y_corners = [0, 0, 0, 0, h, h, h, h]

    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
    corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d[0, :] += x
    corners_3d[1, :] += y
    corners_3d[2, :] += z
    return corners_3d.T

def main():
    parser = argparse.ArgumentParser(description="Compare raw and processed KITTI data and save to an interactive HTML file.")
    parser.add_argument('raw_bin_file', type=str, help='Path to the original .bin point cloud file.')
    parser.add_argument('processed_bin_file', type=str, help='Path to the processed .bin point cloud file.')
    parser.add_argument('label_file', type=str, help='Path to the corresponding .txt label file.')
    parser.add_argument('output_file', type=str, help='Name for the output comparison HTML file.')
    parser.add_argument('--downsample', type=int, default=10, help='Downsample factor to keep the HTML file size reasonable.')
    args = parser.parse_args()

    try:
        raw_points = read_kitti_bin(args.raw_bin_file)
        processed_points = read_kitti_bin(args.processed_bin_file)
        objects = read_kitti_label(args.label_file)
    except FileNotFoundError as e:
        print(e)
        return

    fig = go.Figure()

    # 1. Add RAW point cloud (in magenta)
    fig.add_trace(go.Scatter3d(
        x=raw_points[::args.downsample, 0], y=raw_points[::args.downsample, 2], z=raw_points[::args.downsample, 1],
        mode='markers', marker=dict(size=1, color='magenta', opacity=0.8), name='Raw Point Cloud'
    ))

    # 2. Add PROCESSED point cloud (in lime green)
    fig.add_trace(go.Scatter3d(
        x=processed_points[::args.downsample, 0], y=processed_points[::args.downsample, 2], z=processed_points[::args.downsample, 1],
        mode='markers', marker=dict(size=1.5, color='lime', opacity=0.8), name='Processed Point Cloud'
    ))


    # 3. Add bounding boxes (in red)
    for i, obj in enumerate(objects):
        if obj['type'] != 'DontCare':
            corners = compute_box_corners(obj['location'], obj['dimensions'], obj['rotation_y'])
            x_lines, y_lines, z_lines = [], [], []
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7)
            ]
            for p1, p2 in edges:
                x_lines.extend([corners[p1, 0], corners[p2, 0], None])
                y_lines.extend([corners[p1, 2], corners[p2, 2], None])
                z_lines.extend([corners[p1, 1], corners[p2, 1], None])

            fig.add_trace(go.Scatter3d(
                x=x_lines, y=y_lines, z=z_lines,
                mode='lines', line=dict(color='red', width=2), name=f'Label: {obj["type"]}'
            ))

    fig.update_layout(
        title_text=f"Comparison for {os.path.basename(args.raw_bin_file)}",
        scene=dict(xaxis_title='X', yaxis_title='Z', zaxis_title='Y (Up)', aspectmode='data')
    )

    fig.write_html(args.output_file)
    print(f"Successfully saved comparison visualization to '{args.output_file}'")


if __name__ == "__main__":
    main()

