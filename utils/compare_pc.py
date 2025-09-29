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
                'location': [float(p) for p in parts[11:14]], # x, y, z (in camera coords)
                'rotation_y': float(parts[14])
            }
            if obj_data['type'] != 'DontCare':
                objects.append(obj_data)
    return objects

def read_calib_file(calib_path):
    """Reads a KITTI calibration file and returns the transformation matrices."""
    if not os.path.isfile(calib_path):
        raise FileNotFoundError(f"Error: Calibration file not found at {calib_path}")
    with open(calib_path) as f:
        lines = f.readlines()
    
    obj = lines[4].strip().split(' ')[1:]
    R0_rect = np.array(obj, dtype=np.float32).reshape(3, 3)

    obj = lines[5].strip().split(' ')[1:]
    Tr_velo_to_cam = np.array(obj, dtype=np.float32).reshape(3, 4)

    return {'R0_rect': R0_rect, 'Tr_velo_to_cam': Tr_velo_to_cam}

def project_camera_to_lidar(points_camera, calib):
    """Projects points from camera coordinates to LiDAR coordinates."""
    R0_rect_homo = np.eye(4)
    R0_rect_homo[:3, :3] = calib['R0_rect']
    
    Tr_velo_to_cam_homo = np.eye(4)
    Tr_velo_to_cam_homo[:3, :] = calib['Tr_velo_to_cam']

    inv_R0_rect = np.linalg.inv(R0_rect_homo)
    inv_Tr = np.linalg.inv(Tr_velo_to_cam_homo)
    
    points_camera_homo = np.hstack([points_camera, np.ones((points_camera.shape[0], 1))])
    points_lidar = points_camera_homo @ inv_R0_rect.T @ inv_Tr.T
    
    return points_lidar[:, :3]

def compute_box_corners(center_lidar, dimensions, yaw):
    """Computes the 8 corners of a 3D bounding box in LiDAR coordinates."""
    h, w, l = dimensions
    x, y, z = center_lidar
    
    R = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
    y_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
    z_corners = [-h/4, -h/4, -h/4, -h/4, h, h, h, h]
    
    corners = R @ np.vstack([x_corners, y_corners, z_corners])
    corners[0, :] += x
    corners[1, :] += y
    corners[2, :] += z
    
    return corners.T

def main():
    parser = argparse.ArgumentParser(description="Correctly visualize and compare raw and processed KITTI data.")
    parser.add_argument('raw_bin_file', type=str, help='Path to the original .bin point cloud file.')
    parser.add_argument('processed_bin_file', type=str, help='Path to the processed .bin point cloud file.')
    parser.add_argument('label_file', type=str, help='Path to the corresponding .txt label file.')
    parser.add_argument('calib_file', type=str, help='Path to the corresponding .txt calibration file.')
    parser.add_argument('output_file', type=str, help='Name for the output comparison HTML file.')
    parser.add_argument('--downsample', type=int, default=5, help='Downsample factor for point clouds.')
    args = parser.parse_args()

    try:
        raw_points = read_kitti_bin(args.raw_bin_file)
        processed_points = read_kitti_bin(args.processed_bin_file)
        objects = read_kitti_label(args.label_file)
        calib = read_calib_file(args.calib_file)
    except FileNotFoundError as e:
        print(e)
        return

    fig = go.Figure()

    # 1. Add RAW point cloud (magenta)
    fig.add_trace(go.Scatter3d(
        x=raw_points[::args.downsample, 0], y=raw_points[::args.downsample, 1], z=raw_points[::args.downsample, 2],
        mode='markers', marker=dict(size=1, color='magenta', opacity=0.7),
        name='Raw Point Cloud'
    ))

    # 2. Add PROCESSED point cloud (cyan)
    fig.add_trace(go.Scatter3d(
        x=processed_points[::args.downsample, 0], y=processed_points[::args.downsample, 1], z=processed_points[::args.downsample, 2],
        mode='markers', marker=dict(size=1.5, color='cyan', opacity=0.8),
        name='Processed Point Cloud'
    ))

    # 3. Add Transformed Bounding Boxes (red)
    for obj in objects:
        box_center_camera = np.array([obj['location']])
        box_center_lidar = project_camera_to_lidar(box_center_camera, calib)[0]
        lidar_yaw = -obj['rotation_y'] - (np.pi / 2)
        corners = compute_box_corners(box_center_lidar, obj['dimensions'], lidar_yaw)

        x_lines, y_lines, z_lines = [], [], []
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]
        for p1, p2 in edges:
            x_lines.extend([corners[p1, 0], corners[p2, 0], None])
            y_lines.extend([corners[p1, 1], corners[p2, 1], None])
            z_lines.extend([corners[p1, 2], corners[p2, 2], None])

        fig.add_trace(go.Scatter3d(
            x=x_lines, y=y_lines, z=z_lines,
            mode='lines', line=dict(color='red', width=2), name=f'Label: {obj["type"]}'
        ))

    fig.update_layout(
        title_text=f"Corrected Comparison for {os.path.basename(args.raw_bin_file)}",
        scene=dict(xaxis_title='X (LiDAR)', yaxis_title='Y (LiDAR)', zaxis_title='Z (Up)', aspectmode='data'),
        legend_title="Legend",
        font=dict(family="Arial", size=12, color="black")
    )

    fig.write_html(args.output_file)
    print(f"Successfully saved correct comparison visualization to '{args.output_file}'")


if __name__ == "__main__":
    main()