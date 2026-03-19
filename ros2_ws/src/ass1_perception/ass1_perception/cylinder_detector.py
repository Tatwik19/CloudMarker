#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray


class PipelineConfig:
    def __init__(self):
        self.topic = '/oakd/points'
        self.voxel_size = 0.02
        self.box_min = np.array([-1.0, -0.6, 0.2], dtype=np.float64)
        self.box_max = np.array([1.0, 0.6, 5.0], dtype=np.float64)

        self.floor_dist = 0.02
        self.target_normal = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        self.normal_thresh = 0.85

        self.cyl_radius = 0.055
        self.max_cylinders = 3

        self.plane_ransac_iters = 500


        # Task 2: clustering
        self.cluster_dist = 0.05
        self.min_cluster_size = 80
        self.max_cluster_size_pts = 3500

        # Task 3: normals + cylinder RANSAC
        self.normal_k = 10
        self.cyl_ransac_iters = 750
        self.cyl_dist_thresh = 0.02
        self.axis_align_thresh = 0.75
        self.center_tol = 0.08
        self.normal_perp_thresh = 0.45
        self.min_cylinder_inliers = 75
        self.min_cylinder_inlier_ratio = 0.12

def pointcloud2_to_xyzrgb(msg: PointCloud2):
    field_map = {f.name: f.offset for f in msg.fields}

    required = ['x', 'y', 'z']
    for name in required:
        if name not in field_map:
            raise ValueError(f"Missing field '{name}' in PointCloud2")

    n_points = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, msg.point_step)

    x = raw[:, field_map['x']:field_map['x'] + 4].copy().view(np.float32).reshape(-1)
    y = raw[:, field_map['y']:field_map['y'] + 4].copy().view(np.float32).reshape(-1)
    z = raw[:, field_map['z']:field_map['z'] + 4].copy().view(np.float32).reshape(-1)

    xyz = np.stack((x, y, z), axis=1)

    rgb = None
    if 'rgb' in field_map:
        rgb_f = raw[:, field_map['rgb']:field_map['rgb'] + 4].copy().view(np.float32).reshape(-1)
        rgb_u = rgb_f.view(np.uint32)

        r = ((rgb_u >> 16) & 255).astype(np.uint8)
        g = ((rgb_u >> 8) & 255).astype(np.uint8)
        b = (rgb_u & 255).astype(np.uint8)

        rgb = np.stack((r, g, b), axis=1)

    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]
    if rgb is not None:
        rgb = rgb[valid]

    return xyz, rgb

def to_uint8_rgb(colors, n_points):
    if colors is None:
        return np.full((n_points, 3), 255, dtype=np.uint8)

    colors = np.asarray(colors)
    if colors.shape[0] != n_points:
        raise ValueError('colors must have the same number of rows as pts')

    if colors.dtype.kind in ('u', 'i'):
        return np.clip(colors, 0, 255).astype(np.uint8)

    return np.clip(colors, 0.0, 1.0) * 255.0


def numpy_to_pc2_rgb(pts, colors, frame_id, stamp=None):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    colors_u8 = to_uint8_rgb(colors, pts.shape[0]).astype(np.uint32)

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp

    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    rgb_packed = (
        (255 << 24)
        | (colors_u8[:, 0] << 16)
        | (colors_u8[:, 1] << 8)
        | colors_u8[:, 2]
    )

    data = np.empty((pts.shape[0], 4), dtype=np.float32)
    data[:, :3] = pts
    data[:, 3] = rgb_packed.view(np.float32)
    msg.data = data.tobytes()
    return msg


def box_filter(xyz, rgb, box_min, box_max):
    mask = np.all((xyz >= box_min) & (xyz <= box_max), axis=1)
    xyz_out = xyz[mask]
    rgb_out = rgb[mask] if rgb is not None else None
    return xyz_out, rgb_out


def downsample(xyz, rgb, voxel_size):
    if xyz.shape[0] == 0:
        return xyz, rgb

    voxel_indices = np.floor(xyz / voxel_size).astype(np.int32)
    unique_voxels, inverse = np.unique(voxel_indices, axis=0, return_inverse=True)
    n_voxels = unique_voxels.shape[0]
    counts = np.bincount(inverse)

    xyz_sum = np.zeros((n_voxels, 3), dtype=np.float64)
    np.add.at(xyz_sum, inverse, xyz)
    xyz_out = (xyz_sum / counts[:, None]).astype(np.float32)

    rgb_out = None
    if rgb is not None:
        rgb_sum = np.zeros((n_voxels, 3), dtype=np.float64)
        np.add.at(rgb_sum, inverse, rgb.astype(np.float64))
        rgb_out = np.clip(rgb_sum / counts[:, None], 0, 255).astype(np.uint8)

    return xyz_out, rgb_out


def fit_plane_from_points(p1, p2, p3):
    v1 = p2 - p1
    v2 = p3 - p1
    n = np.cross(v1, v2)
    norm = np.linalg.norm(n)
    if norm < 1e-8:
        return None

    n = n / norm
    d = -np.dot(n, p1)
    return np.array([n[0], n[1], n[2], d], dtype=np.float64)


def plane_distances(points, plane):
    a, b, c, d = plane
    denom = np.sqrt(a * a + b * b + c * c)
    if denom < 1e-12:
        return np.full(points.shape[0], np.inf)
    return np.abs(points[:, 0] * a + points[:, 1] * b + points[:, 2] * c + d) / denom


def refit_plane_svd(points):
    if points.shape[0] < 3:
        return None

    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1, :]
    norm = np.linalg.norm(normal)
    if norm < 1e-8:
        return None

    normal = normal / norm
    d = -np.dot(normal, centroid)
    return np.array([normal[0], normal[1], normal[2], d], dtype=np.float64)


def ransac_plane(
    points,
    target_normal,
    normal_thresh,
    max_iterations=500,
    distance_threshold=0.02,
    min_inliers=50,
):
    n_points = points.shape[0]
    if n_points < 3:
        return None, None

    target_normal = np.asarray(target_normal, dtype=np.float64)
    target_normal_norm = np.linalg.norm(target_normal)
    if target_normal_norm < 1e-8:
        raise ValueError('target_normal must be non-zero')
    target_normal = target_normal / target_normal_norm

    best_plane = None
    best_inliers = None
    best_count = 0

    rng = np.random.default_rng()

    for _ in range(max_iterations):
        idx = rng.choice(n_points, size=3, replace=False)
        p1, p2, p3 = points[idx]

        plane = fit_plane_from_points(p1, p2, p3)
        if plane is None:
            continue

        plane_normal = plane[:3]
        if abs(np.dot(plane_normal, target_normal)) < normal_thresh:
            continue

        distances = plane_distances(points, plane)
        inliers = distances < distance_threshold
        count = int(np.sum(inliers))

        if count > best_count:
            best_count = count
            best_plane = plane
            best_inliers = inliers

    if best_inliers is None or best_count < min_inliers:
        return None, None

    refined_plane = refit_plane_svd(points[best_inliers])
    if refined_plane is None:
        refined_plane = best_plane

    refined_normal = refined_plane[:3]
    refined_norm = np.linalg.norm(refined_normal)
    if refined_norm < 1e-8:
        return None, None
    refined_normal = refined_normal / refined_norm

    if abs(np.dot(refined_normal, target_normal)) < normal_thresh:
        return None, None

    refined_distances = plane_distances(points, refined_plane)
    refined_inliers = refined_distances < distance_threshold

    return refined_plane, refined_inliers


def remove_plane(points, colors, inliers):
    if inliers is None:
        return points, colors

    keep = ~inliers
    points_out = points[keep]
    colors_out = colors[keep] if colors is not None else None
    return points_out, colors_out


def euclidean_clusters(points, distance_threshold=0.1, min_cluster_size=20, max_cluster_size=1000):
    n = points.shape[0]
    if n == 0:
        return []

    tree = cKDTree(points)
    unvisited = np.ones(n, dtype=bool)
    clusters = []

    for seed in range(n):
        if not unvisited[seed]:
            continue

        queue = [seed]
        unvisited[seed] = False
        cluster_indices = []

        while queue:
            current = queue.pop()
            cluster_indices.append(current)

            neighbors = tree.query_ball_point(points[current], r=distance_threshold)
            if len(neighbors) == 0:
                continue

            neighbors = np.asarray(neighbors, dtype=np.int32)
            new_neighbors = neighbors[unvisited[neighbors]]
            if new_neighbors.size > 0:
                unvisited[new_neighbors] = False
                queue.extend(new_neighbors.tolist())

        if min_cluster_size <= len(cluster_indices) <= max_cluster_size:
            clusters.append(np.array(cluster_indices, dtype=np.int32))

    return clusters


def cluster_stats(points):
    centroid = np.mean(points, axis=0)
    mins = np.min(points, axis=0)
    maxs = np.max(points, axis=0)
    size = maxs - mins
    return centroid, mins, maxs, size


def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm < 1e-8:
        return None
    return v / norm


def estimate_normals(points, k=15):
    n_points = points.shape[0]
    if n_points < 3:
        return np.zeros((n_points, 3), dtype=np.float32)

    k = int(max(3, min(k, n_points)))
    tree = cKDTree(points)
    _, nn_idx = tree.query(points, k=k)

    if k == 1:
        nn_idx = nn_idx[:, None]

    normals = np.zeros((n_points, 3), dtype=np.float64)

    for i in range(n_points):
        neighborhood = points[nn_idx[i]]
        centroid = np.mean(neighborhood, axis=0)
        centered = neighborhood - centroid

        if centered.shape[0] < 3:
            continue

        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1]
        normal = normalize_vector(normal)
        if normal is None:
            continue

        normals[i] = normal

    return normals.astype(np.float32)


def cylinder_inliers(points, normals, axis_point, axis_dir, radius, radius_threshold, normal_perp_thresh):
    rel = points - axis_point
    axial = rel @ axis_dir
    radial_vec = rel - np.outer(axial, axis_dir)
    radial_dist = np.linalg.norm(radial_vec, axis=1)
    radius_error = np.abs(radial_dist - radius)

    inliers = radius_error < radius_threshold

    if normals is not None:
        normal_dot = np.abs(normals @ axis_dir)
        inliers &= normal_dot < normal_perp_thresh

    return inliers


def find_single_cylinder(
    points,
    normals,
    cyl_radius,
    target_axis,
    axis_align_thresh,
    max_iterations=300,
    radius_threshold=0.02,
    center_tol=0.06,
    normal_perp_thresh=0.35,
    min_inliers=60,
    min_inlier_ratio=0.35,
):
    n_points = points.shape[0]
    if n_points < 10 or normals is None or normals.shape[0] != n_points:
        return None, None

    target_axis = normalize_vector(np.asarray(target_axis, dtype=np.float64))
    if target_axis is None:
        raise ValueError('target_axis must be non-zero')

    valid_normals = np.linalg.norm(normals, axis=1) > 0.5
    candidate_idx = np.where(valid_normals)[0]
    if candidate_idx.shape[0] < 2:
        return None, None

    min_required = max(min_inliers, int(np.ceil(min_inlier_ratio * n_points)))

    rng = np.random.default_rng()
    best_model = None
    best_inliers = None
    best_count = 0

    for _ in range(max_iterations):
        idx = rng.choice(candidate_idx, size=2, replace=False)
        p1, p2 = points[idx]
        n1, n2 = normals[idx]

        axis = np.cross(n1, n2)
        axis = normalize_vector(axis)
        if axis is None:
            continue

        if abs(np.dot(axis, target_axis)) < axis_align_thresh:
            continue

        if np.dot(axis, target_axis) < 0.0:
            axis = -axis

        n1_proj = n1 - np.dot(n1, axis) * axis
        n2_proj = n2 - np.dot(n2, axis) * axis
        n1_proj = normalize_vector(n1_proj)
        n2_proj = normalize_vector(n2_proj)

        if n1_proj is None or n2_proj is None:
            continue

        axial_mid = 0.5 * (np.dot(p1, axis) + np.dot(p2, axis))

        for s1 in (-1.0, 1.0):
            for s2 in (-1.0, 1.0):
                c1 = p1 + s1 * cyl_radius * n1_proj
                c2 = p2 + s2 * cyl_radius * n2_proj

                c1_perp = c1 - np.dot(c1, axis) * axis
                c2_perp = c2 - np.dot(c2, axis) * axis
                center_gap = np.linalg.norm(c1_perp - c2_perp)

                if center_gap > center_tol:
                    continue

                axis_point = 0.5 * (c1_perp + c2_perp) + axial_mid * axis

                inliers = cylinder_inliers(
                    points,
                    normals,
                    axis_point,
                    axis,
                    cyl_radius,
                    radius_threshold,
                    normal_perp_thresh,
                )
                count = int(np.sum(inliers))

                if count > best_count:
                    best_count = count
                    best_model = (axis_point.astype(np.float32), axis.astype(np.float32), float(cyl_radius))
                    best_inliers = inliers

    if best_model is None or best_count < min_required:
        return None, None

    return best_model, best_inliers


def rgb_to_hsv(r, g, b):
    r = float(r)
    g = float(g)
    b = float(b)

    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx - mn

    if df < 1e-8:
        h = 0.0
    elif mx == r:
        h = (60.0 * ((g - b) / df) + 360.0) % 360.0
    elif mx == g:
        h = (60.0 * ((b - r) / df) + 120.0) % 360.0
    else:
        h = (60.0 * ((r - g) / df) + 240.0) % 360.0

    s = 0.0 if mx < 1e-8 else (df / mx)
    v = mx
    return h, s, v


def classify_cluster_color(cluster_rgb):
    if cluster_rgb is None or cluster_rgb.shape[0] == 0:
        return 'unknown', (1.0, 1.0, 1.0)

    mean_rgb = cluster_mean_rgb(cluster_rgb)
    mean_rgb01 = np.clip(mean_rgb / 255.0, 0.0, 1.0)
    h, s, v = rgb_to_hsv(mean_rgb01[0], mean_rgb01[1], mean_rgb01[2])

    if v < 0.12 or s < 0.05:
        return 'unknown', tuple(mean_rgb01.tolist())

    if 270.0 <= h < 350.0 and s >= 0.1 and v >= 0.1:
        return 'pink', (1.0, 0.4, 0.7)
    if h >= 350.0 or h < 20.0:
        return 'red', (1.0, 0.0, 0.0)
    if 70.0 <= h < 170.0:
        return 'green', (0.0, 1.0, 0.0)
    if 190.0 <= h < 270.0:
        return 'blue', (0.0, 0.0, 1.0)

    return 'unknown', tuple(mean_rgb01.tolist())
    
def cluster_mean_rgb(cluster_rgb):
    if cluster_rgb is None or cluster_rgb.shape[0] == 0:
        return np.array([0.0, 0.0, 0.0], dtype=np.float64)
    return np.mean(cluster_rgb.astype(np.float64), axis=0)


def concat_cloud_parts(parts_xyz, parts_rgb):
    xyz_list = []
    rgb_list = []

    for xyz, rgb in zip(parts_xyz, parts_rgb):
        if xyz is None or xyz.shape[0] == 0:
            continue

        xyz_list.append(np.asarray(xyz, dtype=np.float32))

        if rgb is None:
            rgb_list.append(np.full((xyz.shape[0], 3), 255, dtype=np.uint8))
        else:
            rgb_list.append(np.asarray(rgb, dtype=np.uint8))

    if len(xyz_list) == 0:
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.uint8),
        )

    return np.vstack(xyz_list), np.vstack(rgb_list)


def build_semantic_cloud(clusters_xyz, clusters_rgb):
    semantic_xyz = []
    semantic_rgb = []

    for xyz, rgb in zip(clusters_xyz, clusters_rgb):
        if xyz is None or xyz.shape[0] == 0:
            continue

        _, rgb01 = classify_cluster_color(rgb)
        rgb255 = np.round(255.0 * np.clip(np.asarray(rgb01), 0.0, 1.0)).astype(np.uint8)

        semantic_xyz.append(np.asarray(xyz, dtype=np.float32))
        semantic_rgb.append(np.tile(rgb255[None, :], (xyz.shape[0], 1)))

    return concat_cloud_parts(semantic_xyz, semantic_rgb)


def make_cylinder_markers(cylinder_models, clusters_xyz, clusters_rgb, frame_id):
    markers = MarkerArray()

    delete_all = Marker()
    delete_all.action = Marker.DELETEALL
    markers.markers.append(delete_all)

    marker_id = 0

    for model, cluster_xyz, cluster_rgb in zip(cylinder_models, clusters_xyz, clusters_rgb):
        axis_point, axis, radius = model
        axis = normalize_vector(axis)
        if axis is None:
            continue

        projections = cluster_xyz @ axis
        t_min = float(np.min(projections))
        t_max = float(np.max(projections))
        height = max(t_max - t_min, 0.05)

        axis_perp = axis_point - np.dot(axis_point, axis) * axis
        center = axis_perp + 0.5 * (t_min + t_max) * axis

        color_name, (cr, cg, cb) = classify_cluster_color(cluster_rgb)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = 'cylinders'
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])

        marker.pose.orientation.x = 0.70710678
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.70710678

        marker.scale.x = float(2.0 * radius)
        marker.scale.y = float(2.0 * radius)
        marker.scale.z = float(height) # 0.4

        marker.color.a = 0.75
        marker.color.r = float(cr)
        marker.color.g = float(cg)
        marker.color.b = float(cb)

        markers.markers.append(marker)

        text = Marker()
        text.header.frame_id = frame_id
        text.ns = 'cylinder_labels'
        text.id = marker_id
        marker_id += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.pose.position.x = float(center[0])
        text.pose.position.y = float(center[1] + 0.5 * height + 0.07)
        text.pose.position.z = float(center[2])
        text.pose.orientation.w = 1.0

        text.scale.z = 0.05
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = color_name

        markers.markers.append(text)

    return markers


class CylinderDetector(Node):
    def __init__(self):
        super().__init__('cylinder_detector')

        self.cfg = PipelineConfig()

        self.declare_parameter('cloud_topic', self.cfg.topic)
        cloud_topic = self.get_parameter('cloud_topic').value

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_callback,
            10,
        )

        self.pub_stage0 = self.create_publisher(PointCloud2, 'pipeline/stage0_box', 10)
        self.pub_stage1 = self.create_publisher(PointCloud2, 'pipeline/stage1_voxel', 10)
        self.pub_stage2 = self.create_publisher(PointCloud2, 'pipeline/stage2_nonplane', 10)
        self.pub_stage3 = self.create_publisher(PointCloud2, 'pipeline/stage3_candidates', 10)
        self.pub_stage4 = self.create_publisher(PointCloud2, 'pipeline/stage4_cylinders', 10)

        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/cylinder_markers',
            10,
        )

        self.msg_count = 0
        self.get_logger().info(f'Subscribed to: {cloud_topic}')

    def cloud_callback(self, msg: PointCloud2):
        self.msg_count += 1

        xyz, rgb = pointcloud2_to_xyzrgb(msg)
        n_raw = xyz.shape[0]

        xyz, rgb = box_filter(xyz, rgb, self.cfg.box_min, self.cfg.box_max)
        n_box = xyz.shape[0]
        self.pub_stage0.publish(numpy_to_pc2_rgb(xyz, rgb, msg.header.frame_id, msg.header.stamp))

        xyz, rgb = downsample(xyz, rgb, voxel_size=self.cfg.voxel_size)
        n_voxel = xyz.shape[0]
        self.pub_stage1.publish(numpy_to_pc2_rgb(xyz, rgb, msg.header.frame_id, msg.header.stamp))


        plane, inliers = ransac_plane(
            xyz,
            target_normal=self.cfg.target_normal,
            normal_thresh=self.cfg.normal_thresh,
            max_iterations=self.cfg.plane_ransac_iters,
            distance_threshold=self.cfg.floor_dist,
        )

        if inliers is not None:
            n_plane = int(np.sum(inliers))
            xyz_no_plane, rgb_no_plane = remove_plane(xyz, rgb, inliers)
        else:
            n_plane = 0
            xyz_no_plane, rgb_no_plane = xyz, rgb

        n_no_plane = xyz_no_plane.shape[0]
        self.pub_stage2.publish(numpy_to_pc2_rgb(xyz_no_plane, rgb_no_plane, msg.header.frame_id, msg.header.stamp))

        clusters_idx = euclidean_clusters(
            xyz_no_plane,
            distance_threshold=self.cfg.cluster_dist,
            min_cluster_size=self.cfg.min_cluster_size,
            max_cluster_size=self.cfg.max_cluster_size_pts,
        )

        clusters_xyz = [xyz_no_plane[idx] for idx in clusters_idx]
        clusters_rgb = [rgb_no_plane[idx] if rgb_no_plane is not None else None for idx in clusters_idx]

        candidate_xyz, candidate_rgb = concat_cloud_parts(clusters_xyz, clusters_rgb)
        self.pub_stage3.publish(numpy_to_pc2_rgb(candidate_xyz, candidate_rgb, msg.header.frame_id, msg.header.stamp))

        detections = []
        cluster_logs = []

        for cluster_xyz, cluster_rgb in zip(clusters_xyz, clusters_rgb):
            normals = estimate_normals(cluster_xyz, k=self.cfg.normal_k)
            model, cyl_inliers = find_single_cylinder(
                cluster_xyz,
                normals,
                cyl_radius=self.cfg.cyl_radius,
                target_axis=self.cfg.target_normal,
                axis_align_thresh=self.cfg.axis_align_thresh,
                max_iterations=self.cfg.cyl_ransac_iters,
                radius_threshold=self.cfg.cyl_dist_thresh,
                center_tol=self.cfg.center_tol,
                normal_perp_thresh=self.cfg.normal_perp_thresh,
                min_inliers=self.cfg.min_cylinder_inliers,
                min_inlier_ratio=self.cfg.min_cylinder_inlier_ratio,
            )

            inlier_count = 0 if cyl_inliers is None else int(np.sum(cyl_inliers))
            accepted = model is not None and cyl_inliers is not None

            cluster_logs.append((cluster_xyz, cluster_rgb, inlier_count, accepted))

            if not accepted:
                continue

            inlier_xyz = cluster_xyz[cyl_inliers]
            inlier_rgb = cluster_rgb[cyl_inliers] if cluster_rgb is not None else None
            detections.append((inlier_count, model, inlier_xyz, inlier_rgb))

        detections.sort(key=lambda item: item[0], reverse=True)
        detections = detections[:self.cfg.max_cylinders]

        cylinder_models = [item[1] for item in detections]
        cylinder_clusters = [item[2] for item in detections]
        cylinder_clusters_rgb = [item[3] for item in detections]
        best_inliers = 0 if len(detections) == 0 else int(detections[0][0])

        cylinder_cloud_xyz, cylinder_cloud_rgb = build_semantic_cloud(cylinder_clusters, cylinder_clusters_rgb)
        self.pub_stage4.publish(numpy_to_pc2_rgb(cylinder_cloud_xyz, cylinder_cloud_rgb, msg.header.frame_id, msg.header.stamp))

        markers = make_cylinder_markers(
            cylinder_models,
            cylinder_clusters,
            cylinder_clusters_rgb,
            msg.header.frame_id,
        )

        self.pub_markers.publish(markers)

        if self.msg_count % 3 == 0:
            self.get_logger().info(
                f'cloud #{self.msg_count} | '
                f'raw={n_raw} | box={n_box} | voxel={n_voxel} | '
                f'plane={n_plane} | nonplane={n_no_plane} | '
                f'clusters={len(clusters_xyz)} | cylinders={len(cylinder_models)} | '
                f'best_cyl_inliers={best_inliers}'
            )

            for i, (cluster_xyz, cluster_rgb, inlier_count, accepted) in enumerate(cluster_logs[:10]):
                centroid, mins, maxs, size = cluster_stats(cluster_xyz)
                mean_rgb = cluster_mean_rgb(cluster_rgb)
                mean_hsv = rgb_to_hsv(*(np.clip(mean_rgb / 255.0, 0.0, 1.0)))
                color_name, _ = classify_cluster_color(cluster_rgb)

                self.get_logger().info(
                    f'  cluster {i} | pts={cluster_xyz.shape[0]} | '
                    f'centroid=({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}) | '
                    f'size=({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}) | '
                    f'mean_rgb=({mean_rgb[0]:.1f}, {mean_rgb[1]:.1f}, {mean_rgb[2]:.1f}) | '
                    f'mean_hsv=({mean_hsv[0]:.1f}, {mean_hsv[1]:.2f}, {mean_hsv[2]:.2f}) | '
                    f'color={color_name} | cyl_inliers={inlier_count} | accepted={accepted}'
                )

def main():
    rclpy.init()
    node = CylinderDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
