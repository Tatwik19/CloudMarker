# CloudMarker
A ROS 2 perception pipeline for point-cloud cylinder detection, geometric fitting, and semantic color labeling with RViz-ready visualization.


ROS 2 package for detecting colored cylinders from an RGB-D point cloud.

The main node is `cylinder_detector`. It subscribes to `/oakd/points` by default, publishes debug point clouds for each stage of the pipeline, and publishes `/cylinder_markers` for the final cylinder detections.

---

## 1. What this repository does

This package runs a perception pipeline with five main stages:

1. crop the input cloud to a workspace box
2. downsample the cloud with a voxel grid
3. remove the dominant plane
4. cluster the remaining points into object candidates
5. fit cylinder models, classify their colors, and publish markers

Published debug topics:

- `/pipeline/stage0_box`
- `/pipeline/stage1_voxel`
- `/pipeline/stage2_nonplane`
- `/pipeline/stage3_candidates`
- `/pipeline/stage4_cylinders`
- `/cylinder_markers`

---

## 2. Before you start

Make sure you have:

- Ubuntu 24.04
- ROS 2 Jazzy
- a workspace at `~/ros2_ws`
- this package inside `~/ros2_ws/src/ass1_perception`
- the add bag files:
  - `~/ros2_ws/bag/ass1/rgbd_bag_0`
  - `~/ros2_ws/bag/ass1/rgbd_bag_1`
  - `~/ros2_ws/bag/ass1/rgbd_bag_2`

---

## 3. Repository layout


```text
ass1_perception/
|- ass1_perception/
|  `- cylinder_detector.py
|- launch/
|  |- rgbd_bag_0.launch.py
|  |- rgbd_bag_1.launch.py
|  `- rgbd_bag_2.launch.py
|- package.xml
|- resource/
|- setup.py
`- README.md
```


---

## 4. Build the package

Run these commands in a terminal:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ass1_perception --symlink-install
source ~/ros2_ws/install/setup.bash
```

If the build succeeds, you are ready to run the node.

---

## 5. Recommended way to run the repo

This is the easiest way for a beginner.

### Bag 0

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ass1_perception rgbd_bag_0.launch.py
```

### Bag 1

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ass1_perception rgbd_bag_1.launch.py
```

### Bag 2

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ass1_perception rgbd_bag_2.launch.py
```

Each launch file should start all of these together:

- `cylinder_detector`
- `ros2 bag play ... --loop --rate 0.5`
- `rviz2`
- `rqt_image_view`

---

## 6. Manual way to run the repo

Use this if your launch files are not ready yet.

Open **4 terminals**.

### Terminal 1: detector node

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ass1_perception cylinder_detector
```

### Terminal 2: play a bag

For bag 0:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 bag play ~/ros2_ws/bag/ass1/rgbd_bag_0 --loop --rate 0.5
```

For bag 1:

```bash
ros2 bag play ~/ros2_ws/bag/ass1/rgbd_bag_1 --loop --rate 0.5
```

For bag 2:

```bash
ros2 bag play ~/ros2_ws/bag/ass1/rgbd_bag_2 --loop --rate 0.5
```

### Terminal 3: image viewer

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view --clear-config --ros-args -p image_transport:=compressed
```

### Terminal 4: RViz

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

---

## 7. RViz setup the first time

When RViz opens, do this:

### Step 1: set the fixed frame

Set **Fixed Frame** to:

```text
oakd_rgb_camera_optical_frame
```

### Step 2: add debug clouds

Add **PointCloud2** displays for these topics:

```text
/pipeline/stage0_box
/pipeline/stage1_voxel
/pipeline/stage2_nonplane
/pipeline/stage3_candidates
/pipeline/stage4_cylinders
```

### Step 3: add final markers

Add a **MarkerArray** display for:

```text
/cylinder_markers
```

### Step 4: choose an image topic

In `rqt_image_view`, select the bag image topic from the dropdown menu.

---

## 8. What each stage means

### `/pipeline/stage0_box`

This is the input cloud after workspace cropping.

Current values:

```python
self.box_min = np.array([-1.0, -0.6, 0.2], dtype=np.float64)
self.box_max = np.array([1.0, 0.6, 5.0], dtype=np.float64)
```

Change these when:

- cylinders are missing because they are outside the crop box
- too much background is getting through
- clutter is entering from one side only

### `/pipeline/stage1_voxel`

This is the cropped cloud after voxel downsampling.

Current value:

```python
self.voxel_size = 0.02
```

How to tune it:

- smaller value -> more detail, more points, slower, noisier
- larger value -> smoother, faster, but cylinders may become too sparse

Good first tests:

- try `0.01` to `0.015` if cylinders look thin or broken
- try `0.025` to `0.03` if the cloud is too dense or noisy

### `/pipeline/stage2_nonplane`

This is the cloud after plane removal.

Current values:

```python
self.floor_dist = 0.02
self.target_normal = np.array([0.0, 1.0, 0.0], dtype=np.float64)
self.normal_thresh = 0.85
self.plane_ransac_iters = 500
```

What these do:

- `floor_dist`: plane inlier distance threshold
- `target_normal`: expected plane normal in the camera frame
- `normal_thresh`: how closely the plane normal must match `target_normal`
- `plane_ransac_iters`: number of RANSAC iterations

Typical tuning:

- floor still visible -> increase `floor_dist` or `plane_ransac_iters`, or lower `normal_thresh` slightly
- wall removed by mistake -> increase `normal_thresh` or correct `target_normal`
- cylinder bases disappear -> reduce `floor_dist`

### `/pipeline/stage3_candidates`

This is the non-plane cloud after Euclidean clustering.

Current values:

```python
self.cluster_dist = 0.05
self.min_cluster_size = 80
self.max_cluster_size_pts = 3500
```

What these do:

- `cluster_dist`: distance for connecting nearby points into one cluster
- `min_cluster_size`: reject very small clusters
- `max_cluster_size_pts`: reject oversized clusters

Typical tuning:

- two objects merge into one cluster -> reduce `cluster_dist`
- one cylinder breaks into multiple clusters -> increase `cluster_dist` a little
- too many tiny junk clusters -> increase `min_cluster_size`
- large background blobs survive -> reduce `max_cluster_size_pts`

### `/pipeline/stage4_cylinders`

This is the final cylinder-only cloud.

Current values:

```python
self.cyl_radius = 0.055
self.max_cylinders = 3
self.normal_k = 10
self.cyl_ransac_iters = 750
self.cyl_dist_thresh = 0.02
self.axis_align_thresh = 0.75
self.center_tol = 0.08
self.normal_perp_thresh = 0.45
self.min_cylinder_inliers = 75
self.min_cylinder_inlier_ratio = 0.12
```

What these do:

- `cyl_radius`: expected cylinder radius
- `max_cylinders`: cap on final detections
- `normal_k`: number of neighbors used for normal estimation
- `cyl_ransac_iters`: cylinder RANSAC iterations
- `cyl_dist_thresh`: allowed radial error from the cylinder surface
- `axis_align_thresh`: how well the cylinder axis must align with the expected axis
- `center_tol`: tolerance between center estimates from sampled point-normal pairs
- `normal_perp_thresh`: normals should be roughly perpendicular to the cylinder axis
- `min_cylinder_inliers`: minimum number of inliers needed to accept a cylinder
- `min_cylinder_inlier_ratio`: minimum fraction of inliers needed to accept a cylinder

Typical tuning:

- real cylinders are missed -> increase `cyl_ransac_iters`, increase `cyl_dist_thresh`, reduce `min_cylinder_inliers`, or reduce `min_cylinder_inlier_ratio`
- false cylinders appear -> reduce `cyl_dist_thresh`, increase `axis_align_thresh`, reduce `normal_perp_thresh`, or increase `min_cylinder_inliers`
- cylinder shape looks correct but still fails -> check `cyl_radius` first

---

## 9. Color classification

Cylinder geometry and cylinder color are handled separately.

The current color thresholds are:

```python
if v < 0.12 or s < 0.05:
    return 'unknown', ...

if 270.0 <= h < 350.0 and s >= 0.1 and v >= 0.1:
    return 'pink', ...
if h >= 350.0 or h < 20.0:
    return 'red', ...
if 70.0 <= h < 170.0:
    return 'green', ...
if 190.0 <= h < 270.0:
    return 'blue', ...
```

Typical tuning:

- too many `unknown` labels -> lower the `s` or `v` thresholds
- pink gets called red -> widen or shift the pink hue range
- blue and green get confused -> tighten those hue bands

---

## 10. Helpful runtime checks

In the terminal running `cylinder_detector`, the node prints summary logs every few messages.

These logs show:

- raw point count
- cropped point count
- voxelized point count
- plane inlier count
- number of clusters
- number of accepted cylinders
- best cylinder inlier count

It also prints per-cluster statistics such as:

- point count
- centroid
- bounding-box size
- mean RGB
- mean HSV
- predicted color
- cylinder inlier count
- whether the cluster was accepted

Use these logs together with RViz.

---

## 12. Troubleshooting

### `ros2 launch` cannot find the launch file

Usually this means one of these is still wrong:

- `launch/*.launch.py` is missing
- `setup.py` does not install the launch files
- you forgot to rebuild the workspace
- you forgot to re-source `~/ros2_ws/install/setup.bash`

### RViz opens but no cloud appears

Check these first:

- bag playback is actually running
- Fixed Frame is `oakd_rgb_camera_optical_frame`
- the detector is subscribed to the correct topic
- the bag contains `/oakd/points`

### `rqt_image_view` is blank

Try these:

- manually choose the image topic in the dropdown
- remove the compressed transport argument if your bag does not provide that transport

Example raw-image command:

```bash
ros2 run rqt_image_view rqt_image_view
```

### The detector misses obvious cylinders

Try this order:

1. verify the cylinder is inside `stage0_box`
2. verify it survives plane removal in `stage2_nonplane`
3. verify it appears as one clean cluster in `stage3_candidates`
4. check `cyl_radius` before changing many other cylinder parameters


This order saves time because early-stage mistakes affect every later stage.

---

## 14. Quick command reference

### Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ass1_perception --symlink-install
source ~/ros2_ws/install/setup.bash
```

### Run bags with launch

```bash
ros2 launch ass1_perception rgbd_bag_0.launch.py
```

```bash
ros2 launch ass1_perception rgbd_bag_1.launch.py
```

```bash
ros2 launch ass1_perception rgbd_bag_2.launch.py
```

### Run the node only

```bash
ros2 run ass1_perception cylinder_detector
```

### Play bag2 manually

```bash
ros2 bag play ~/ros2_ws/bag/ass1/rgbd_bag_0 --loop --rate 0.5
```

```bash
ros2 bag play ~/ros2_ws/bag/ass1/rgbd_bag_1 --loop --rate 0.5
```

```bash
ros2 bag play ~/ros2_ws/bag/ass1/rgbd_bag_2 --loop --rate 0.5
```

### Open image viewer

```bash
ros2 run rqt_image_view rqt_image_view --clear-config --ros-args -p image_transport:=compressed
```

### Open RViz

```bash
rviz2
```
