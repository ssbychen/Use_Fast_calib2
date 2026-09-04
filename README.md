# FAST-Calib2

## LiDAR-Camera Extrinsic Calibration with Reflective Annular Targets

FAST-Calib2 extends [FAST-Calib](https://github.com/hku-mars/FAST-Calib) to LiDAR-camera modules that were previously hard to calibrate due to **low-quality point clouds**. With a custom-designed reflective annular calibration target, it enables robust center extraction on **large-spot solid-state and mechanical LiDARs**, including Mid360, Avia, Ouster, XT32, JT128, Airy, E1R, and Adaps Photonics Spad LiDAR.

**Key highlights include:**

1. A self-designed 3D reflective annular calibration target that avoids center extraction errors caused by hole-edge inflation and bleeding artifacts in previous circular-hole calibration boards.
2. A robust concentric-circle fitting method that uses the fixed inner and outer annulus radii as geometric constraints.
3. Automatic calibration board ROI extraction without manual pass-through tuning.
4. Geometry and radius quality checks for extracted annulus centers.
5. Single-scene and multi-scene LiDAR-camera extrinsic calibration without initial extrinsic parameters.

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at zhengcr@connect.hku.hk.

<p align="center">
  <img src="./pics/cover.jpg" width="100%">
  <font color=#a0a0a0 size=2>Mid360 calibration example.</font>
</p>

## 1. Prerequisites

PCL>=1.8, OpenCV>=4.0.

## 2. Calibration Target

FAST-Calib2 uses four reflective annuli and four visual markers on one board. The annuli are used by LiDAR center extraction, while the visual markers are used by the camera pipeline.

Materials:

- Board: PVC
- Reflective annulus stickers: 3M engineering-grade reflective film

<p align="center">
  <img src="./pics/FAST-Calib2-board.png" width="100%">
  <font color=#a0a0a0 size=2>Reflective annular calibration target and annotated dimensions.</font>
</p>

DIY Calibration Target Tips:

1. Fabricate the board based on the schematic. Ensure a minimum thickness of 1 cm to avoid bending.
2. Apply reflective annulus stickers to the designated ring positions on the fabricated board.

## 3. Method Overview

Both LiDAR pipelines first **locate the calibration board automatically**, fit the board plane, and align the plane to `Z=0`. Center extraction is then performed in the aligned board frame.

Solid-state LiDAR pipeline:

1. Extract high-reflectivity annulus points on the fitted board plane.
2. Cluster the extracted annulus points.
3. Fit robust single circles as the default center estimate.
4. Optionally extract annulus boundary points and fit fixed inner/outer radius concentric circles.
5. Select the best result by checking four-center geometry consistency against the known target geometry.

Mechanical LiDAR pipeline:

1. Use LiDAR ring order within each scan to find intensity transition points on the annulus boundary.
2. Try both interpolated boundary points and high-reflectivity-side boundary points.
3. Cluster the extracted boundary points.
4. Fit fixed inner/outer radius concentric circles.
5. Select the best result by checking four-center geometry consistency against the known target geometry.

The final quality checks include center-to-center geometry error and annulus radius consistency.

### Circle-hole board mode

The repository also supports a `circle_hole_board` mode for boards whose LiDAR-side features are circular holes instead of reflective annuli. In this mode:

- the **camera side still uses the existing QR / ArUco board pose estimation** to recover the board pose;
- the **LiDAR side reuses the existing circle fitting path** on hole-boundary points extracted from the fitted board plane;
- the solver input remains the same point-correspondence format, so no separate backend is required.

Configure it in `config/qr_params.yaml`:

```yaml
target_type: circle_hole_board
hole_rows: 3
hole_cols: 4
hole_spacing_x: 0.15
hole_spacing_y: 0.15
hole_diameter: 0.07
hole_radius_tolerance: 0.02
hole_max_fit_error: 0.02
hole_min_edge_points: 30
```

Parameter meanings:

- `target_type`: `qr` (default) or `circle_hole_board`
- `hole_rows`, `hole_cols`: number of hole centers to extract
- `hole_spacing_x`, `hole_spacing_y`: center-to-center spacing in metres
- `hole_diameter`: physical hole diameter in metres
- `hole_radius_tolerance`: allowable fitted-radius error in metres
- `hole_max_fit_error`: maximum mean absolute circle-fit residual in metres
- `hole_min_edge_points`: minimum boundary points required per hole cluster

MID360 suggestions:

- start with a **manual board ROI** (`use_auto_lidar_roi: false` + pass-through bounds), because the existing auto-ROI logic is tuned for reflective annuli;
- keep the board at roughly **2-6 m** and cover several viewing angles;
- ensure each hole boundary has enough returns before calibration, otherwise the frame will be skipped with a log message.

### Standalone cropped-board circle extraction

If you already manually cropped a single calibration-board point cloud and only want ordered circle centers, build and run `circle_center_extract`:

```bash
rosrun fast_calib circle_center_extract input_board.pcd output_centers.txt 3 4 0.015 20
```

Arguments:

- `input_board.pcd`: cropped point cloud containing one board
- `output_centers.txt`: text file written in row/column order
- `3 4`: board row and column counts
- `0.015`: Euclidean clustering tolerance in metres for per-hole boundary clusters
- `20` (optional): minimum cluster size

Output rows use the format:

```txt
row col center_x center_y center_z radius mean_abs_error
```

## 4. Run Examples

Prepare static acquisition data in the `calib_data` folder (Download the example data from [Google Drive](https://drive.google.com/drive/folders/1VnMCsGj3Gat7dxe6IION0SfS7jYNMw1g?usp=sharing)):

- rosbag containing point cloud messages
- corresponding image

Describe the LiDAR mounting in `config/qr_params.yaml`:

```yaml
lidar_forward_axis: "+x"
lidar_up_axis: "+z"
```

The axis values must be signed, perpendicular axes such as `+x` and `-y`.

Minimal circle-hole example:

```yaml
target_type: circle_hole_board
use_auto_lidar_roi: false
hole_rows: 3
hole_cols: 4
hole_spacing_x: 0.15
hole_spacing_y: 0.15
hole_diameter: 0.07
```

Run single-scene calibration:

```bash
roslaunch fast_calib calib.launch
```

After collecting at least three scenes, run multi-scene joint calibration:

```bash
roslaunch fast_calib multi_calib.launch
```

Typical multi-scene target placement:

<p align="center">
  <img src="./pics/multi-scene.jpg" width="100%">
  <font color=#a0a0a0 size=2>Placement of the calibration target for multi-scene data collection: (a) facing forward, (b) oriented to the right, (c) oriented to the left.</font>
</p>

## 5. Standalone LiDAR Center Extraction Test

<details>
<summary>Show Unit Test Usage</summary>

The repository also provides a LiDAR-only test tool for checking annulus center extraction before running full camera-LiDAR calibration.

Load parameters:

```bash
rosparam load config/qr_params.yaml /
rosparam set /output_path "$(rospack find fast_calib)/output"
```

Run solid-state LiDAR data:

```bash
rosrun fast_calib lidar_center_test calib_data/avia/left.bag /livox/lidar solid
rosrun fast_calib lidar_center_test calib_data/avia/mid.bag /livox/lidar solid
rosrun fast_calib lidar_center_test calib_data/avia/right.bag /livox/lidar solid
```

Run mechanical LiDAR data:

```bash
rosrun fast_calib lidar_center_test calib_data/hesai-jt128/left.bag /lidar_points mech
rosrun fast_calib lidar_center_test calib_data/hesai-jt128/mid.bag /lidar_points mech
rosrun fast_calib lidar_center_test calib_data/hesai-jt128/right.bag /lidar_points mech
```

The test tool writes:

- `*_centers.txt`: extracted annulus center coordinates
- `*_debug_cloud.pcd`: board point cloud, annulus points, boundary points, and center markers for visualization

Debug PCD colors:

- Board points: intensity color map
- Annulus points: green
- Solid-LiDAR boundary points: red
- Centers: white spheres

</details>
