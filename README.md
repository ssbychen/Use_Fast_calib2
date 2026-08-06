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

- CMake >= 3.14
- C++17 compiler (GCC 7+ / Clang 5+)
- PCL >= 1.8
- OpenCV >= 4.0
- Eigen3
- yaml-cpp

Install on Ubuntu:

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    libyaml-cpp-dev
```

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

## 4. Build

```bash
git clone https://github.com/ssbychen/Use_Fast_calib2.git
cd Use_Fast_calib2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

This produces the following executables inside `build/`:

| Executable | Description |
|---|---|
| `fast_calib2` | Single-scene LiDAR-camera calibration |
| `multi_fast_calib` | Multi-scene joint calibration |
| `lidar_center_test` | Standalone LiDAR annulus center test |
| `common_lib_test` | Unit tests for common functions |

## 5. Run Examples

Prepare static acquisition data in the `calib_data` folder (Download the example data from [Google Drive](https://drive.google.com/drive/folders/1VnMCsGj3Gat7dxe6IION0SfS7jYNMw1g?usp=sharing)):

- `.pcd` point cloud file
- `.png` / `.bmp` / `.jpg` corresponding image

Edit `config/qr_params.yaml` to match your camera intrinsics and target dimensions. Describe the LiDAR mounting:

```yaml
lidar_forward_axis: "+x"
lidar_up_axis: "+z"
```

The axis values must be signed, perpendicular axes such as `+x` and `-y`.

Run single-scene calibration:

```bash
./build/fast_calib2 \
    --pcd   calib_data/avia/mid.pcd \
    --image calib_data/avia/mid.png \
    --output output/ \
    --config config/qr_params.yaml
```

After collecting at least three scenes, run multi-scene joint calibration:

```bash
./build/multi_fast_calib \
    --config config/qr_params.yaml \
    --input  output/circle_center_record.txt \
    --output output/multi_calib_result.yaml
```

Typical multi-scene target placement:

<p align="center">
  <img src="./pics/multi-scene.jpg" width="100%">
  <font color=#a0a0a0 size=2>Placement of the calibration target for multi-scene data collection: (a) facing forward, (b) oriented to the right, (c) oriented to the left.</font>
</p>

## 6. Standalone LiDAR Center Extraction Test

<details>
<summary>Show Unit Test Usage</summary>

The repository also provides a LiDAR-only test tool for checking annulus center extraction before running full camera-LiDAR calibration.

Run solid-state LiDAR data:

```bash
./build/lidar_center_test \
    --pcd    calib_data/avia/mid.pcd \
    --config config/qr_params.yaml \
    --type   solid \
    --output output/
```

Run mechanical LiDAR data:

```bash
./build/lidar_center_test \
    --pcd    calib_data/hesai-jt128/mid.pcd \
    --config config/qr_params.yaml \
    --type   mech \
    --output output/
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
