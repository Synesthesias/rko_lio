<h1 align="center">
  RKO-LIO
</h1>
<h3 align="center">Robust LiDAR-Inertial Odometry Without Sensor-Specific Modelling</h3>

<div align="center">

[![IEEE RA-L](https://img.shields.io/badge/IEEE_RA--L-10.1109%2FLRA.2026.3685966-00629B.svg)](https://doi.org/10.1109/LRA.2026.3685966) [![arXiv](https://img.shields.io/badge/arXiv-2509.06593-b31b1b.svg)](https://arxiv.org/abs/2509.06593) [![GitHub License](https://img.shields.io/github/license/PRBonn/rko_lio)](/LICENSE) [![GitHub last commit](https://img.shields.io/github/last-commit/PRBonn/rko_lio)](/)

[![PyPI - Version](https://img.shields.io/pypi/v/rko_lio?color=blue)](https://pypi.org/project/rko-lio/)

</div>

<p align="center">
  <a href="https://www.youtube.com/watch?v=NNpzXdf9XmU" target="_blank">
    <img src="/docs/_static/example_multiple_platforms_shadow.png" alt="Visualization of odometry system running on data from four different platforms in four different environments" style="max-width: 100%; height: auto;" width="800"/>
  </a>
  <br />
  <em>Four different platforms, four different environments, one odometry system</em>
</p>

This repository contains a **ROS1 (catkin / roscpp)** port of RKO-LIO, intended for **Ubuntu 22.04** with the **ros-one** distribution.

## Quick Start (ROS1)

### Prerequisites

- Ubuntu 22.04
- ros-one (ROS 1) with `roscpp`, `rosbag`, `tf2`, `sensor_msgs`, `nav_msgs`
- A catkin workspace (e.g. catkin_tools)

System packages commonly required for building:

```bash
sudo apt install libeigen3-dev libtbb-dev
```

### Build

Clone this package into your catkin workspace `src/` directory, then:

```bash
cd /path/to/your/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build rko_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Build in **Release** mode (`-DCMAKE_BUILD_TYPE=Release`). Debug builds are significantly slower and may not keep up with 10 Hz LiDAR in real time.

If `rosdep` cannot resolve dependencies such as Sophus or `tsl-robin-map`, fetch them automatically at configure time:

```bash
catkin build rko_lio --cmake-args -DCMAKE_BUILD_TYPE=Release -DRKO_LIO_FETCH_CONTENT_DEPS=ON
```

### Online odometry

The launch file loads defaults from `config/ros_default.yaml`. Override sensor topics and frames as needed:

```bash
roslaunch rko_lio odometry.launch \
  lidar_topic:=/sensing/pandar \
  base_frame:=base_link \
  rviz:=false
```

`imu_topic`, `lidar_topic`, and `base_frame` must be set either via launch arguments or in the YAML config loaded by `config_file`. When using `ros_default.yaml`, `imu_topic` defaults to `/sensing/ins/imu`.

Example with an explicit config file:

```bash
roslaunch rko_lio odometry.launch \
  config_file:=$(rospack find rko_lio)/config/ros_default.yaml \
  lidar_topic:=/sensing/pandar \
  base_frame:=base_link
```

### Launch modes

`odometry.launch` supports three modes via the `mode` argument:

| `mode` | Node | Description |
|--------|------|-------------|
| `online` (default) | `online_node` | Threaded async registration from live topics |
| `online_imu_rate` | `online_imu_rate_node` | Sequential pipeline; optional IMU-rate odometry |
| `offline` | `offline_node` | Replay from a rosbag (`bag_path` required) |

Offline example:

```bash
roslaunch rko_lio odometry.launch \
  mode:=offline \
  bag_path:=/path/to/data.bag \
  lidar_topic:=/sensing/pandar \
  base_frame:=base_link \
  rviz:=false
```

### Published topics (defaults)

| Topic | Type | Description |
|-------|------|-------------|
| `/rko_lio/odom` | `nav_msgs/Odometry` | LiDAR-rate odometry |
| `rko_lio/lidar_acceleration` | custom | LiDAR-frame acceleration estimate |
| `/rko_lio/local_map` | `sensor_msgs/PointCloud2` | Local map (if `publish_local_map:=true`) |

TF: `odom` → `base_link` (child frame configurable via `base_frame`).

### Configuration

Parameters are loaded onto the private namespace `/rko_lio/` from YAML. See `config/ros_default.yaml` for defaults and available LIO tuning options (`voxel_size`, `max_range`, `deskew`, etc.).

To inspect all launch arguments:

```bash
roslaunch rko_lio odometry.launch --help
```

### Extrinsics and convention

The system needs extrinsics between IMU, LiDAR, and the base frame. If your TF tree defines `imu` → `base_link` and `pandar` → `base_link` (or equivalent), extrinsics are resolved automatically from TF.

Otherwise, set them in the config YAML:

```yaml
extrinsic_imu2base_quat_xyzw_xyz: [qx, qy, qz, qw, tx, ty, tz]
extrinsic_lidar2base_quat_xyzw_xyz: [qx, qy, qz, qw, tx, ty, tz]
```

Each value is a list of seven numbers: quaternion `(x, y, z, w)` followed by translation `(x, y, z)`. Example identity: `[0, 0, 0, 1, 0, 0, 0]`.

Transform naming follows `extrinsic_<from>2<to>`: a vector expressed in `<from>` is transformed into `<to>`.

### Notes on real-time operation

If registration is slightly slower than the LiDAR rate, you may see throttled warnings such as `Registration backlog: skipping N stale lidar scan(s).` This is expected under load; the node keeps the latest scan and continues publishing odometry. For best throughput, always build with `-DCMAKE_BUILD_TYPE=Release`.

## Python (optional)

The upstream project also provides a standalone Python CLI and PyPI package. That path is independent of the ROS1 nodes above.

```bash
pip install "rko_lio[all]"
rko_lio -v /path/to/data
```

See `rko_lio --help` and the [upstream Python docs](https://prbonn.github.io/rko_lio/pages/python.html).

## Citation

If you found this work useful, please consider leaving a star :star: on this repository and citing our paper ([RA-L](https://doi.org/10.1109/LRA.2026.3685966) | [arXiv](https://arxiv.org/abs/2509.06593)):

```bib
@article{malladi2026ral,
  author      = {M.V.R. Malladi and T. Guadagnino and L. Lobefaro and C. Stachniss},
  title       = {A Robust Approach for LiDAR-Inertial Odometry Without Sensor-Specific Modeling},
  journal     = {IEEE Robotics and Automation Letters},
  year        = {2026},
  volume      = {11},
  number      = {6},
  pages       = {7420--7427},
  doi         = {10.1109/LRA.2026.3685966},
}
```

The [paper supplementary material](https://prbonn.github.io/rko_lio/suppl.html) page collects extra figures and explanations pertaining to the same paper.

You can check out the branch `ral_submission` for the version of the code used for submission to RA-L. Please note that that branch is meant to be an as-is reproduction of the code used during submission and is not supported. The `master` and release versions are vastly improved, supported, and are the recommended way to use this system.

## Contributing

My goal with this project is to have a simple LiDAR-Inertial odometry system that can work with minimal friction, in the lines of ["it just works"](https://youtu.be/nVqcxarP9J4?si=LvTfV4m0NkNC62Tf&t=6). I gladly welcome any contribution or feedback you think would help in this direction. Performance improvements or bug fixes are of course always appreciated.

Thanks to the following contributors

<a href="https://github.com/PRBonn/rko_lio/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=PRBonn/rko_lio" />
</a>

## Acknowledgements

<details>
<summary>KISS-ICP, Kinematic-ICP, Bonxai, PlotJuggler, Rerun</summary>

This package is inspired by and would not be possible without the work of [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [Kinematic-ICP](https://github.com/PRBonn/kinematic-icp).
Additionally, we use and rely heavily on, either in the package itself or during development, [Bonxai](https://github.com/facontidavide/Bonxai), [PlotJuggler](https://github.com/facontidavide/PlotJuggler), [Rerun](https://github.com/rerun-io/rerun), and of course ROS itself.

A special mention goes out to [Rerun](https://rerun.io/) for providing an extremely easy-to-use but highly performative visualization system. Without this, I probably would not have made a python interface at all.

</details>

## License

This project is free software made available under the MIT license. For details, see the [LICENSE](LICENSE) file.
