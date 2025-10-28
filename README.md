# Blue Robotics Sonar ROS 2 Package

This ROS 2 package provides drivers for the Blue Robotics **Ping1D** altimeter and **Ping360** scanning sonar. It includes nodes for interfacing with the hardware and visualizing the data.

<p align="center">
<img src=".assets/ping1d.gif" alt="Ping1D" width="49%"/> <img src=".assets/ping360.gif" alt="Ping360" width="49%"/>
</p>

---

## Table of Contents
- [Overview](#overview)
- [Requirements & Installation](#requirements--installation)
- [Launch Files](#launch-files)
  - [ping1d.launch.py](#ping1dlaunchpy)
  - [ping360.launch.py](#ping360launchpy)
- [Nodes](#nodes)
  - [ping1d](#ping1d)
  - [ping1d_imager](#ping1d_imager)
  - [ping360](#ping360)
  - [ping360_imager](#ping360_imager)
- [Notes & Tips](#notes--tips)
- [License](#license)

---

## Overview

This package contains four main nodes:

- **`ping1d`** and **`ping360`** — Driver nodes that interface with the hardware and publish raw sonar data as ROS 2 messages.
- **`ping1d_imager`** and **`ping360_imager`** — Processing nodes that subscribe to the raw sonar messages and render them to images. Each imager can either publish live images on a topic or export a video when consuming a ROS 2 bag file.

---

## Requirements & Installation

Install the [Blue Robotics `ping-python`] library first, then clone and build this package with `colcon`.

```bash
# Install ping-python (tested with 'deployment' branch)
git clone https://github.com/bluerobotics/ping-python.git -b deployment
cd ping-python
python3 setup.py install --user

# Install bluerobotics_sonar ROS 2 package
cd ~/ros2_ws/src
git clone https://github.com/itskalvik/bluerobotics_sonar.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

> If building in an environment without `rosdep`, install missing dependencies manually (OpenCV, `cv_bridge`, `rosbag2_py`, etc.).

---

## Launch Files

### `ping1d.launch.py`

Launches the [`ping1d`](#ping1d) node to publish sonar data. You can also pass parameters here.

**Published Topic**: `/sonar/ping1d/data` (`bluerobotics_sonar_msgs/SonarPing1D`)

**Run Example**:

```bash
ros2 launch bluerobotics_sonar ping1d.launch.py
```

Adjust maximum range while running:

```bash
ros2 param set /ping1d scan_length 10.0
```

---

### `ping360.launch.py`

Launches the [`ping360`](#ping360) node to publish sonar data. You can also pass parameters here.

**Published Topic**: `/sonar/ping360/data` (`bluerobotics_sonar_msgs/SonarPing360`)

**Run Example**:

```bash
ros2 launch bluerobotics_sonar ping360.launch.py
```

Change the maximum range while running:

```bash
ros2 param set /ping360 range 10.0
```

---

## Nodes

### `ping1d`

**Description**: Publishes sonar profiles from a Blue Robotics Ping1D.

**Published Topic**: `/sonar/ping1d/data` (`bluerobotics_sonar_msgs/SonarPing1D`)

**Runtime-Configurable Parameters**

| Name | Type | Default | Description |
|---|---:|:---:|---|
| `device` | string | `/dev/ttyUSB0` | Serial device path **or** IPv4 address for UDP. If an IPv4 address is provided, the node connects via UDP. |
| `baudrate` | int | `115200` | Serial baudrate. When `device` is an IPv4 address, this value is used as the UDP **port**. |
| `gain_setting` | int | `0` | Receiver gain level \[0–6\]. |
| `mode_auto` | int | `0` | 0 = manual; 1 = auto mode. |
| `ping_enable` | bool | `true` | Enable/disable pinging (publishing pauses when disabled). |
| `pings_per_second` | int | `30` | Desired ping rate \[1–50\]. When set, it computes `ping_interval = round(1000 / pings_per_second)`. |
| `ping_interval` | int | `-1` | Interval between pings in **ms**. `-1` means: derive from `pings_per_second`. |
| `scan_start` | double | `0.0` | Minimum range in **meters**. |
| `scan_length` | double | `1.0` | Maximum range in **meters** (typical limits ~0.3–100 m depending on conditions). |
| `speed_of_sound` | int | `1500` | Speed of sound in **m/s** used for distance conversions. |
| `topic` | string | `/sonar/ping1d/data` | Output topic for sonar profiles. |
| `frame_id` | string | `ping1d` | Frame ID set in message headers. |

**Run Example**

```bash
ros2 run bluerobotics_sonar ping1d
```

---

### `ping1d_imager`

**Description**: Subscribes to Ping1D data and publishes a scrolling “waterfall” image, or exports a video when reading from a ROS 2 bag file.

**Published Topic**: `/sonar/ping1d/image` (`sensor_msgs/Image`)

**Subscribed Topics**: `/sonar/ping1d/data`

**Runtime-Configurable Parameters**

| Name | Type | Default | Description |
|---|---:|:---:|---|
| `data_topic` | string | `/sonar/ping1d/data` | Input sonar data topic. |
| `image_topic` | string | `/sonar/ping1d/image` | Output image topic (used when not exporting video). |
| `image_length` | int | `200` | Width (number of columns) of the scrolling waterfall image. |
| `bag_file` | string | *(empty)* | If provided, the node reads sonar messages from this ROS 2 bag. |
| `video_file` | string | *(empty)* | If set (or when `bag_file` is provided), write an MP4 video to this path. Default is `ping1d_sonar.mp4` when reading from a bag. |

**Run Example**

```bash
ros2 run bluerobotics_sonar ping1d_imager --ros-args -p data_topic:=/sonar/ping1d/data
```

---

### `ping360`

**Description**: Publishes sonar profiles from a Blue Robotics Ping360. Internally, range-related parameters are auto-tuned for sane operation while scanning.

**Published Topic**: `/sonar/ping360/data` (`bluerobotics_sonar_msgs/SonarPing360`)

**Runtime-Configurable Parameters**

| Name | Type | Default | Description |
|---|---:|:---:|---|
| `device` | string | `/dev/ttyUSB0` | Serial device path **or** IPv4 address for UDP. If an IPv4 address is provided, the node connects via UDP. |
| `baudrate` | int | `115200` | Serial baudrate. When `device` is an IPv4 address, this value is used as the UDP **port**. |
| `mode` | int | `1` | Scanning mode for auto-transmit (fixed to 1 by firmware). |
| `gain_setting` | int | `0` | Receiver gain level \[0–6\]. |
| `transmit_frequency` | int | `750` | Transmit frequency in **kHz**. |
| `start_angle` | int | `0` | Start angle in **grads** \[0–399\]; 400 grads = 360°. |
| `stop_angle` | int | `399` | Stop angle in **grads** \[0–399\]. |
| `num_steps` | int | `1` | Step size between successive angles (in grads). |
| `delay` | int | `0` | Extra inter-ping delay in **ms** (0–100). |
| `range` | double | `1.0` | Scan range in **meters** (typical limits ~0.75–50 m). |
| `motor_off` | bool | `false` | If `true`, stop the motor/scan. |
| `speed_of_sound` | int | `1500` | Speed of sound in **m/s** used for range/time-of-flight calculations. |
| `scan_threshold` | int | `150` | Peak detection threshold for estimating distance to nearest return. |
| `offset` | int | `20` | Number of initial bins to ignore per profile (ring-down/near-field). |
| `window_size` | int | `15` | Window used by the edge/peak detector for distance estimation. |
| `topic` | string | `/sonar/ping360/data` | Output topic for sonar profiles. |
| `frame_id` | string | `ping360` | Frame ID set in message headers. |

**Run Example**

```bash
ros2 run bluerobotics_sonar ping360
```

---

### `ping360_imager`

**Description**: Converts Ping360 sonar profiles into a polar image. Can publish live images or export a video when reading from a ROS 2 bag file. Use `rotation` to compensate for the physical mounting orientation.

**Published Topic**: `/sonar/ping360/image` (`sensor_msgs/Image`)

**Subscribed Topics**: `/sonar/ping360/data`

**Runtime-Configurable Parameters**

| Name | Type | Default | Description |
|---|---:|:---:|---|
| `data_topic` | string | `/sonar/ping360/data` | Input sonar data topic. |
| `image_topic` | string | `/sonar/ping360/image` | Output image topic (used when not exporting video). |
| `rotation` | double | `0.0` | Optional image rotation (±90° increments) to match mounting. |
| `bag_file` | string | *(empty)* | If provided, the node reads sonar messages from this ROS 2 bag. |
| `video_file` | string | *(empty)* | If set (or when `bag_file` is provided), write an MP4 video to this path. Default is `ping360_sonar.mp4` when reading from a bag. |

**Run Example**

```bash
ros2 run bluerobotics_sonar ping360_imager --ros-args -p data_topic:=/sonar/ping360/data
```

---

## Notes & Tips

- **Serial vs. UDP**: If `device` looks like an IPv4 address (e.g., `192.168.2.2`), the node opens a UDP connection; otherwise it opens a serial connection. In UDP mode the `baudrate` parameter is interpreted as the **port**.
- **Ping1D rate control**: Prefer setting `pings_per_second`; the node computes `ping_interval` automatically and bounds rates to a safe 1–50 Hz.
- **Angles in grads (Ping360)**: The hardware reports/accepts angles in grads (0–399), where **1 grad = 0.9°**. The imagers convert this for display overlays.
- **Distance estimation (Ping360)**: The driver includes a simple edge/peak-based range estimator with a 1D filter for smoothing. Tune `offset`, `window_size`, and `scan_threshold` for your environment.
- **QoS**: All publishers/subscribers use the ROS 2 Sensor Data QoS and allow QoS overrides via the standard mechanisms.

---

## License

MIT License. See the top of each file or [LICENSE](LICENSE) for details.
