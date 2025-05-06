
# Blue Robotics Sonar ROS 2 Package

This ROS2 package provides nodes for interfacing with and visualizing data from Blue Robotics [**Ping1D**](https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/) and [**Ping360**](https://bluerobotics.com/store/sonars/imaging-sonars/ping360-sonar-r1-rp/) sonars. It includes four nodes:

- `ping1d`: Publishes sonar data from the Ping1D device.
- `ping1d_imager`: Converts Ping1D data into real-time image visualizations.
- `ping360`: Publishes sonar data from the Ping360 device.
- `ping360_imager`: Converts Ping360 data into real-time radial image visualizations.

---

## Table of Contents

- [Installation](#installation)
- [Launch Files](#launch-files)
  - [ping1d.launch.py](#ping1dlaunchpy)
  - [ping360.launch.py](#ping360launchpy)
- [Nodes and Usage](#nodes-and-usage)
  - [ping1d](#ping1d)
  - [ping1d_imager](#ping1d_imager)
  - [ping360](#ping360)
  - [ping360_imager](#ping360_imager)
- [License](#license)

---

## Installation

Install [Blue Robotics ping-python](https://github.com/bluerobotics/ping-python/tree/deployment) package:
```bash
git clone --single-branch --branch deployment https://github.com/bluerobotics/ping-python.git
cd ping-python
python3 setup.py install --user
```

Install [Blue Robotics Sonar](https://github.com/itskalvik/bluerobotics_sonar) ROS2 package:

```bash
cd ~/ros2_ws/src
git clone https://github.com/itskalvik/bluerobotics_sonar.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Launch Files

### `ping1d.launch.py`

Launches the [ping1d](#ping1d) node to publish sonar data.

**Published Topic**: `/sonar/ping1d/data` (`bluerobotics_sonar_msgs/SonarPing1D`)

**Run Example**:

```bash
ros2 launch bluerobotics_sonar ping1d.launch.py
```

### `ping360.launch.py`

Launches the [ping360](#ping360) node to publish sonar data.

**Published Topics**:  `/sonar/ping360/data` (`bluerobotics_sonar_msgs/Image`)

**Run Example**:

```bash
ros2 launch bluerobotics_sonar ping360.launch.py
```

---

## Nodes and Usage

### `ping1d`

**Description**: Publishes sonar profiles from the Blue Robotics Ping1D.

**Published Topic**: `/sonar/ping1d/data` (`bluerobotics_sonar_msgs/SonarPing1D`)

**Parameters**: The parameters can be updated while running the node.

| Name             | Type   | Default         | Description                                  |
|------------------|--------|-----------------|----------------------------------------------|
| `device`         | string | `/dev/ttyUSB0`  | Serial port or UDP IP address                |
| `baudrate`       | int    | `115200`        | Baudrate for serial connection               |
| `gain_setting`   | int    | `0`             | Gain level [0–6]                             |
| `mode_auto`      | int    | `0`             | Manual (0) or auto (1) mode                  |
| `ping_enable`    | bool   | `true`          | Whether sonar is enabled                     |
| `ping_interval`  | int    | `100`           | Interval between pings (ms)                  |
| `scan_start`     | float  | `0.0`           | Minimum range (m) [0-99]                     |
| `scan_length`    | float  | `1.0`           | Maximum range (m) [0.3-100]                  |
| `speed_of_sound` | int    | `1500`          | Speed of sound in water (m/s)                |
| `topic`          | string | `/sonar/ping1d/data` | Output topic                            |
| `frame_id`       | string | `ping1d`        | TF frame ID                                  |

**Run Example**:
```bash
ros2 run bluerobotics_sonar ping1d
```

---

### `ping1d_imager`

**Description**: Subscribes to Ping1D data and publishes a waterfall scan image.

**Published Topic**: `/sonar/ping1d/image` (`sensor_msgs/Image`)

**Subscribed Topics**: `/sonar/ping1d/data`

**Parameters**: The parameters can be updated while running the node.

| Name           | Type   | Default               | Description                          |
|----------------|--------|-----------------------|--------------------------------------|
| `data_topic`   | string | `/sonar/ping1d/data`  | Input sonar data topic               |
| `image_topic`  | string | `/sonar/ping1d/image` | Output image topic                   |
| `image_length` | int    | `200`                 | Width (in samples) of scroll image   |
| `bag_file`     | str    |                       | Optional path to an input ros2 bag file with sonar data |
| `video_file`   | str    |                       | Optional path to an output mp4 video file |

**Run Example**:
```bash
ros2 run bluerobotics_sonar ping1d_imager
```

---

### `ping360`

**Description**: Publishes sonar profiles from the Blue Robotics Ping360.

**Published Topic**: `/sonar/ping360/data` (`bluerobotics_sonar_msgs/SonarPing360`)

**Parameters**: The parameters can be updated while running the node.

| Name              | Type   | Default            | Description                                  |
|-------------------|--------|--------------------|----------------------------------------------|
| `device`          | string | `/dev/ttyUSB0`     | Serial port or UDP IP address                |
| `baudrate`        | int    | `115200`           | Baudrate                                     |
| `mode`            | int    | `1`                | Scan mode (always 1)                         |
| `gain_setting`    | int    | `0`                | Gain level [0–6]                             |
| `transmit_frequency` | int | `750`              | Transmit frequency (kHz)                     |
| `start_angle`     | int    | `0`                | Starting angle (Grads)                       |
| `stop_angle`      | int    | `399`              | Stopping angle (Grads)                       |
| `num_steps`       | int    | `1`                | Step size between angles                     |
| `delay`           | int    | `0`                | An additional delay between successive transmit pulses [0-100 ms] |
| `range`           | float  | `1.0`              | Scan range in meters [0.75-50]               |
| `motor_off`       | bool   | `false`            | If true, stops sonar                         |
| `scan_threshold`  | int    | `100`              | Threshold for peak detection                 |
| `offset`          | int    | `120`              | Number of samples to ignore in each sonar profile before peak detection |
| `speed_of_sound`  | int    | `1500`             | Speed of sound in water (m/s)                |
| `topic`           | string | `/sonar/ping360/data` | Output topic                              |
| `frame_id`        | string | `ping360`          | TF frame ID                                  |

**Run Example**:
```bash
ros2 run bluerobotics_sonar ping360
```

---

### `ping360_imager`

**Description**: Converts Ping360 sonar data into a polar image and publishes it.

**Published Topic**: `/sonar/ping360/image` (`sensor_msgs/Image`)

**Subscribed Topics**: `/sonar/ping360/data`

**Parameters**: The parameters can be updated while running the node.

| Name         | Type   | Default              | Description                                 |
|--------------|--------|----------------------|---------------------------------------------|
| `data_topic` | string | `/sonar/ping360/data` | Input sonar data topic                     |
| `image_topic`| string | `/sonar/ping360/image`| Output image topic                         |
| `rotation`   | float  | `0.0`                 | Optional image rotation to match sonar mount |
| `bag_file`   | str    |                       | Optional path to an input ros2 bag file with sonar data |
| `video_file` | str    |                       | Optional path to an output mp4 video file |

**Run Example**:
```bash
ros2 run bluerobotics_sonar ping360_imager
```

---

## License

This package is licensed under the MIT License. See the top of each file or [LICENSE](LICENSE) for details.
