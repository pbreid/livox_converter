# Livox Converter

## Overview

The Livox Converter is a ROS package designed to convert Livox LiDAR custom messages (`livox_ros_driver/CustomMsg`) to standard ROS `sensor_msgs/PointCloud2` messages. This package is particularly useful for integrating Livox LiDAR sensors with existing ROS ecosystems that expect standard point cloud data.

## Features

- Converts `livox_ros_driver/CustomMsg` to `sensor_msgs/PointCloud2`
- Dynamically manages topic subscriptions and advertisements based on availability and demand
- Efficient processing to minimize CPU usage when no subscribers are present

## Requirements

- ROS (tested on Noetic, but should work on earlier versions)
- Livox ROS Driver
- PCL (Point Cloud Library)

## Installation

1. Clone this repository into your catkin workspace:
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/pbreid/livox_converter.git
   ```

2. Install dependencies:
   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:
   ```
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage

1. Launch the Livox Converter node:
   ```
   roslaunch livox_converter livox_converter.launch
   ```

2. (Optional) Modify the input and output topics in the launch file if needed.

## Node Details

### Subscribed Topics

- `/livox/lidar` (default, configurable) of type `livox_ros_driver/CustomMsg`

### Published Topics

- `/livox/pointcloud2` (default, configurable) of type `sensor_msgs/PointCloud2`

### Parameters

- `input_topic` (string, default: "/livox/lidar"): The input topic for Livox custom messages
- `output_topic` (string, default: "/livox/pointcloud2"): The output topic for converted PointCloud2 messages

## Behavior

1. The node checks for the availability of the input topic (`livox_ros_driver/CustomMsg`).
2. When the input topic is available, it advertises the output topic (`sensor_msgs/PointCloud2`).
3. The node only subscribes to the input topic when there are subscribers to the output topic.
4. When the last subscriber to the output topic disconnects, the node unsubscribes from the input topic to save resources.
5. The output topic remains advertised as long as the input topic is available, even if there are no current subscribers.
6. If the input topic becomes unavailable, the node stops advertising the output topic.

## Building with Buildroot

If you're using Buildroot, you can include this package in your build by adding the following to your external Buildroot tree:

1. Create a file named `livox_converter_node.mk` in your Buildroot package directory with the following content:

   ```makefile
   ################################################################################
   #
   # livox_converter_node
   #
   ################################################################################

   LIVOX_CONVERTER_NODE_VERSION = main
   LIVOX_CONVERTER_NODE_SITE = $(call github,pbreid,livox_converter,$(LIVOX_CONVERTER_NODE_VERSION))
   LIVOX_CONVERTER_NODE_SITE_METHOD = git
   LIVOX_CONVERTER_NODE_GIT_SUBMODULES = YES
   LIVOX_CONVERTER_NODE_DEPENDENCIES = livox-ros-driver

   $(eval $(catkin-package))
   ```

2. Add the package to your Buildroot configuration.

## Contributing

Contributions to improve the Livox Converter are welcome. Please feel free to submit issues or pull requests on the GitHub repository.

## License

This project is licensed under the MIT License - see below for details:

MIT License

Copyright (c) 2024 Peter Reid

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Contact

For any queries or support, please open an issue on the GitHub repository.
