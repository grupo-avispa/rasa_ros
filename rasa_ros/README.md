# rasa_ros

## Overview
This package is a ROS2 wrapper for the natural language understanding (NLU) [Rasa Open Source](https://rasa.com/docs/rasa/) framework. It provides an interface to interact with a Rasa NLU model using ROS2 actions.

## Installation
### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/jazzy/) (middleware for robotics),


#### Building

To build from source, clone the latest version from the main repository into your colcon workspace and install the python dependencies by running the following commands:
```bash
cd colcon_workspace/src
git clone https://github.com/grupo-avispa/rasa_ros.git -b main
cd colcon_workspace
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install
```

## Usage

First, you need to have a trained Rasa NLU model. You can train a model using the [Rasa Open Source](https://rasa.com/docs/rasa/) framework.

With the model trained, you can run the Rasa server with:
```bash
rasa run --enable-api -p 5005
```

Then, you can launch the rasa_ros node with:
```bash
ros2 launch rasa_ros default.launch.py
```

You can edit the `default.yaml` file to change the parameters of the node.

## Nodes

### rasa_ros

This node has an action server that receives a string with a message and returns the intent and entities extracted by the Rasa NLU model.

#### Actions

* **`parse`** ([rasa_msgs/action/Parse](../rasa_msgs/action/Parse.action))

	Predicts the intent and entities of the message.

#### Parameters

* **`ip_address`** (string, default: "localhost")

	IP address of the Rasa server.

* **`port`** (int, default: 5005)

	Port of the Rasa server.


