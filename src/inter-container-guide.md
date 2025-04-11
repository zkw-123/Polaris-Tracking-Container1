# Inter-Container Communication Guide

This guide explains how to set up communication between the ROS 2 NDI Tracker container and other containers. This allows you to:

1. Receive tracking data from the NDI tracker in another container
2. Send commands to the NDI tracker from another container

## Communication Mechanisms

The NDI tracker node provides two main communication mechanisms:

1. **ROS 2 Topic (`ndi_transforms`)**: Publishes transform data as JSON strings
2. **ROS 2 Service (`ndi_command`)**: Accepts commands to control the tracker

## Setting Up Client Containers

### 1. Using the Docker Compose File

The `docker-compose.yml` file includes a commented-out example of a client container. To use it:

1. Uncomment the `client_container` section
2. Customize it for your specific needs
3. Create the necessary client code in the `./client` directory

```yaml
client_container:
  image: osrf/ros:humble-desktop
  container_name: client_container
  volumes:
    - ./client:/ros2_ws/src/client
  environment:
    - ROS_DOMAIN_ID=1  # Must match tracker's domain ID
  network_mode: host
  entrypoint: ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && cd /ros2_ws && ros2 run client_node client_node"]
  depends_on:
    - ros2_ndi_tracker
```

### 2. Creating a Custom Client Container

You can also create your own Dockerfile for a client container:

```dockerfile
FROM osrf/ros:humble-desktop

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy your client code
COPY ./client_code /ros2_ws/src/client_code

# Set up ROS environment
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select client_code

# Run client
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run client_code client_node"]
```

### 3. Using an Existing Container

To connect to the NDI tracker from an existing ROS 2 container:

1. Ensure both containers use the same network (preferably `network_mode: host`)
2. Set the same `ROS_DOMAIN_ID` environment variable in both containers
3. Use the ROS 2 command-line tools to verify connection:

```bash
# List available topics
ros2 topic list

# Echo the transform data
ros2 topic echo /ndi_transforms

# Call the command service
ros2 service call /ndi_command std_srvs/srv/Trigger
```

## Example Client Code

An example client node is provided in `client_node.py`. This node demonstrates:

1. Subscribing to the `ndi_transforms` topic to receive tracking data
2. Sending commands to the `ndi_command` service

To use this code:

1. Create a ROS 2 package in your client container
2. Add this file to the package
3. Configure the package to register this node as an entry point

## Available Commands

The NDI tracker node accepts the following commands via its service:

- **`start_logging`**: Start logging transforms to a new log file
- **`stop_logging`**: Stop logging transforms
- **`get_status`**: Get the current status of the tracker

Example of sending a command:

```bash
# Using ros2 command-line
ros2 service call /ndi_command std_srvs/srv/Trigger "{}"

# Using Python client
client.send_command("start_logging")
```

## Transform Data Format

The transform data is published as a JSON string with the following format:

```json
{
  "timestamp": "2