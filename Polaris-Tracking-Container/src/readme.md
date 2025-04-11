# scikit-surgerynditracker in ROS 2 Docker

This project provides Docker configuration to run the scikit-surgerynditracker package within a ROS 2 environment, enabling easy integration of NDI tracking devices with ROS 2 applications.

## Prerequisites

- Docker
- Docker Compose
- NDI tracking device (Polaris, Aurora, or Vega) - or you can use the dummy mode for testing

## Files in this repository

- `Dockerfile`: Builds a ROS 2 Humble image with scikit-surgerynditracker installed
- `docker-compose.yml`: Docker Compose configuration for running the container
- `entrypoint.sh`: Entrypoint script for the container
- `ndi_tracker_node.py`: ROS 2 node for interfacing with NDI trackers
- `rom_files/`: Directory where you should place your NDI ROM files

## Setup Instructions

1. **Prepare ROM files** (optional for real trackers):
   
   Place your NDI ROM files (`.rom` extension) in the `rom_files/` directory. These files define the tools that will be tracked.

2. **Build the Docker image**:

   ```bash
   docker-compose build
   ```

3. **Run the container**:

   ```bash
   docker-compose up
   ```

   By default, the container will run in dummy mode, which doesn't require real hardware.

## Configuration

Edit the `docker-compose.yml` file to modify the tracker configuration:

### Device Path

If your NDI device is connected via a different port than `/dev/ttyUSB0`, update the `devices` section in the `docker-compose.yml` file:

```yaml
devices:
  - /dev/ttyUSB1:/dev/ttyUSB1  # Replace with your actual device path
```

### Using Real Hardware

To use a real NDI tracker instead of the dummy mode, you can set the `use_dummy` parameter to `false` when launching the node:

```bash
docker-compose run ros2_ndi_tracker ros2 run ros2_ndi_tracker ndi_tracker_node --ros-args -p use_dummy:=false
```

### Tracker-specific Configuration

For Polaris or Aurora trackers, connected via serial port, no additional configuration is needed beyond placing ROM files in the `rom_files/` directory.

For Vega tracker, connected via network, set the IP address and port:

```bash
docker-compose run ros2_ndi_tracker ros2 run ros2_ndi_tracker ndi_tracker_node --ros-args -p use_dummy:=false -p tracker_type:=vega -p ip_address:=192.168.1.10 -p port:=8765
```

## ROS 2 Parameters

The NDI tracker node supports the following parameters:

- `tracker_type`: Type of tracker (`polaris`, `aurora`, or `vega`). Default: `polaris`
- `rom_files_dir`: Directory containing ROM files. Default: `/opt/ndi/rom_files`
- `use_dummy`: Use a dummy tracker for testing. Default: `true`
- `ip_address`: IP address for Vega tracker. Default: empty
- `port`: Port for Vega tracker. Default: `8765`
- `update_rate`: Tracking update rate in Hz. Default: `30.0`
- `log_dir`: Directory for storing log files. Default: `/ros2_ws/logs`
- `log_enabled`: Enable/disable logging transforms to file. Default: `true`

## Data Output

The tracker data is published in three ways:

1. **TF2 Transforms**: Each tool is published with:
   - Parent frame: `ndi_world`
   - Child frame: `tool_X` (where X is the port handle number)

2. **JSON Log Files**: Transform data is logged to time-stamped JSON files in the `logs` directory:
   - File naming format: `log_YYYYMMDD_HHMMSS.json`
   - Each line in the file is a complete JSON object with timestamp and transform data

3. **ROS 2 Topic**: Transform data is published to the `ndi_transforms` topic as JSON strings:
   - Other containers can subscribe to this topic to receive transform data in real-time

You can visualize the transforms using RViz2:

```bash
# In a new terminal
docker-compose run ros2_ndi_tracker ros2 run rviz2 rviz2
```

Then add a TF display in RViz2 to visualize the transforms.

## Troubleshooting

1. **Permission denied for /dev/ttyUSB0**:
   
   Make sure the user in the container has proper permissions. The Dockerfile already adds the user to the `dialout` group, but you may need to ensure permissions on the host system as well:

   ```bash
   sudo usermod -a -G dialout $USER
   # Then log out and back in for the change to take effect
   ```

2. **No tracking data**:

   - Check that your ROM files are correctly placed in the `rom_files/` directory
   - For Vega, make sure the IP address and port are correct
   - Try running with the dummy mode first to verify the node is working

3. **Docker permission issues**:

   You might need to run Docker commands with `sudo`, or add your user to the `docker` group:

   ```bash
   sudo usermod -aG docker $USER
   # Then log out and back in for the change to take effect
   ```

## Extending the System

To create your own ROS 2 package that uses the NDI tracker data:

1. Create a new ROS 2 package that depends on `tf2_ros`
2. Subscribe to the TF topics to receive the tracking data
3. Process the transforms as needed for your application

Example of a simple subscriber:

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class NDITrackerSubscriber(Node):
    def __init__(self):
        super().__init__('ndi_tracker_subscriber')
        
        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a timer to check for transforms
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        try:
            # Look up the transform for tool_0
            transform = self.tf_buffer.lookup_transform('ndi_world', 'tool_0', rclpy.time.Time())
            
            # Process the transform
            position = transform.transform.translation
            orientation = transform.transform.rotation
            
            self.get_logger().info(f"Tool position: ({position.x}, {position.y}, {position.z})")
            
        except Exception as e:
            self.get_logger().warning(f"Could not get transform: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = NDITrackerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
