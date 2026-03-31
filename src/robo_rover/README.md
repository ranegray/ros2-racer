# ArduPilot Rover ROS2 Package

This ROS2 package provides integrated control and IMU data publishing for a Pixhawk 4 Mini running ArduPilot Rover firmware.

## Topics

### Published Topics
- `/imu/gyro` (`geometry_msgs/Vector3`): Gyroscope data (x, y, z in rad/s)
- `/imu/accel` (`geometry_msgs/Vector3`): Accelerometer data (x, y, z in m/s²)
- `/rover/armed` (`std_msgs/Bool`): Rover armed status

### Subscribed Topics
- `/cmd_vel` (`geometry_msgs/Twist`): Velocity commands
  - `linear.x`: Forward/backward speed (-1.0 to 1.0)
  - `angular.z`: Turning rate (-2.0 to 2.0) [rad/s]

## Package Structure

```
robo_rover/
├── robo_rover/
│   ├── __init__.py
│   └── rover_node.py          # Main rover node
├── launch/
│   └── rover_launch.py        # Launch file
├── resource/
│   └── robo_rover             # Package marker file
├── test/                      # Test files
├── package.xml
├── setup.py
├── requirements.txt
└── README.md
```

## Installation

1. **Create ROS2 workspace (if not already done):**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone this package to your workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Ian-McConachie-CU/ROBO_rover.git
   
3. **Fix the file structure:**
   ```bash
   cd ~/ros2_ws/src
   mv ROBO_rover/robo_rover/ .

3. **Install Python dependencies:**
   ```bash
   cd ~/ros2_ws/src/robo_rover
   pip install -r requirements.txt
   ```

4. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

5. **Set up permissions for serial access:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in for changes to take effect
   ```

## Usage

### Building and Testing

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash

# Test the node directly
ros2 run robo_rover rover_node
```

### Basic Launch

Launch with default parameters:
```bash
ros2 launch robo_rover rover_launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch robo_rover rover_launch.py \
    connection_string:=/dev/ttyACM0 \
    baud_rate:=57600 \
    control_frequency:=25.0 \
    imu_frequency:=25.0 
```

### Available Launch Parameters

- `connection_string`: MAVLink connection (default: `/dev/ttyACM1`)
- `baud_rate`: Serial baud rate (default: `115200`)
- `control_frequency`: Control command rate in Hz (default: `20.0`)
- `imu_frequency`: IMU publishing rate in Hz (default: `20.0`)

## Controlling the Rover

### Using Command Line

Send velocity commands using the command line:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'

# Turn left while moving forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}'

# Turn right
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: -0.5}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist '{}'
```

### Using Python Script

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def move_forward(self, speed=0.5):
        msg = Twist()
        msg.linear.x = speed
        self.cmd_pub.publish(msg)
        
    def turn_left(self, rate=0.5):
        msg = Twist()
        msg.angular.z = rate
        self.cmd_pub.publish(msg)
        
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    controller = RoverController()
    
    # Example usage
    controller.move_forward(0.3)
    
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Monitoring IMU Data
### IMU Data Format

The IMU data is published as separate Vector3 messages:

**Gyroscope (`/imu/gyro`):**
- `x`: Roll rate (rad/s)
- `y`: Pitch rate (rad/s) 
- `z`: Yaw rate (rad/s)

**Accelerometer (`/imu/accel`):**
- `x`: X-axis acceleration (m/s²)
- `y`: Y-axis acceleration (m/s²)
- `z`: Z-axis acceleration (m/s²)

### View Gyroscope Data

```bash
# Show gyroscope messages (rad/s)
ros2 topic echo /imu/gyro

# Show gyro data rate
ros2 topic hz /imu/gyro

# Plot gyro data (requires rqt)
rqt_plot /imu/gyro/x /imu/gyro/y /imu/gyro/z
```

### View Accelerometer Data

```bash
# Show accelerometer messages (m/s²)
ros2 topic echo /imu/accel

# Show accel data rate
ros2 topic hz /imu/accel

# Plot accel data (requires rqt)
rqt_plot /imu/accel/x /imu/accel/y /imu/accel/z
```

### View Rover Status

```bash
# Check if rover is armed
ros2 topic echo /rover/armed

# View all available topics
ros2 topic list

# Get topic info
ros2 topic info /cmd_vel
ros2 topic info /imu/gyro
ros2 topic info /imu/accel
```

### Monitor Both Sensors Simultaneously

```bash
# Terminal 1: Launch rover
ros2 launch robo_rover rover_launch.py

# Terminal 2: Monitor gyro data
ros2 topic echo /imu/gyro

# Terminal 3: Monitor accelerometer data
ros2 topic echo /imu/accel

# Terminal 4: Check data rates
ros2 topic hz /imu/gyro
ros2 topic hz /imu/accel
```

## Safety Features

1. **Command Timeout**: If no velocity commands are received for 1 second, the rover will use default throttle and steering values (usually 0,0 to stop).

2. **Automatic Arming**: The node automatically sets ACRO mode and arms the rover on startup.

3. **Clean Shutdown**: When the node is terminated, it stops the rover and disarms it.


## Troubleshooting

### Connection Issues

1. **Check device permissions:**
   ```bash
   ls -l /dev/ttyACM*
   # Should show your user in the group, or try:
   sudo chmod 666 /dev/ttyACM1
   ```

2. **Verify ArduPilot is running:**
   ```bash
   # Connect with MAVProxy to test
   mavproxy.py --master=/dev/ttyACM1 --baudrate=115200
   ```

3. **Check available serial ports:**
   ```bash
   dmesg | grep tty
   ls /dev/ttyACM* /dev/ttyUSB*
   ```

### Node Issues

1. **Check ROS2 setup:**
   ```bash
   source /opt/ros/humble/setup.bash  # or your ROS2 distro
   source ~/ros2_ws/install/setup.bash
   ```

2. **Verify package installation:**
   ```bash
   ros2 pkg list | grep ardupilot_rover
   ros2 run ardupilot_rover rover_node --help
   ```

3. **Check node logs:**
   ```bash
   ros2 launch ardupilot_rover rover.launch.py --log-level DEBUG
   ```

### IMU Data Issues

1. **Verify message types:**
   ```bash
   ros2 interface show geometry_msgs/Vector3
   ```

2. **Check if IMU messages are being received:**
   ```bash
   # In another terminal, monitor MAVLink messages
   mavproxy.py --master=/dev/ttyACM1 --baudrate=115200
   # Then type: output list
   ```

3. **No IMU data appearing:**
   ```bash
   # Check if topics exist
   ros2 topic list | grep imu
   
   # Check topic info
   ros2 topic info /imu/gyro
   ros2 topic info /imu/accel
   ```
## Notes:
This package provides a low-level controller for steering and velocity. 
However, it requires some tuning to get good tracking results. Don't expect the car to hit the reference targets solely based on the commands. You are expected to use the IMU data to create an outer control loop for optimal performance. 
