#!/usr/bin/env python3
"""
ROS2 ArduPilot Rover Node
Combines steering/throttle control and IMU data publishing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Bool, Float32, Float32MultiArray
from nav_msgs.msg import Odometry
import time
import threading
from pymavlink import mavutil
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Vector3

class ArduPilotRoverNode(Node):
    def __init__(self):
        super().__init__('rover_node')
        
        # Parameters
        self.declare_parameter('connection_string', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('imu_frequency', 20.0)

        # Closed-loop velocity control. Setpoint is cmd_vel.linear.x in m/s,
        # feedback is twist.linear.x from the rf2o LIDAR-odometry topic.
        self.declare_parameter('use_velocity_feedback', True)
        self.declare_parameter('odom_topic', '/odom_rf2o')
        self.declare_parameter('kp_speed', 220.0)        # MAVLink throttle units / (m/s)
        self.declare_parameter('ki_speed', 150.0)        # MAVLink throttle units / (m/s * s)
        self.declare_parameter('max_speed', 1.5)         # m/s setpoint clamp
        self.declare_parameter('max_throttle', 300)      # MAVLink throttle clamp
        self.declare_parameter('odom_stale_sec', 0.5)    # refuse to drive if feedback older than this
        # Setpoint slew limit. Without this, a sudden cmd_vel step (0 -> 0.5 m/s)
        # gives the integrator a huge initial error to chew on while the rover
        # is still breaking stiction; by the time the wheels move the integrator
        # has wound up far past what's needed and the loop overshoots.
        self.declare_parameter('max_accel', 1.0)         # m/s^2 cap on rate of change of target_speed

        # Get parameters
        self.connection_string = self.get_parameter('connection_string').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.imu_freq = self.get_parameter('imu_frequency').value
        self.use_velocity_feedback = bool(self.get_parameter('use_velocity_feedback').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.kp_speed = float(self.get_parameter('kp_speed').value)
        self.ki_speed = float(self.get_parameter('ki_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_throttle = int(self.get_parameter('max_throttle').value)
        self.odom_stale_sec = float(self.get_parameter('odom_stale_sec').value)
        self.max_accel = float(self.get_parameter('max_accel').value)

        # Control variables
        self.default_throttle = 0.0
        self.default_steering = 0.0
        self.current_throttle = self.default_throttle
        self.current_steering = self.default_steering
        self.last_cmd_time = time.time()
        self.cmd_timeout = 1.0  # 1 second timeout for commands

        # Velocity loop state
        self.target_speed_raw = 0.0   # m/s, signed; latest cmd_vel.linear.x (clamped)
        self.target_speed = 0.0       # m/s, signed; slewed setpoint that the PI sees
        self.measured_speed = 0.0     # m/s, signed; from /odom_rf2o
        self.last_odom_time = 0.0     # wall-time of latest odom sample (0 = never)
        self.speed_integral = 0.0     # PI integral term, MAVLink throttle units

        # Connection variables
        self.master = None
        self.connected = False
        self.armed = False
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.gyro_pub = self.create_publisher(Vector3, 'imu/gyro', sensor_qos)
        self.accel_pub = self.create_publisher(Vector3, 'imu/accel', sensor_qos)
        self.armed_pub = self.create_publisher(Bool, 'rover/armed', control_qos)
        # Velocity-loop telemetry. Always published so you can compare measured
        # vs setpoint in the dashboard regardless of whether the loop is active.
        self.speed_pub = self.create_publisher(Float32, 'rover/speed', sensor_qos)
        self.speed_target_pub = self.create_publisher(Float32, 'rover/speed_target', sensor_qos)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.motor_temps_pub = self.create_publisher(Float32MultiArray, '/motor_temps', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, control_qos)
        # rf2o publishes /odom_rf2o as RELIABLE/VOLATILE — match it exactly.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, odom_qos)
        
        # Timers
        self.control_timer = self.create_timer(
            1.0 / self.control_freq, self.control_loop)
        self.imu_timer = self.create_timer(
            1.0 / self.imu_freq, self.imu_loop)
        self.status_timer = self.create_timer(1.0, self.status_loop)
        self.battery_timer = self.create_timer(0.5, self.battery_loop)
        self.motor_temps_timer = self.create_timer(0.5, self.motor_temps_loop)
        
        # Initialize connection
        self.get_logger().info('Initializing ArduPilot Rover Node...')
        self.connect_to_rover()
        
    def connect_to_rover(self):
        """Connect to the rover via MAVLink"""
        try:
            self.get_logger().info(f'Connecting to rover on {self.connection_string}...')
            
            self.master = mavutil.mavlink_connection(
                self.connection_string, 
                baud=self.baud_rate,
                timeout=10
            )
            
            # Wait for heartbeat
            self.get_logger().info('Waiting for heartbeat...')
            heartbeat = self.master.wait_heartbeat(timeout=10)
            
            if heartbeat is None:
                self.get_logger().error('No heartbeat received')
                return False
                
            self.get_logger().info(
                f'Connected to system {self.master.target_system} '
                f'component {self.master.target_component}'
            )
            
            self.connected = True
            
            # Set mode to ACRO
            if self.set_mode('ACRO'):
                time.sleep(2)
                # Arm the rover
                self.arm_rover()
                # Request IMU data
                self.request_imu_data()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            return False
    
    def set_mode(self, mode_name):
        """Set the rover flight mode"""
        if not self.connected:
            return False
            
        mode_mapping = self.master.mode_mapping()
        if mode_name not in mode_mapping:
            self.get_logger().error(f'Mode {mode_name} not available')
            return False
            
        mode_id = mode_mapping[mode_name]
        
        self.get_logger().info(f'Setting mode to {mode_name}')
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Wait for mode change confirmation
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                current_mode = mavutil.mode_string_v10(msg)
                if current_mode == mode_name:
                    self.get_logger().info(f'Mode changed to {mode_name}')
                    return True
            time.sleep(0.1)
            
        self.get_logger().error(f'Failed to change mode to {mode_name}')
        return False
    
    def arm_rover(self):
        """Arm the rover"""
        if not self.connected:
            return False
            
        self.get_logger().info('Arming rover...')
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        
        # Wait for arming confirmation
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.get_logger().info('Rover armed successfully')
                    self.armed = True
                    return True
            time.sleep(0.1)
            
        self.get_logger().error('Failed to arm rover')
        return False
    
    def disarm_rover(self):
        """Disarm the rover"""
        if not self.connected:
            return
            
        self.get_logger().info('Disarming rover...')
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        
        self.armed = False
    
    def request_imu_data(self):
        """Request IMU messages from the autopilot"""
        if not self.connected:
            return
            
        try:
            # Calculate interval in microseconds
            interval_us = int(1000000 / self.imu_freq)
            
            # Request SCALED_IMU messages
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                26,  # SCALED_IMU message ID
                interval_us,
                0, 0, 0, 0, 0
            )
            
            # Request SYS_STATUS (msg ID 1) at 2 Hz for battery monitoring
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                1,       # SYS_STATUS message ID
                500000,  # 2 Hz = 500 000 µs
                0, 0, 0, 0, 0
            )

            self.get_logger().info(f'Requested IMU data at {self.imu_freq} Hz')

        except Exception as e:
            self.get_logger().error(f'Failed to request IMU data: {e}')
    
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands.

        With use_velocity_feedback=True, msg.linear.x is interpreted as a
        signed target speed in m/s and the PI controller in control_loop()
        chooses the throttle that achieves it.

        With use_velocity_feedback=False (legacy), msg.linear.x is treated
        as a normalized throttle in [-1, 1] and mapped directly to MAVLink
        throttle units with a stiction offset.
        """
        # Steering mapping is unchanged in both modes.
        self.current_steering = int(np.clip(msg.angular.z * 500, -1000, 1000))
        self.last_cmd_time = time.time()

        # Closed-loop setpoint (clamped to safe range). The slew limiter in
        # control_loop ramps self.target_speed toward this value at max_accel.
        self.target_speed_raw = float(np.clip(msg.linear.x, -self.max_speed, self.max_speed))

        if not self.use_velocity_feedback:
            # Legacy open-loop mapping (preserved verbatim).
            throttle_raw = msg.linear.x * -400
            offset = 80
            if throttle_raw >= 0:
                throttle_with_offset = throttle_raw + offset
            else:
                throttle_with_offset = throttle_raw - offset
            self.current_throttle = int(np.clip(throttle_with_offset, -300, 300))

    def odom_callback(self, msg):
        """Cache the latest velocity feedback from rf2o (or any nav_msgs/Odometry)."""
        self.measured_speed = float(msg.twist.twist.linear.x)
        self.last_odom_time = time.time()

    def _velocity_pi_step(self, setpoint, measured, dt):
        """One step of PI control with conditional-integration anti-windup.

        Sign convention: positive setpoint = forward. The rover's throttle
        channel is wired inverted, so forward motion requires NEGATIVE
        MAVLink throttle. We compute the natural-sign PI law and negate.

        Anti-windup: tentatively integrate, compute output, clamp. Only
        commit the new integral if integrating did not push the output
        further past the saturation limit.
        """
        error = setpoint - measured
        new_integral = self.speed_integral + error * dt
        natural_out = self.kp_speed * error + self.ki_speed * new_integral
        raw = -natural_out  # apply throttle inversion
        clamped = float(np.clip(raw, -self.max_throttle, self.max_throttle))

        # Freeze integration only when saturated AND error would push deeper
        # past the clamp. Equivalent to (raw-clamped)*error >= 0.
        if (raw - clamped) * error >= 0:
            self.speed_integral = new_integral

        return int(clamped)
    
    def control_loop(self):
        """Main control loop - sends commands at fixed rate."""
        if not self.connected or not self.armed:
            return

        timed_out = (time.time() - self.last_cmd_time) > self.cmd_timeout

        if self.use_velocity_feedback:
            # ---- Closed-loop velocity control ----
            # Slew the setpoint toward the latest commanded value (or zero on
            # watchdog timeout) at max_accel. This caps how fast the PI's
            # error term can grow, which prevents integrator windup at startup
            # and gives a smooth ramp instead of a step into max throttle.
            dt = 1.0 / self.control_freq
            desired_target = 0.0 if timed_out else self.target_speed_raw
            max_step = self.max_accel * dt
            self.target_speed += float(np.clip(
                desired_target - self.target_speed, -max_step, max_step))
            setpoint = self.target_speed

            # Stale-feedback guard: if rf2o stopped publishing, refuse to
            # drive. The integrator would otherwise wind up to max throttle
            # while measured_speed stays at its last (likely wrong) value.
            odom_age = time.time() - self.last_odom_time
            if self.last_odom_time == 0.0 or odom_age > self.odom_stale_sec:
                self.speed_integral = 0.0
                throttle = 0
                steering = 0
                self.get_logger().warning(
                    f'No fresh {self.odom_topic} (age={odom_age:.2f}s); '
                    f'velocity loop holding zero.',
                    throttle_duration_sec=2.0,
                )
            else:
                throttle = self._velocity_pi_step(setpoint, self.measured_speed, dt)
                steering = 0 if timed_out else self.current_steering

            # Telemetry
            sp_msg = Float32()
            sp_msg.data = float(self.measured_speed)
            self.speed_pub.publish(sp_msg)
            tgt_msg = Float32()
            tgt_msg.data = float(setpoint)
            self.speed_target_pub.publish(tgt_msg)
        else:
            # ---- Legacy open-loop path (unchanged behaviour) ----
            if timed_out:
                throttle = int(self.default_throttle * 1000)
                steering = int(self.default_steering * 1000)
            else:
                throttle = self.current_throttle
                steering = self.current_steering

            # Still publish telemetry for monitoring.
            if self.last_odom_time != 0.0:
                sp_msg = Float32()
                sp_msg.data = float(self.measured_speed)
                self.speed_pub.publish(sp_msg)
            tgt_msg = Float32()
            tgt_msg.data = float(self.target_speed_raw)
            self.speed_target_pub.publish(tgt_msg)

        # Send manual control command
        try:
            self.master.mav.manual_control_send(
                self.master.target_system,
                0,
                steering,
                throttle,
                0,
                0,
            )
        except Exception as e:
            self.get_logger().error(f'Failed to send control command: {e}')
    
    def imu_loop(self):
        """IMU data processing loop"""
        if not self.connected:
            return
        
        # Try to get SCALED_IMU first (preferred)
        scaled_imu = self.master.recv_match(type='SCALED_IMU', blocking=False)
        if scaled_imu is not None:
            self.publish_scaled_imu(scaled_imu)
            return
    
    def publish_scaled_imu(self, scaled_imu_msg):
        # Gyro message
        gyro_msg = Vector3()
        gyro_msg.x = scaled_imu_msg.xgyro / 1000.0
        gyro_msg.y = scaled_imu_msg.ygyro / 1000.0
        gyro_msg.z = scaled_imu_msg.zgyro / 1000.0
        self.gyro_pub.publish(gyro_msg)
        
        # Accel message
        accel_msg = Vector3()
        accel_msg.x = (scaled_imu_msg.xacc / 1000.0) * 9.80665
        accel_msg.y = (scaled_imu_msg.yacc / 1000.0) * 9.80665
        accel_msg.z = (scaled_imu_msg.zacc / 1000.0) * 9.80665
        self.accel_pub.publish(accel_msg)
        
    
    def status_loop(self):
        """Publish status information"""
        # Publish armed status
        armed_msg = Bool()
        armed_msg.data = self.armed
        self.armed_pub.publish(armed_msg)
        
        # Check connection health
        if self.connected:
            heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if heartbeat is not None:
                # Update armed status from heartbeat
                self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    
    def battery_loop(self):
        if not self.connected:
            return
        sys_status = self.master.recv_match(type='SYS_STATUS', blocking=False)
        if sys_status is None:
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = sys_status.voltage_battery / 1000.0
        msg.current = (
            sys_status.current_battery / 100.0
            if sys_status.current_battery >= 0 else float('nan')
        )
        msg.percentage = (
            sys_status.battery_remaining / 100.0
            if sys_status.battery_remaining >= 0 else float('nan')
        )
        self.battery_pub.publish(msg)

    def motor_temps_loop(self):
        if not self.connected:
            return
        esc = self.master.recv_match(type='ESC_TELEMETRY_1_TO_4', blocking=False)
        if esc is None:
            return
        msg = Float32MultiArray()
        msg.data = [float(t) for t in esc.temperature]  # degrees C, uint8 per ESC
        self.motor_temps_pub.publish(msg)

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.get_logger().info('Shutting down rover node...')
        
        if self.connected and self.armed:
            # Stop the rover
            try:
                self.master.mav.manual_control_send(
                    self.master.target_system,
                    0, 0, 0, 0, 0  # All zeros to stop
                )
                time.sleep(0.1)
                self.disarm_rover()
            except:
                pass
        
        if self.master:
            self.master.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArduPilotRoverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
