# RacerTelemetry.msg
# Aggregated low-frequency telemetry message for the React dashboard.
# Scalar / status fields only — the camera stream is published separately on
# /telemetry/camera (sensor_msgs/CompressedImage) so heavy payloads don't
# backpressure this aggregate. All fields are optional; unpopulated fields
# stay at their zero-value defaults so the node can publish even if a sensor
# is offline.

std_msgs/Header header

# --- IMU (from rover_node imu/gyro and imu/accel) ---
geometry_msgs/Vector3 gyro         # rad/s
geometry_msgs/Vector3 accel        # m/s^2

# --- Rover status ---
bool armed
string internal_state              # e.g. "IDLE", "NAVIGATING", "STOPPED"

# --- LiDAR ---
float32 front_distance             # processed front-facing distance in metres

# --- Perception / Navigation ---
float32 heading_error              # radians; positive = left of target
string obstacle_detection          # e.g. "NONE", "FRONT", "LEFT", "RIGHT"
