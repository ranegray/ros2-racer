export type RosVec3 = { x: number; y: number; z: number }

export type RosTime = { sec: number; nanosec: number }

// CompressedImage from rosbridge. The `data` field arrives as:
//   - base64 string        under default JSON compression
//   - Uint8Array (or ArrayBuffer) under `cbor-raw` compression — we subscribe
//     with cbor-raw to skip the base64 bloat and parse cost for the dashboard.
export type CompressedImage = {
  header: { stamp: RosTime; frame_id: string }
  format: string
  data: string | Uint8Array | ArrayBuffer
}

export type LaserScan = {
  header: { stamp: RosTime; frame_id: string }
  angle_min: number
  angle_max: number
  angle_increment: number
  time_increment: number
  scan_time: number
  range_min: number
  range_max: number
  ranges: number[]
  intensities: number[]
}

export type RacerTelemetry = {
  header: { stamp: RosTime; frame_id: string }
  gyro: RosVec3
  accel: RosVec3
  armed: boolean
  internal_state: string
  front_distance: number
  heading_error: number
  obstacle_detection: string
}

export type BatteryState = {
  header: { stamp: RosTime; frame_id: string }
  voltage: number     // V
  current: number     // A  (NaN if not measured)
  percentage: number  // 0.0–1.0 (NaN if not estimated)
}

export type MotorTemps = {
  data: number[]  // °C per ESC, up to 4 elements
}

export type Odometry = {
  header: { stamp: RosTime; frame_id: string }
  twist: { twist: { linear: RosVec3; angular: RosVec3 } }
}

export type RosPoint2D = { x: number; y: number }

export type RosPath = {
  header: { stamp: RosTime; frame_id: string }
  poses: Array<{
    header: { stamp: RosTime; frame_id: string }
    pose: { position: { x: number; y: number; z: number }; orientation: { x: number; y: number; z: number; w: number } }
  }>
}

export type OccupancyGrid = {
  header: { stamp: RosTime; frame_id: string }
  info: {
    resolution: number   // metres per cell
    width: number        // cells
    height: number       // cells
    origin: {
      position: { x: number; y: number; z: number }
      orientation: { x: number; y: number; z: number; w: number }
    }
  }
  // -1 = unknown, 0 = free, 100 = occupied
  data: number[] | Int8Array | Uint8Array
}

export type Sample = {
  t: number // epoch seconds
  front_distance: number
  heading_error: number
  gyro_z: number
  accel_x: number
}

export const BUFFER_LEN = 200

export type BatterySample = {
  t: number
  voltage: number
  current: number
  power: number
}
export const BATTERY_BUFFER_LEN = 200

export type LidarSample = {
  t: number
  minRange: number   // minimum valid range in full scan
  fwdMean: number    // mean range of ±30° forward sector
  validPct: number   // fraction of readings that are finite & in-range (0–1)
}
export const LIDAR_BUFFER_LEN = 200

export function stampToSeconds(stamp: RosTime): number {
  return stamp.sec + stamp.nanosec * 1e-9
}

export function toSample(msg: RacerTelemetry): Sample {
  return {
    t: stampToSeconds(msg.header.stamp) || Date.now() / 1000,
    front_distance: msg.front_distance,
    heading_error: msg.heading_error,
    gyro_z: msg.gyro.z,
    // Forward-axis linear acceleration. Magnitude would be dominated by gravity
    // on the z-axis (~9.8 m/s² at rest), so we report the body-x component.
    accel_x: msg.accel.x,
  }
}
