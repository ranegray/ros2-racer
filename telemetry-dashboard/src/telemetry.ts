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

export type Sample = {
  t: number // epoch seconds
  front_distance: number
  heading_error: number
  gyro_z: number
  accel_mag: number
}

export const BUFFER_LEN = 200

export function stampToSeconds(stamp: RosTime): number {
  return stamp.sec + stamp.nanosec * 1e-9
}

export function toSample(msg: RacerTelemetry): Sample {
  const a = msg.accel
  return {
    t: stampToSeconds(msg.header.stamp) || Date.now() / 1000,
    front_distance: msg.front_distance,
    heading_error: msg.heading_error,
    gyro_z: msg.gyro.z,
    accel_mag: Math.hypot(a.x, a.y, a.z),
  }
}
