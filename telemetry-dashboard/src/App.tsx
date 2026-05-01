import { useEffect, useRef, useState } from 'react'
import * as ROSLIB from 'roslib'
import { BatteryPanel } from './components/BatteryPanel'
import { CameraPanel } from './components/CameraPanel'
import { Charts } from './components/Charts'
import { LidarPolar } from './components/LidarPolar'
import { MapPanel } from './components/MapPanel'
import { RawJson } from './components/RawJson'
import { StatusTiles } from './components/StatusTiles'
import type { BatteryState, CompressedImage, LaserScan, Odometry, OccupancyGrid, RacerTelemetry, RosPath, Sample } from './telemetry'
import { BUFFER_LEN, toSample } from './telemetry'
import './App.css'

const DEFAULT_ROS_URL = 'ws://100.86.204.127:9090'
const RECONNECT_MS = 2000
const TELEMETRY_STALE_MS = 500

function resolveRosUrl(): string {
  // Priority: ?ros=ws://... in URL > VITE_ROS_URL env > hardcoded fallback.
  // The query param makes it trivial to point a browser tab at a different
  // robot without rebuilding; the env var covers deployments.
  try {
    const fromQuery = new URLSearchParams(window.location.search).get('ros')
    if (fromQuery) return fromQuery
  } catch {
    /* non-browser env */
  }
  const fromEnv = (import.meta.env.VITE_ROS_URL as string | undefined) ?? ''
  return fromEnv || DEFAULT_ROS_URL
}

function hasFlag(name: string): boolean {
  try {
    return new URLSearchParams(window.location.search).has(name)
  } catch {
    return false
  }
}

function App() {
  const [rosUrl] = useState(resolveRosUrl)
  // Bisection toggles: ?nocam, ?noscan, ?notel disable individual subscriptions
  // without a rebuild. Use these to isolate a memory/perf source.
  const [disableCamera] = useState(() => hasFlag('nocam'))
  const [disableScan] = useState(() => hasFlag('noscan'))
  const [disableTelemetry] = useState(() => hasFlag('notel'))
  const [disableLineDebug] = useState(() => hasFlag('nolinedbg'))
  const [connected, setConnected] = useState(false)
  const [latest, setLatest] = useState<RacerTelemetry | null>(null)
  const [samples, setSamples] = useState<Sample[]>([])
  const [imageArrivedAt, setImageArrivedAt] = useState(0)
  const [imageFormat, setImageFormat] = useState<string | null>(null)
  const [lineDebugArrivedAt, setLineDebugArrivedAt] = useState(0)
  const [lineDebugFormat, setLineDebugFormat] = useState<string | null>(null)
  const [scan, setScan] = useState<LaserScan | null>(null)
  const [scanArrivedAt, setScanArrivedAt] = useState(0)
  const [map, setMap] = useState<OccupancyGrid | null>(null)
  const [mapArrivedAt, setMapArrivedAt] = useState(0)
  const [plannedPath, setPlannedPath] = useState<RosPath | null>(null)
  const [recordedPath, setRecordedPath] = useState<RosPath | null>(null)
  const [inflatedMap, setInflatedMap] = useState<OccupancyGrid | null>(null)
  const [telemetryArrivedAt, setTelemetryArrivedAt] = useState(0)
  const [now, setNow] = useState(() => Date.now())
  const [lapStartMs, setLapStartMs] = useState<number | null>(null)
  const [lapElapsedMs, setLapElapsedMs] = useState(0)
  const lapStartMsRef = useRef<number | null>(null)
  lapStartMsRef.current = lapStartMs
  const motionCountRef = useRef(0)
  const isArmedRef = useRef(false)
  const [battery, setBattery] = useState<BatteryState | null>(null)
  const [batteryArrivedAt, setBatteryArrivedAt] = useState(0)
  const bufferRef = useRef<Sample[]>([])
  // Camera frames bypass React state: at 30 Hz, routing ~20 KB Uint8Arrays
  // through setState ramps Chrome's tab memory by tens of MB/s (off-heap,
  // invisible to JS heap snapshots — likely React retaining state snapshots
  // across renders). CameraPanel registers a paint fn here on mount and we
  // call it synchronously from the subscribe callback.
  const cameraPaintRef = useRef<((raw: CompressedImage) => void) | null>(null)
  const lineDebugPaintRef = useRef<((raw: CompressedImage) => void) | null>(null)

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: rosUrl })
    let reconnectTimer: number | null = null
    let disposed = false

    const scheduleReconnect = () => {
      if (disposed || reconnectTimer !== null) return
      reconnectTimer = window.setTimeout(() => {
        reconnectTimer = null
        if (!disposed) ros.connect(rosUrl)
      }, RECONNECT_MS)
    }

    ros.on('connection', () => setConnected(true))
    ros.on('close', () => {
      setConnected(false)
      scheduleReconnect()
    })
    ros.on('error', () => {
      setConnected(false)
      scheduleReconnect()
    })

    const telemetryTopic = disableTelemetry
      ? null
      : new ROSLIB.Topic({
          ros,
          name: '/telemetry/racer',
          messageType: 'racer_msgs/msg/RacerTelemetry',
        })
    telemetryTopic?.subscribe((raw) => {
      const msg = raw as unknown as RacerTelemetry
      if (msg.armed) isArmedRef.current = true
      setLatest(msg)
      setTelemetryArrivedAt(Date.now())
      const next = bufferRef.current.slice(-(BUFFER_LEN - 1))
      next.push(toSample(msg))
      bufferRef.current = next
      setSamples(next)
    })

    // Camera on its own topic. Use `cbor` (not `cbor-raw`): we get the full
    // message structure with `data` delivered as a Uint8Array — no base64,
    // no atob, and format/header fields preserved. `cbor-raw` would only
    // ship the raw bytes of a single uint8[] field and drop the envelope.
    const cameraTopic = disableCamera
      ? null
      : new ROSLIB.Topic({
          ros,
          name: '/telemetry/camera',
          messageType: 'sensor_msgs/msg/CompressedImage',
          compression: 'cbor',
          queue_length: 1,
          throttle_rate: 0,
        })
    cameraTopic?.subscribe((raw) => {
      const msg = raw as unknown as CompressedImage
      cameraPaintRef.current?.(msg)
      setImageArrivedAt(Date.now())
      setImageFormat((prev) => (prev === msg.format ? prev : msg.format))
    })

    const lineDebugTopic = disableLineDebug
      ? null
      : new ROSLIB.Topic({
          ros,
          name: '/telemetry/line_debug',
          messageType: 'sensor_msgs/msg/CompressedImage',
          compression: 'cbor',
          queue_length: 1,
          throttle_rate: 0,
        })
    lineDebugTopic?.subscribe((raw) => {
      const msg = raw as unknown as CompressedImage
      lineDebugPaintRef.current?.(msg)
      setLineDebugArrivedAt(Date.now())
      setLineDebugFormat((prev) => (prev === msg.format ? prev : msg.format))
    })

    const scanTopic = disableScan
      ? null
      : new ROSLIB.Topic({
          ros,
          name: '/scan',
          messageType: 'sensor_msgs/msg/LaserScan',
          compression: 'cbor',
          queue_length: 1,
          throttle_rate: 200,
        })
    scanTopic?.subscribe((raw) => {
      setScan(raw as unknown as LaserScan)
      setScanArrivedAt(Date.now())
    })

    const mapTopic = new ROSLIB.Topic({
      ros,
      name: '/map',
      messageType: 'nav_msgs/msg/OccupancyGrid',
      compression: 'cbor',
      queue_length: 1,
      throttle_rate: 1000,
    })
    mapTopic.subscribe((raw) => {
      setMap(raw as unknown as OccupancyGrid)
      setMapArrivedAt(Date.now())
    })

    const plannedPathTopic = new ROSLIB.Topic({
      ros,
      name: '/planned_path',
      messageType: 'nav_msgs/msg/Path',
      compression: 'cbor',
      queue_length: 1,
      throttle_rate: 2000,
    })
    plannedPathTopic.subscribe((raw) => {
      setPlannedPath(raw as unknown as RosPath)
    })

    const recordedPathTopic = new ROSLIB.Topic({
      ros,
      name: '/recorded_path',
      messageType: 'nav_msgs/msg/Path',
      compression: 'cbor',
      queue_length: 1,
      throttle_rate: 1000,
    })
    recordedPathTopic.subscribe((raw) => {
      setRecordedPath(raw as unknown as RosPath)
    })

    const inflatedMapTopic = new ROSLIB.Topic({
      ros,
      name: '/inflated_map',
      messageType: 'nav_msgs/msg/OccupancyGrid',
      compression: 'cbor',
      queue_length: 1,
      throttle_rate: 5000,
    })
    inflatedMapTopic.subscribe((raw) => {
      setInflatedMap(raw as unknown as OccupancyGrid)
    })

    const odomTopic = new ROSLIB.Topic({
      ros,
      name: '/odom_rf2o',
      messageType: 'nav_msgs/msg/Odometry',
      compression: 'cbor',
      queue_length: 1,
      throttle_rate: 100,
    })
    odomTopic.subscribe((raw) => {
      const msg = raw as unknown as Odometry
      const vx = msg.twist?.twist?.linear?.x ?? 0
      // Only start timer once armed + 3 consecutive readings > 0.1 m/s
      if (isArmedRef.current && Math.abs(vx) > 0.1) {
        motionCountRef.current += 1
        if (motionCountRef.current >= 3 && lapStartMsRef.current === null) {
          setLapStartMs(Date.now())
        }
      } else {
        motionCountRef.current = 0
      }
    })

    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: '/battery_state',
      messageType: 'sensor_msgs/msg/BatteryState',
      compression: 'cbor',
      queue_length: 1,
      throttle_rate: 1000,
    })
    batteryTopic.subscribe((raw) => {
      setBattery(raw as unknown as BatteryState)
      setBatteryArrivedAt(Date.now())
    })

    return () => {
      disposed = true
      if (reconnectTimer !== null) window.clearTimeout(reconnectTimer)
      telemetryTopic?.unsubscribe()
      cameraTopic?.unsubscribe()
      lineDebugTopic?.unsubscribe()
      scanTopic?.unsubscribe()
      mapTopic.unsubscribe()
      plannedPathTopic.unsubscribe()
      inflatedMapTopic.unsubscribe()
      odomTopic.unsubscribe()
      batteryTopic.unsubscribe()
      ros.close()
    }
  }, [rosUrl, disableCamera, disableLineDebug, disableScan, disableTelemetry])

  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 500)
    return () => window.clearInterval(id)
  }, [])

  useEffect(() => {
    if (lapStartMs === null) return
    const id = window.setInterval(() => setLapElapsedMs(Date.now() - lapStartMs), 100)
    return () => window.clearInterval(id)
  }, [lapStartMs])

  const telemetryStale =
    telemetryArrivedAt === 0 || now - telemetryArrivedAt > TELEMETRY_STALE_MS
  const telemetryAge = telemetryArrivedAt
    ? ((now - telemetryArrivedAt) / 1000).toFixed(1)
    : null

  return (
    <div className="dashboard">
      <header className="dashboard-header">
        <h1>Racer Telemetry</h1>
        <div className="header-status">
          {connected && telemetryAge !== null && (
            <span className={`badge ${telemetryStale ? 'badge-stale' : 'badge-live'}`}>
              telemetry · {telemetryAge}s ago
            </span>
          )}
          <div className={`connection-status ${connected ? 'connected' : 'disconnected'}`}>
            <span className="status-dot" />
            {connected ? 'Connected' : 'Disconnected'}
          </div>
        </div>
      </header>

      <StatusTiles
        connected={connected}
        latest={latest}
        lapMs={lapStartMs !== null ? lapElapsedMs : null}
        onLapReset={() => { setLapStartMs(null); setLapElapsedMs(0) }}
      />

      <div className="dashboard-grid">
        <CameraPanel paintRef={cameraPaintRef} format={imageFormat} arrivedAt={imageArrivedAt} />
        <CameraPanel
          title="Line Debug"
          paintRef={lineDebugPaintRef}
          format={lineDebugFormat}
          arrivedAt={lineDebugArrivedAt}
        />
        <LidarPolar scan={scan} arrivedAt={scanArrivedAt} />
        <Charts samples={samples} />
        <MapPanel map={map} arrivedAt={mapArrivedAt} plannedPath={plannedPath} inflatedMap={inflatedMap} recordedPath={recordedPath} />
        <BatteryPanel battery={battery} arrivedAt={batteryArrivedAt} />
      </div>

      <RawJson msg={latest} />
    </div>
  )
}

export default App
