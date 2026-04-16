import { useEffect, useRef, useState } from 'react'
import * as ROSLIB from 'roslib'
import { CameraPanel } from './components/CameraPanel'
import { Charts } from './components/Charts'
import { RawJson } from './components/RawJson'
import { StatusTiles } from './components/StatusTiles'
import type { CompressedImage, RacerTelemetry, Sample } from './telemetry'
import { BUFFER_LEN, toSample } from './telemetry'
import './App.css'

const ROS_URL = 'ws://100.86.204.127:9090'

function App() {
  const [connected, setConnected] = useState(false)
  const [latest, setLatest] = useState<RacerTelemetry | null>(null)
  const [samples, setSamples] = useState<Sample[]>([])
  const [image, setImage] = useState<CompressedImage | null>(null)
  const [imageArrivedAt, setImageArrivedAt] = useState(0)
  const bufferRef = useRef<Sample[]>([])
  const lastFrameAtRef = useRef<number>(0)

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: ROS_URL })

    ros.on('connection', () => setConnected(true))
    ros.on('close', () => setConnected(false))
    ros.on('error', () => setConnected(false))

    const telemetryTopic = new ROSLIB.Topic({
      ros,
      name: '/telemetry/racer',
      messageType: 'racer_msgs/msg/RacerTelemetry',
    })
    telemetryTopic.subscribe((raw) => {
      const msg = raw as unknown as RacerTelemetry
      setLatest(msg)
      const next = bufferRef.current.slice(-(BUFFER_LEN - 1))
      next.push(toSample(msg))
      bufferRef.current = next
      setSamples(next)
    })

    // Camera on its own topic. Use `cbor` (not `cbor-raw`): we get the full
    // message structure with `data` delivered as a Uint8Array — no base64,
    // no atob, and format/header fields preserved. `cbor-raw` would only
    // ship the raw bytes of a single uint8[] field and drop the envelope.
    const cameraTopic = new ROSLIB.Topic({
      ros,
      name: '/telemetry/camera',
      messageType: 'sensor_msgs/msg/CompressedImage',
      compression: 'cbor',
      queue_length: 1,
      throttle_rate: 0,
    })
    cameraTopic.subscribe((raw) => {
      const img = raw as unknown as CompressedImage
      // Diagnostic: verify frame arrival + payload shape in the browser console.
      // Remove once camera is stable.
      const d = img?.data
      const now = Date.now()
      const deltaMs = lastFrameAtRef.current ? now - lastFrameAtRef.current : 0
      lastFrameAtRef.current = now
      console.debug('[camera] frame', {
        deltaMs,
        format: img?.format,
        dataType: typeof d,
        isUint8: d instanceof Uint8Array,
        isArrayBuffer: d instanceof ArrayBuffer,
        byteLength:
          typeof d === 'string'
            ? d.length
            : d instanceof Uint8Array
              ? d.byteLength
              : d instanceof ArrayBuffer
                ? d.byteLength
                : 'unknown',
      })
      setImage(img)
      setImageArrivedAt(now)
    })

    return () => {
      telemetryTopic.unsubscribe()
      cameraTopic.unsubscribe()
      ros.close()
    }
  }, [])

  return (
    <div className="dashboard">
      <header className="dashboard-header">
        <h1>Racer Telemetry</h1>
        <div className={`connection-status ${connected ? 'connected' : 'disconnected'}`}>
          <span className="status-dot" />
          {connected ? 'Connected' : 'Disconnected'}
        </div>
      </header>

      <StatusTiles connected={connected} latest={latest} />

      <div className="dashboard-grid">
        <CameraPanel image={image} arrivedAt={imageArrivedAt} />
        <Charts samples={samples} />
      </div>

      <RawJson msg={latest} />
    </div>
  )
}

export default App
