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

    // Camera on its own topic with cbor-raw so JPEG bytes ship as binary
    // (no base64 bloat, no atob on the client). roslib delivers `data` as
    // Uint8Array under cbor-raw.
    const cameraTopic = new ROSLIB.Topic({
      ros,
      name: '/telemetry/camera',
      messageType: 'sensor_msgs/msg/CompressedImage',
      compression: 'cbor-raw',
      queue_length: 1,
      throttle_rate: 0,
    })
    cameraTopic.subscribe((raw) => {
      const img = raw as unknown as CompressedImage
      setImage(img)
      setImageArrivedAt(Date.now())
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
