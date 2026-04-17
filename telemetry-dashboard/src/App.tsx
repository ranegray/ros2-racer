import { useEffect, useState } from 'react'
import * as ROSLIB from 'roslib'
import './App.css'

const ROS_URL = 'ws://100.86.204.127:9090'

function App() {
  const [connected, setConnected] = useState(false)
  const [telemetry, setTelemetry] = useState<object | null>(null)

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: ROS_URL })

    ros.on('connection', () => setConnected(true))
    ros.on('close', () => setConnected(false))
    ros.on('error', () => setConnected(false))

    const topic = new ROSLIB.Topic({
      ros,
      name: '/telemetry/racer',
      messageType: 'racer_msgs/msg/RacerTelemetry',
    })

    topic.subscribe((msg) => setTelemetry(msg as object))

    return () => {
      topic.unsubscribe()
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

      <main className="telemetry-output">
        <pre>
          {telemetry
            ? JSON.stringify(telemetry, null, 2)
            : 'Waiting for data…'}
        </pre>
      </main>
    </div>
  )
}

export default App
