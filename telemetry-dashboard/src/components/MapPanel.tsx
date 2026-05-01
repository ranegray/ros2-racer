import { memo, useEffect, useRef, useState } from 'react'
import type { OccupancyGrid, RobotPose, RosPath } from '../telemetry'

type Props = {
  map: OccupancyGrid | null
  arrivedAt: number
  stalenessMs?: number
  plannedPath?: RosPath | null
  overridePath?: RosPath | null
  inflatedMap?: OccupancyGrid | null
  recordedPath?: RosPath | null
  robotPose?: RobotPose | null
}

const BG = '#0b0f14'
// cell colours
const C_UNKNOWN = [18, 20, 30]    // dark blue-grey
const C_FREE    = [220, 240, 220] // pale green
const C_OCC     = [20, 20, 20]    // near-black

function draw(
  canvas: HTMLCanvasElement,
  map: OccupancyGrid | null,
  plannedPath: RosPath | null | undefined,
  overridePath: RosPath | null | undefined,
  inflatedMap: OccupancyGrid | null | undefined,
  recordedPath: RosPath | null | undefined,
  robotPose: RobotPose | null | undefined,
) {
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const dpr = window.devicePixelRatio || 1
  const cssW = canvas.clientWidth
  const cssH = canvas.clientHeight
  const pxW = Math.max(1, Math.floor(cssW * dpr))
  const pxH = Math.max(1, Math.floor(cssH * dpr))
  if (canvas.width !== pxW || canvas.height !== pxH) {
    canvas.width = pxW
    canvas.height = pxH
  }

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = BG
  ctx.fillRect(0, 0, cssW, cssH)

  if (!map) return

  const { width: mw, height: mh, data } = map.info !== undefined
    ? { width: map.info.width, height: map.info.height, data: map.data }
    : { width: 0, height: 0, data: [] }

  if (mw === 0 || mh === 0) return

  // Scale map to fit canvas while preserving aspect ratio
  const scale = Math.min(cssW / mw, cssH / mh)
  const drawW = Math.floor(mw * scale)
  const drawH = Math.floor(mh * scale)
  const offX = Math.floor((cssW - drawW) / 2)
  const offY = Math.floor((cssH - drawH) / 2)

  // Build an ImageData at map resolution, then scale-draw it
  const img = ctx.createImageData(mw, mh)
  const px = img.data
  for (let row = 0; row < mh; row++) {
    for (let col = 0; col < mw; col++) {
      // ROS map: row 0 is the bottom of the world; flip y for canvas
      const rosIdx = (mh - 1 - row) * mw + col
      const v = (data as number[])[rosIdx] ?? -1
      let r: number, g: number, b: number
      if (v < 0) {
        ;[r, g, b] = C_UNKNOWN
      } else if (v === 0) {
        ;[r, g, b] = C_FREE
      } else {
        // Interpolate: 1..100 → light grey → occupied black
        const t = v / 100
        r = Math.round(C_FREE[0] + t * (C_OCC[0] - C_FREE[0]))
        g = Math.round(C_FREE[1] + t * (C_OCC[1] - C_FREE[1]))
        b = Math.round(C_FREE[2] + t * (C_OCC[2] - C_FREE[2]))
      }
      const i = (row * mw + col) * 4
      px[i] = r; px[i + 1] = g; px[i + 2] = b; px[i + 3] = 255
    }
  }

  // Draw at map resolution into an offscreen canvas, then stretch to display size
  const off = new OffscreenCanvas(mw, mh)
  off.getContext('2d')!.putImageData(img, 0, 0)
  ctx.imageSmoothingEnabled = false
  ctx.drawImage(off, offX, offY, drawW, drawH)

  // Overlay inflated obstacles (semi-transparent red)
  if (inflatedMap && inflatedMap.info.width === mw && inflatedMap.info.height === mh) {
    const inflatedData = inflatedMap.data as number[]
    const ovImg = ctx.createImageData(mw, mh)
    const ovPx = ovImg.data
    for (let row = 0; row < mh; row++) {
      for (let col = 0; col < mw; col++) {
        const rosIdx = (mh - 1 - row) * mw + col
        const i = (row * mw + col) * 4
        if ((inflatedData[rosIdx] ?? 0) >= 50) {
          ovPx[i] = 255; ovPx[i+1] = 60; ovPx[i+2] = 60; ovPx[i+3] = 120
        }
      }
    }
    const ovOff = new OffscreenCanvas(mw, mh)
    ovOff.getContext('2d')!.putImageData(ovImg, 0, 0)
    ctx.imageSmoothingEnabled = false
    ctx.drawImage(ovOff, offX, offY, drawW, drawH)
  }

  const { resolution, origin } = map.info
  const ox = origin.position.x
  const oy = origin.position.y
  const toCanvasX = (wx: number) => offX + ((wx - ox) / resolution) * scale
  const toCanvasY = (wy: number) => offY + drawH - ((wy - oy) / resolution) * scale

  // Overlay recorded path (orange dots) — drawn first so planned path renders on top
  if (recordedPath && recordedPath.poses.length > 0) {
    ctx.save()
    ctx.fillStyle = '#ff9900'
    for (const pose of recordedPath.poses) {
      const { x, y } = pose.pose.position
      ctx.beginPath()
      ctx.arc(toCanvasX(x), toCanvasY(y), 3, 0, Math.PI * 2)
      ctx.fill()
    }
    ctx.restore()
  }

  // Overlay dashboard override path (magenta) before planner path.
  if (overridePath && overridePath.poses.length > 1) {
    ctx.save()
    ctx.strokeStyle = '#d946ef'
    ctx.lineWidth = 2
    ctx.setLineDash([6, 4])
    ctx.beginPath()
    const first = overridePath.poses[0].pose.position
    ctx.moveTo(toCanvasX(first.x), toCanvasY(first.y))
    for (let i = 1; i < overridePath.poses.length; i++) {
      const p = overridePath.poses[i].pose.position
      ctx.lineTo(toCanvasX(p.x), toCanvasY(p.y))
    }
    ctx.stroke()
    ctx.restore()
  }

  // Overlay planned path (blue line) — drawn last so it sits on top
  if (plannedPath && plannedPath.poses.length > 1) {
    ctx.save()
    ctx.strokeStyle = '#00e5ff'
    ctx.lineWidth = 2
    ctx.lineJoin = 'round'
    ctx.beginPath()
    const first = plannedPath.poses[0].pose.position
    ctx.moveTo(toCanvasX(first.x), toCanvasY(first.y))
    for (let i = 1; i < plannedPath.poses.length; i++) {
      const p = plannedPath.poses[i].pose.position
      ctx.lineTo(toCanvasX(p.x), toCanvasY(p.y))
    }
    ctx.stroke()

    // Start dot (green)
    ctx.fillStyle = '#00ff88'
    ctx.beginPath()
    ctx.arc(toCanvasX(first.x), toCanvasY(first.y), 4, 0, Math.PI * 2)
    ctx.fill()

    // End dot (red)
    const last = plannedPath.poses[plannedPath.poses.length - 1].pose.position
    ctx.fillStyle = '#ff4444'
    ctx.beginPath()
    ctx.arc(toCanvasX(last.x), toCanvasY(last.y), 4, 0, Math.PI * 2)
    ctx.fill()
    ctx.restore()
  }

  // Robot position: filled white circle + heading arrow
  if (robotPose) {
    const { x, y } = robotPose.pose.position
    const { z: qz, w: qw } = robotPose.pose.orientation
    const yaw = 2.0 * Math.atan2(qz, qw)
    const cx = toCanvasX(x)
    const cy = toCanvasY(y)
    const r = 6

    ctx.save()
    // Body
    ctx.beginPath()
    ctx.arc(cx, cy, r, 0, Math.PI * 2)
    ctx.fillStyle = '#ffffff'
    ctx.fill()
    ctx.strokeStyle = '#000000'
    ctx.lineWidth = 1.5
    ctx.stroke()

    // Heading arrow — canvas x+ is right, y+ is down; ROS yaw+ is CCW → flip y
    const arrowLen = r + 8
    ctx.beginPath()
    ctx.moveTo(cx, cy)
    ctx.lineTo(cx + arrowLen * Math.cos(-yaw), cy + arrowLen * Math.sin(-yaw))
    ctx.strokeStyle = '#ff3300'
    ctx.lineWidth = 2
    ctx.stroke()
    ctx.restore()
  }
}

export const MapPanel = memo(function MapPanel({
  map,
  arrivedAt,
  stalenessMs = 3000,
  plannedPath,
  overridePath,
  inflatedMap,
  recordedPath,
  robotPose,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const mapRef = useRef<OccupancyGrid | null>(map)
  const pathRef = useRef<RosPath | null | undefined>(plannedPath)
  const overrideRef = useRef<RosPath | null | undefined>(overridePath)
  const inflatedRef = useRef<OccupancyGrid | null | undefined>(inflatedMap)
  const recordedRef = useRef<RosPath | null | undefined>(recordedPath)
  const poseRef = useRef<RobotPose | null | undefined>(robotPose)
  mapRef.current = map
  pathRef.current = plannedPath
  overrideRef.current = overridePath
  inflatedRef.current = inflatedMap
  recordedRef.current = recordedPath
  poseRef.current = robotPose
  const [now, setNow] = useState(() => Date.now())

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    draw(canvas, map, plannedPath, overridePath, inflatedMap, recordedPath, robotPose)
  }, [map, plannedPath, overridePath, inflatedMap, recordedPath, robotPose])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ro = new ResizeObserver(() => draw(canvas, mapRef.current, pathRef.current, overrideRef.current, inflatedRef.current, recordedRef.current, poseRef.current))
    ro.observe(canvas)
    return () => ro.disconnect()
  }, [])

  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 500)
    return () => window.clearInterval(id)
  }, [])

  const stale = !map || arrivedAt === 0 || now - arrivedAt > stalenessMs
  const cells = map ? `${map.info.width}×${map.info.height}` : null
  const res = map ? `${(map.info.resolution * 100).toFixed(0)} cm/cell` : null

  return (
    <section className="panel map-panel">
      <div className="panel-header">
        <h2>SLAM Map</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
          {map ? `${cells} · ${res}` : 'no map'}
        </span>
        {plannedPath && plannedPath.poses.length > 0 && (
          <span className="badge badge-live" style={{ marginLeft: 6, color: '#00e5ff' }}>
            path · {plannedPath.poses.length} pts
          </span>
        )}
      </div>
      <div className="map-frame">
        <canvas ref={canvasRef} />
        {!map && <div className="map-placeholder">Waiting for /map…</div>}
      </div>
      {(plannedPath && plannedPath.poses.length > 1) || (overridePath && overridePath.poses.length > 1) ? (
        <div className="map-legend">
          <span className="map-legend-item"><span className="map-legend-dot" style={{ background: '#00ff88' }} />Start</span>
          <span className="map-legend-item"><span className="map-legend-dot" style={{ background: '#ff4444' }} />End</span>
          {overridePath && overridePath.poses.length > 1 && (
            <span className="map-legend-item"><span className="map-legend-line" style={{ background: '#d946ef' }} />Override</span>
          )}
          <span className="map-legend-item"><span className="map-legend-line" style={{ background: '#00e5ff' }} />Plan</span>
          {!robotPose && <span className="map-legend-item"><span className="map-legend-dot" style={{ background: '#ff9900' }} />Recorded</span>}
          {robotPose && <span className="map-legend-item"><span className="map-legend-dot" style={{ background: '#ffffff' }} />Robot</span>}
        </div>
      ) : null}
    </section>
  )
})
