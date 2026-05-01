import { memo, useEffect, useRef, useState } from 'react'
import type { LaserScan } from '../telemetry'

type Props = {
  scan: LaserScan | null
  arrivedAt: number
  displayMax?: number
  stalenessMs?: number
}

const BG = '#0b0f14'
const GRID = 'rgba(255, 255, 255, 0.08)'
const GRID_STRONG = 'rgba(255, 255, 255, 0.16)'
const FORWARD = '#38bdf8'
const POINT = '#22c55e'

function draw(
  canvas: HTMLCanvasElement,
  scan: LaserScan | null,
  displayMax: number,
) {
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  const dpr = window.devicePixelRatio || 1
  const cssSize = canvas.clientWidth
  const pxSize = Math.max(1, Math.floor(cssSize * dpr))
  if (canvas.width !== pxSize || canvas.height !== pxSize) {
    canvas.width = pxSize
    canvas.height = pxSize
  }

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = BG
  ctx.fillRect(0, 0, cssSize, cssSize)

  const cx = cssSize / 2
  const cy = cssSize / 2
  const R = cssSize / 2 - 6
  const scale = R / displayMax

  // Range rings at 1 m intervals, every other one emphasized.
  ctx.lineWidth = 1
  for (let r = 1; r <= displayMax; r += 1) {
    ctx.beginPath()
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2)
    ctx.strokeStyle = r % 2 === 0 ? GRID_STRONG : GRID
    ctx.stroke()
  }

  // Cross-hairs + forward tick.
  ctx.strokeStyle = GRID
  ctx.beginPath()
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R)
  ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy)
  ctx.stroke()

  ctx.fillStyle = FORWARD
  ctx.beginPath()
  ctx.moveTo(cx, cy - R - 2)
  ctx.lineTo(cx - 5, cy - R + 6)
  ctx.lineTo(cx + 5, cy - R + 6)
  ctx.closePath()
  ctx.fill()

  if (!scan) return

  // Points — ROS LaserScan: angle 0 is forward (+x), CCW positive.
  // Canvas: forward = up, so x = sin(angle), y = -cos(angle).
  const { ranges, angle_min, angle_increment, range_min, range_max } = scan
  const maxValid = Math.min(range_max, displayMax)
  ctx.fillStyle = POINT
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i]
    if (!Number.isFinite(r) || r <= 0 || r < range_min || r > maxValid) continue
    const a = angle_min + i * angle_increment
    const px = cx + Math.sin(a) * r * scale
    const py = cy - Math.cos(a) * r * scale
    ctx.fillRect(px - 1, py - 1, 2, 2)
  }
}

export const LidarPolar = memo(function LidarPolar({
  scan,
  arrivedAt,
  displayMax = 6,
  stalenessMs = 1000,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const scanRef = useRef<LaserScan | null>(scan)
  const displayMaxRef = useRef(displayMax)
  scanRef.current = scan
  displayMaxRef.current = displayMax
  const [now, setNow] = useState(() => Date.now())

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    draw(canvas, scan, displayMax)
  }, [scan, displayMax])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ro = new ResizeObserver(() =>
      draw(canvas, scanRef.current, displayMaxRef.current),
    )
    ro.observe(canvas)
    return () => ro.disconnect()
  }, [])

  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 500)
    return () => window.clearInterval(id)
  }, [])

  const stale = !scan || arrivedAt === 0 || now - arrivedAt > stalenessMs
  const age = arrivedAt ? ((now - arrivedAt) / 1000).toFixed(1) : '—'

  return (
    <section className="panel lidar-panel">
      <div className="panel-header">
        <h2>Lidar</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
          {scan ? `${displayMax} m · ${age}s ago` : 'no scan'}
        </span>
      </div>
      <div className="lidar-frame">
        <canvas ref={canvasRef} />
        {!scan && <div className="lidar-placeholder">Waiting for /scan…</div>}
      </div>
    </section>
  )
})
