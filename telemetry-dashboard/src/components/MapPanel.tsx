import { memo, useEffect, useRef, useState } from 'react'
import type { OccupancyGrid } from '../telemetry'

type Props = {
  map: OccupancyGrid | null
  arrivedAt: number
  stalenessMs?: number
}

const BG = '#0b0f14'
// cell colours
const C_UNKNOWN = [18, 20, 30]    // dark blue-grey
const C_FREE    = [220, 240, 220] // pale green
const C_OCC     = [20, 20, 20]    // near-black

function draw(canvas: HTMLCanvasElement, map: OccupancyGrid | null) {
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
}

export const MapPanel = memo(function MapPanel({
  map,
  arrivedAt,
  stalenessMs = 3000,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const mapRef = useRef<OccupancyGrid | null>(map)
  mapRef.current = map
  const [now, setNow] = useState(() => Date.now())

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    draw(canvas, map)
  }, [map])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ro = new ResizeObserver(() => draw(canvas, mapRef.current))
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
      </div>
      <div className="map-frame">
        <canvas ref={canvasRef} />
        {!map && <div className="map-placeholder">Waiting for /map…</div>}
      </div>
    </section>
  )
})
