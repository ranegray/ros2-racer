import { memo, useEffect, useRef, useState } from 'react'
import type { OccupancyGrid } from '../telemetry'

type Props = {
  map: OccupancyGrid | null
  arrivedAt: number
  stalenessMs?: number
}

function toInt8Array(data: OccupancyGrid['data']): Int8Array {
  if (data instanceof ArrayBuffer) return new Int8Array(data)
  if (ArrayBuffer.isView(data)) {
    return new Int8Array(data.buffer, data.byteOffset, data.byteLength)
  }
  return Int8Array.from(data)
}

function draw(canvas: HTMLCanvasElement, map: OccupancyGrid | null) {
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const dpr = window.devicePixelRatio || 1
  const cssWidth = Math.max(1, canvas.clientWidth)
  const cssHeight = Math.max(1, canvas.clientHeight)
  const pxWidth = Math.max(1, Math.floor(cssWidth * dpr))
  const pxHeight = Math.max(1, Math.floor(cssHeight * dpr))
  if (canvas.width !== pxWidth || canvas.height !== pxHeight) {
    canvas.width = pxWidth
    canvas.height = pxHeight
  }

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = '#0b0f14'
  ctx.fillRect(0, 0, cssWidth, cssHeight)

  if (!map) return

  const { width, height } = map.info
  if (!width || !height) return

  const cells = toInt8Array(map.data)
  const image = ctx.createImageData(width, height)

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const srcIndex = y * width + x
      const dstY = height - 1 - y
      const dstIndex = (dstY * width + x) * 4
      const value = cells[srcIndex] ?? -1

      let shade = 127
      if (value >= 0) {
        shade = 255 - Math.round((Math.min(100, value) / 100) * 255)
      }

      image.data[dstIndex] = shade
      image.data[dstIndex + 1] = shade
      image.data[dstIndex + 2] = shade
      image.data[dstIndex + 3] = value < 0 ? 140 : 255
    }
  }

  const bitmapCanvas = document.createElement('canvas')
  bitmapCanvas.width = width
  bitmapCanvas.height = height
  const bitmapCtx = bitmapCanvas.getContext('2d')
  if (!bitmapCtx) return
  bitmapCtx.putImageData(image, 0, 0)

  const scale = Math.min(cssWidth / width, cssHeight / height)
  const drawWidth = width * scale
  const drawHeight = height * scale
  const offsetX = (cssWidth - drawWidth) / 2
  const offsetY = (cssHeight - drawHeight) / 2

  ctx.imageSmoothingEnabled = false
  ctx.drawImage(bitmapCanvas, offsetX, offsetY, drawWidth, drawHeight)
}

export const MapPanel = memo(function MapPanel({
  map,
  arrivedAt,
  stalenessMs = 2000,
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
  const age = arrivedAt ? ((now - arrivedAt) / 1000).toFixed(1) : '—'
  const details = map
    ? `${map.info.width}x${map.info.height} @ ${map.info.resolution.toFixed(2)} m`
    : 'no map'

  return (
    <section className="panel map-panel">
      <div className="panel-header">
        <h2>Map</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
          {map ? `${details} · ${age}s ago` : details}
        </span>
      </div>
      <div className="map-frame">
        <canvas ref={canvasRef} />
        {!map && <div className="map-placeholder">Waiting for /map…</div>}
      </div>
    </section>
  )
})
