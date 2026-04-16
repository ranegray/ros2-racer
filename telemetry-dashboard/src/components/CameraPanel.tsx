import { memo, useEffect, useRef, useState } from 'react'
import type { CompressedImage } from '../telemetry'

type Props = {
  image: CompressedImage | null
  /** wall-clock ms when the latest frame arrived on the dashboard (Date.now()) */
  arrivedAt: number
  stalenessMs?: number
}

function mimeFor(format: string | undefined): string {
  const f = format?.toLowerCase() ?? ''
  if (f.includes('png')) return 'image/png'
  return 'image/jpeg'
}

function toBuffer(data: string | Uint8Array | ArrayBuffer): ArrayBuffer {
  if (typeof data === 'string') {
    return Uint8Array.from(atob(data), (c) => c.charCodeAt(0)).buffer as ArrayBuffer
  }
  if (data instanceof ArrayBuffer) return data
  return new Uint8Array(data.buffer, data.byteOffset, data.byteLength).slice().buffer as ArrayBuffer
}

export const CameraPanel = memo(function CameraPanel({ image, arrivedAt, stalenessMs = 1000 }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const hasFrameRef = useRef(false)
  const [hasFrame, setHasFrame] = useState(false)

  // Decode each frame with createImageBitmap (off-main-thread) and paint to a
  // canvas in a single drawImage call. This avoids the <img src> swap flicker:
  // the canvas bitmap is only replaced once the new frame is fully decoded.
  useEffect(() => {
    if (!image?.data) return
    let cancelled = false
    const blob = new Blob([toBuffer(image.data)], { type: mimeFor(image.format) })
    createImageBitmap(blob)
      .then((bitmap) => {
        if (cancelled) {
          bitmap.close()
          return
        }
        const canvas = canvasRef.current
        if (!canvas) {
          bitmap.close()
          return
        }
        if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
          canvas.width = bitmap.width
          canvas.height = bitmap.height
        }
        const ctx = canvas.getContext('2d')
        if (ctx) ctx.drawImage(bitmap, 0, 0)
        bitmap.close()
        if (!hasFrameRef.current) {
          hasFrameRef.current = true
          setHasFrame(true)
        }
      })
      .catch(() => {
        /* decode failed — drop this frame */
      })
    return () => {
      cancelled = true
    }
  }, [image])

  const [now, setNow] = useState(() => Date.now())
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 500)
    return () => window.clearInterval(id)
  }, [])

  const stale = !image || arrivedAt === 0 || now - arrivedAt > stalenessMs
  const age = arrivedAt ? ((now - arrivedAt) / 1000).toFixed(1) : '—'

  return (
    <section className="panel camera-panel">
      <div className="panel-header">
        <h2>Camera</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
          {image ? `${image.format?.split(';')[0] || 'jpeg'} · ${age}s ago` : 'no frames'}
        </span>
      </div>
      <div className={`camera-frame ${stale ? 'stale' : ''}`}>
        <canvas ref={canvasRef} />
        {!hasFrame && <div className="camera-placeholder">Waiting for frames…</div>}
      </div>
    </section>
  )
})
