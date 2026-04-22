import { memo, useEffect, useRef, useState, type RefObject } from 'react'
import type { CompressedImage } from '../telemetry'

type Props = {
  /** Mutated by CameraPanel on mount to register its paint fn. The App's ROS
   *  subscribe callback invokes this directly so camera bytes never enter
   *  React state (doing so leaks Chrome tab memory at 30 Hz). */
  paintRef: RefObject<((raw: CompressedImage) => void) | null>
  format: string | null
  /** wall-clock ms when the latest frame arrived (Date.now()) */
  arrivedAt: number
  stalenessMs?: number
}

function mimeFor(format: string | undefined): string {
  const f = format?.toLowerCase() ?? ''
  if (f.includes('png')) return 'image/png'
  return 'image/jpeg'
}

function toBuffer(data: string | Uint8Array | ArrayBuffer | number[]): ArrayBuffer {
  if (typeof data === 'string') {
    return Uint8Array.from(atob(data), (c) => c.charCodeAt(0)).buffer as ArrayBuffer
  }
  if (Array.isArray(data)) {
    return new Uint8Array(data).buffer as ArrayBuffer
  }
  if (data instanceof ArrayBuffer) return data
  return new Uint8Array((data as any).buffer, (data as any).byteOffset, (data as any).byteLength).slice().buffer as ArrayBuffer
}

export const CameraPanel = memo(function CameraPanel({
  paintRef,
  format,
  arrivedAt,
  stalenessMs = 1000,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const hasFrameRef = useRef(false)
  const [hasFrame, setHasFrame] = useState(false)
  const pendingRef = useRef<CompressedImage | null>(null)
  const decodingRef = useRef(false)

  useEffect(() => {
    // Drop-old-frames backpressure: only one createImageBitmap in flight at a
    // time; newer frames overwrite the pending slot. Without this, a decode
    // that runs slower than the 30 Hz input accumulates Blobs off-heap.
    const drain = async () => {
      decodingRef.current = true
      while (pendingRef.current) {
        const frame = pendingRef.current
        pendingRef.current = null
        const blob = new Blob([toBuffer(frame.data)], { type: mimeFor(frame.format) })
        try {
          const bitmap = await createImageBitmap(blob)
          const canvas = canvasRef.current
          if (canvas) {
            if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
              canvas.width = bitmap.width
              canvas.height = bitmap.height
            }
            const ctx = canvas.getContext('2d')
            if (ctx) ctx.drawImage(bitmap, 0, 0)
          }
          bitmap.close()
          if (!hasFrameRef.current) {
            hasFrameRef.current = true
            setHasFrame(true)
          }
        } catch (e) {
          console.error("Decode failed:", e)
          /* decode failed — drop this frame */
        }
      }
      decodingRef.current = false
    }

    paintRef.current = (raw) => {
      pendingRef.current = raw
      if (!decodingRef.current) drain()
    }
    return () => {
      paintRef.current = null
      pendingRef.current = null
    }
  }, [paintRef])

  const [now, setNow] = useState(() => Date.now())
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 500)
    return () => window.clearInterval(id)
  }, [])

  const stale = !hasFrame || arrivedAt === 0 || now - arrivedAt > stalenessMs
  const age = arrivedAt ? ((now - arrivedAt) / 1000).toFixed(1) : '—'

  return (
    <section className="panel camera-panel">
      <div className="panel-header">
        <h2>Camera</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
          {hasFrame ? `${format?.split(';')[0] || 'jpeg'} · ${age}s ago` : 'no frames'}
        </span>
      </div>
      <div className={`camera-frame ${stale ? 'stale' : ''}`}>
        <canvas ref={canvasRef} />
        {!hasFrame && <div className="camera-placeholder">Waiting for frames…</div>}
      </div>
    </section>
  )
})
