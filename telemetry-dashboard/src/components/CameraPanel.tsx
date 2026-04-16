import { useEffect, useRef, useState } from 'react'
import type { CompressedImage } from '../telemetry'

type Props = {
  image: CompressedImage | null
  /** wall-clock ms when the latest frame arrived on the dashboard (Date.now()) */
  arrivedAt: number
  stalenessMs?: number
}

function mimeFor(format: string): string {
  const f = format.toLowerCase()
  if (f.includes('png')) return 'image/png'
  return 'image/jpeg'
}

export function CameraPanel({ image, arrivedAt, stalenessMs = 1000 }: Props) {
  // Drive the <img> imperatively via blob URLs so the browser's
  // decoded-image cache (keyed by URL) can't accumulate an entry per frame:
  // each blob URL is revoked when the next frame arrives.
  const imgRef = useRef<HTMLImageElement | null>(null)
  useEffect(() => {
    const img = imgRef.current
    if (!img || !image?.data) return
    // With cbor-raw, `data` is already a Uint8Array / ArrayBuffer, so Blob
    // can take it directly. Under default JSON compression it's a base64
    // string and we decode via atob.
    // Normalize to a fresh ArrayBuffer so Blob's BlobPart type is unambiguous
    // (guards against SharedArrayBuffer-backed views).
    const d = image.data
    let buffer: ArrayBuffer
    if (typeof d === 'string') {
      buffer = Uint8Array.from(atob(d), (c) => c.charCodeAt(0)).buffer as ArrayBuffer
    } else if (d instanceof ArrayBuffer) {
      buffer = d
    } else {
      const view = new Uint8Array(d.buffer, d.byteOffset, d.byteLength)
      buffer = view.slice().buffer as ArrayBuffer
    }
    const url = URL.createObjectURL(new Blob([buffer], { type: mimeFor(image.format) }))
    img.src = url
    return () => URL.revokeObjectURL(url)
  }, [image])

  const [now, setNow] = useState(() => Date.now())
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 500)
    return () => window.clearInterval(id)
  }, [])

  const stale = !image || arrivedAt === 0 || now - arrivedAt > stalenessMs
  const age = arrivedAt ? ((now - arrivedAt) / 1000).toFixed(1) : '—'
  const hasImage = !!image?.data

  return (
    <section className="panel camera-panel">
      <div className="panel-header">
        <h2>Camera</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
          {image ? `${image.format.split(';')[0] || 'jpeg'} · ${age}s ago` : 'no frames'}
        </span>
      </div>
      <div className={`camera-frame ${stale ? 'stale' : ''}`}>
        <img ref={imgRef} alt="camera frame" style={{ display: hasImage ? 'block' : 'none' }} />
        {!hasImage && <div className="camera-placeholder">Waiting for frames…</div>}
      </div>
    </section>
  )
}
