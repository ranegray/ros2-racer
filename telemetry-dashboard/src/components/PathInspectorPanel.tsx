import { memo, useCallback, useEffect, useRef, useState } from 'react'

const MIN_DIST   = 0.15
const MAX_BRIDGE = 1.0

type Pose = { x: number; y: number; qz: number; qw: number }

// ─── YAML parser (handles yaml.dump output for {poses: [{x,y,qz,qw},...]} only) ───
function parsePathYaml(text: string): Pose[] {
  const poses: Pose[] = []
  let current: Partial<Pose> | null = null
  for (const rawLine of text.split('\n')) {
    const line = rawLine.trimEnd()
    const listStart = /^- (.+)/.exec(line)
    const keyVal    = /^ {2}(\w+): (.+)/.exec(line)
    if (listStart) {
      if (current) poses.push(current as Pose)
      current = {}
      const kv = /^(\w+): (.+)/.exec(listStart[1])
      if (kv) (current as Record<string, number>)[kv[1]] = parseFloat(kv[2])
    } else if (keyVal && current) {
      (current as Record<string, number>)[keyVal[1]] = parseFloat(keyVal[2])
    }
  }
  if (current && Object.keys(current).length === 4) poses.push(current as Pose)
  return poses
}

// ─── Serialise back to YAML (matches Python yaml.dump output) ──────────────────
function serializePathYaml(poses: Pose[]): string {
  const lines = ['poses:']
  for (const p of poses) {
    lines.push(`- qw: ${p.qw}`)
    lines.push(`  qz: ${p.qz}`)
    lines.push(`  x: ${p.x}`)
    lines.push(`  y: ${p.y}`)
  }
  return lines.join('\n') + '\n'
}

// ─── Interpolation (mirrors interpolate_path.py) ─────────────────────────────
function interpolateGaps(poses: Pose[]): { poses: Pose[]; added: number } {
  if (poses.length < 2) return { poses, added: 0 }
  const filled: Pose[] = [poses[0]]
  let added = 0
  for (let i = 1; i < poses.length; i++) {
    const prev = poses[i - 1]
    const curr = poses[i]
    const gap  = Math.hypot(curr.x - prev.x, curr.y - prev.y)
    if (gap > MIN_DIST && gap <= MAX_BRIDGE) {
      const n = Math.max(1, Math.floor(gap / MIN_DIST))
      for (let j = 1; j < n; j++) {
        const t = j / n
        filled.push({
          x:  prev.x  + t * (curr.x  - prev.x),
          y:  prev.y  + t * (curr.y  - prev.y),
          qz: prev.qz + t * (curr.qz - prev.qz),
          qw: prev.qw + t * (curr.qw - prev.qw),
        })
        added++
      }
    }
    filled.push(curr)
  }
  return { poses: filled, added }
}

function closeLoop(poses: Pose[]): { poses: Pose[]; closed: boolean } {
  if (poses.length < 2) return { poses, closed: false }
  const first = poses[0], last = poses[poses.length - 1]
  const gap = Math.hypot(first.x - last.x, first.y - last.y)
  if (gap < MIN_DIST || gap > MAX_BRIDGE) return { poses, closed: false }
  const n = Math.max(1, Math.floor(gap / MIN_DIST))
  const closing: Pose[] = []
  for (let i = 1; i < n; i++) {
    const t = i / n
    closing.push({
      x:  last.x + t * (first.x - last.x),
      y:  last.y + t * (first.y - last.y),
      qz: last.qz,
      qw: last.qw,
    })
  }
  return { poses: [...poses, ...closing], closed: true }
}

// ─── Canvas draw ─────────────────────────────────────────────────────────────
function drawPath(canvas: HTMLCanvasElement, poses: Pose[], color: string) {
  const ctx = canvas.getContext('2d')
  if (!ctx || poses.length === 0) return

  const dpr  = window.devicePixelRatio || 1
  const cssW = canvas.clientWidth
  const cssH = canvas.clientHeight
  const pxW  = Math.max(1, Math.floor(cssW * dpr))
  const pxH  = Math.max(1, Math.floor(cssH * dpr))
  if (canvas.width !== pxW || canvas.height !== pxH) { canvas.width = pxW; canvas.height = pxH }

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = '#0b0f14'
  ctx.fillRect(0, 0, cssW, cssH)

  const pad = 24
  let minX = poses[0].x, maxX = poses[0].x
  let minY = poses[0].y, maxY = poses[0].y
  for (const p of poses) {
    if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x
    if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y
  }
  const rangeX = maxX - minX || 1
  const rangeY = maxY - minY || 1
  const scale  = Math.min((cssW - pad * 2) / rangeX, (cssH - pad * 2) / rangeY)
  const offX   = pad + ((cssW - pad * 2) - rangeX * scale) / 2
  const offY   = pad + ((cssH - pad * 2) - rangeY * scale) / 2
  const tx = (x: number) => offX + (x - minX) * scale
  const ty = (y: number) => offY + (maxY - y) * scale  // flip y

  // Path line
  ctx.beginPath()
  ctx.strokeStyle = color
  ctx.lineWidth = 1.5
  ctx.lineJoin = 'round'
  ctx.moveTo(tx(poses[0].x), ty(poses[0].y))
  for (let i = 1; i < poses.length; i++) ctx.lineTo(tx(poses[i].x), ty(poses[i].y))
  ctx.stroke()

  // Start dot
  ctx.fillStyle = '#00ff88'
  ctx.beginPath()
  ctx.arc(tx(poses[0].x), ty(poses[0].y), 5, 0, Math.PI * 2)
  ctx.fill()

  // End dot
  const last = poses[poses.length - 1]
  ctx.fillStyle = '#ff4444'
  ctx.beginPath()
  ctx.arc(tx(last.x), ty(last.y), 5, 0, Math.PI * 2)
  ctx.fill()
}

type Props = {
  onSendPath?: (poses: Pose[]) => void
}

// ─── Component ────────────────────────────────────────────────────────────────
export const PathInspectorPanel = memo(function PathInspectorPanel({ onSendPath }: Props) {
  const [fileName, setFileName]         = useState<string | null>(null)
  const [original, setOriginal]         = useState<Pose[]>([])
  const [processed, setProcessed]       = useState<Pose[]>([])
  const [stats, setStats]               = useState<string | null>(null)
  const [showInterp, setShowInterp]     = useState(false)
  const [sent, setSent]                 = useState(false)
  const canvasRef                       = useRef<HTMLCanvasElement | null>(null)
  const posesRef                        = useRef<Pose[]>([])

  const redraw = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const poses = showInterp ? posesRef.current : original
    if (poses.length > 0) drawPath(canvas, poses, showInterp ? '#00e5ff' : '#ff9900')
    else {
      const ctx = canvas.getContext('2d')
      if (ctx) { ctx.fillStyle = '#0b0f14'; ctx.fillRect(0, 0, canvas.width, canvas.height) }
    }
  }, [showInterp, original])

  useEffect(() => { redraw() }, [redraw])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ro = new ResizeObserver(() => redraw())
    ro.observe(canvas)
    return () => ro.disconnect()
  }, [redraw])

  const onFile = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0]
    if (!file) return
    setFileName(file.name)
    setSent(false)
    const reader = new FileReader()
    reader.onload = (ev) => {
      const text = ev.target?.result as string
      const poses = parsePathYaml(text)
      setOriginal(poses)
      setProcessed([])
      setShowInterp(false)
      posesRef.current = []
      setStats(null)
      if (poses.length === 0) setStats('Could not parse any poses from file.')
    }
    reader.readAsText(file)
  }

  const onInterpolate = () => {
    if (original.length < 2) return
    const { poses: gapFilled, added } = interpolateGaps(original)
    const { poses: looped, closed }   = closeLoop(gapFilled)
    posesRef.current = looped
    setProcessed(looped)
    setShowInterp(true)
    setSent(false)
    setStats(
      `${original.length} → ${looped.length} poses  ·  +${added} gap-fill  ·  loop ${closed ? 'closed' : 'gap > 1 m, not closed'}`
    )
  }

  const onSend = () => {
    if (!onSendPath || processed.length === 0) return
    onSendPath(processed)
    setSent(true)
  }

  const onDownload = () => {
    const data = showInterp ? processed : original
    if (data.length === 0) return
    const yaml = serializePathYaml(data)
    const blob = new Blob([yaml], { type: 'text/yaml' })
    const url  = URL.createObjectURL(blob)
    const a    = document.createElement('a')
    a.href     = url
    a.download = fileName ? fileName.replace('.yaml', '_interp.yaml') : 'recorded_path_interp.yaml'
    a.click()
    URL.revokeObjectURL(url)
  }

  const canInterp   = original.length >= 2
  const canDownload = (showInterp ? processed : original).length > 0

  return (
    <section className="panel path-inspector-panel">
      <div className="panel-header">
        <h2>Path Inspector</h2>
        {fileName && <span className="badge badge-live">{fileName}</span>}
        {original.length > 0 && !showInterp && (
          <span className="badge" style={{ marginLeft: 6, color: '#ff9900' }}>{original.length} pts</span>
        )}
        {showInterp && processed.length > 0 && (
          <span className="badge badge-live" style={{ marginLeft: 6, color: '#00e5ff' }}>{processed.length} pts</span>
        )}
      </div>
      <p className="panel-desc">
        Load a <code>recorded_path.yaml</code> from disk, preview the raw path, then interpolate
        gaps (bridges ≤ 1 m) and close the loop before downloading the cleaned file.
        Use the downloaded YAML to replace the one on the rover at <code>~/.ros/recorded_path.yaml</code>.
      </p>

      <div className="path-inspector-controls">
        <label className="path-file-btn">
          Choose file
          <input type="file" accept=".yaml,.yml" onChange={onFile} style={{ display: 'none' }} />
        </label>

        <button
          className="path-action-btn"
          disabled={!canInterp}
          onClick={onInterpolate}
          title="Fill gaps ≤ 1 m and close the loop"
        >
          Interpolate
        </button>

        {showInterp && (
          <button
            className="path-action-btn path-action-secondary"
            onClick={() => { setShowInterp(false); posesRef.current = [] }}
          >
            Show original
          </button>
        )}

        {showInterp && onSendPath && (
          <button
            className={`path-action-btn ${sent ? 'path-action-sent' : 'path-action-primary'}`}
            onClick={onSend}
            title="Publish interpolated path to /dashboard/path_override — planner will replan immediately"
          >
            {sent ? 'Sent ✓' : 'Send to planner'}
          </button>
        )}

        <button
          className="path-action-btn"
          disabled={!canDownload}
          onClick={onDownload}
        >
          Download {showInterp ? 'interpolated' : 'original'}
        </button>
      </div>

      {stats && <div className="path-inspector-stats">{stats}</div>}

      <div className="map-frame">
        <canvas ref={canvasRef} />
        {original.length === 0 && (
          <div className="map-placeholder">Load a recorded_path.yaml to preview</div>
        )}
      </div>

      {(original.length > 0 || showInterp) && (
        <div className="map-legend">
          <span className="map-legend-item"><span className="map-legend-dot" style={{ background: '#00ff88' }} />Start</span>
          <span className="map-legend-item"><span className="map-legend-dot" style={{ background: '#ff4444' }} />End</span>
          <span className="map-legend-item">
            <span className="map-legend-line" style={{ background: showInterp ? '#00e5ff' : '#ff9900' }} />
            {showInterp ? 'Interpolated' : 'Raw'}
          </span>
        </div>
      )}
    </section>
  )
})
