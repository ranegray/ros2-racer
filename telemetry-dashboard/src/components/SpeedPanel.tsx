import { memo, useEffect, useLayoutEffect, useMemo, useRef } from 'react'
import uPlot from 'uplot'
import type { AlignedData } from 'uplot'
import 'uplot/dist/uPlot.min.css'
import type { SpeedSample } from '../telemetry'

type Props = {
  measured: number | null
  setpoint: number | null
  samples: SpeedSample[]
  arrivedAt: number
  stalenessMs?: number
}

const STALL_SETPOINT_MIN = 0.1   // setpoint must exceed this to declare stall
const STALL_MEASURED_MAX = 0.05  // measured below this = stall
const DEFICIT_WARN = 0.08        // warn when lagging setpoint by this much

export const SpeedPanel = memo(function SpeedPanel({
  measured, setpoint, samples, arrivedAt, stalenessMs = 2000,
}: Props) {
  const stale = arrivedAt === 0 || Date.now() - arrivedAt > stalenessMs

  const deficit = measured !== null && setpoint !== null ? setpoint - measured : null
  const stalled = !stale
    && setpoint !== null && setpoint > STALL_SETPOINT_MIN
    && measured !== null && measured < STALL_MEASURED_MAX

  const tone = stalled
    ? 'bad'
    : deficit !== null && deficit > DEFICIT_WARN
      ? 'warn'
      : stale ? 'idle' : 'ok'

  const COLOR: Record<string, string> = {
    ok: '#22c55e', warn: '#f59e0b', bad: '#ef4444', idle: '#4b5563',
  }
  const measuredColor = COLOR[tone]

  // dual-series: [xs, measured, setpoint]
  const data = useMemo<AlignedData>(() => {
    const n = samples.length
    const xs = new Float64Array(n)
    const ms = new Float64Array(n)
    const sp = new Float64Array(n)
    for (let i = 0; i < n; i++) {
      xs[i] = samples[i].t
      ms[i] = samples[i].measured
      sp[i] = samples[i].setpoint
    }
    return [xs, ms, sp]
  }, [samples])

  const hostRef = useRef<HTMLDivElement | null>(null)
  const plotRef = useRef<uPlot | null>(null)

  useEffect(() => {
    if (!hostRef.current) return
    const host = hostRef.current
    const opts: uPlot.Options = {
      width: host.clientWidth || 10,
      height: 90,
      legend: { show: false },
      cursor: { show: false },
      scales: { x: { time: true }, y: { auto: true, min: 0 } },
      axes: [
        { show: false },
        {
          size: 44,
          stroke: getComputedStyle(document.documentElement).getPropertyValue('--text') || '#888',
          grid: { stroke: 'rgba(128,128,128,0.15)' },
          ticks: { show: false },
        },
      ],
      series: [
        {},
        { label: 'Measured', stroke: '#22c55e', width: 2, points: { show: false } },
        { label: 'Setpoint', stroke: '#94a3b8', width: 1.5, dash: [5, 3], points: { show: false } },
      ],
    }
    const plot = new uPlot(opts, data, host)
    plotRef.current = plot
    const ro = new ResizeObserver(() => {
      plot.setSize({ width: host.clientWidth, height: 90 })
    })
    ro.observe(host)
    return () => { ro.disconnect(); plot.destroy(); plotRef.current = null }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  useEffect(() => { plotRef.current?.setData(data) }, [data])

  // When the chart transitions from hidden to visible, ResizeObserver may not
  // have fired yet (it's async). Force a sync resize so the chart fills the panel.
  useLayoutEffect(() => {
    if (!stale && hostRef.current && plotRef.current) {
      plotRef.current.setSize({ width: hostRef.current.clientWidth, height: 90 })
    }
  }, [stale])

  return (
    <section className="panel">
      <div className="panel-header">
        <h2>Speed</h2>
        {stalled
          ? <span className="badge badge-stall">STALLED</span>
          : <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
              {stale ? 'no data' : `${(measured ?? 0).toFixed(2)} m/s`}
            </span>
        }
      </div>
      <p className="panel-desc">
        Measured (green) vs setpoint (dashed). A large deficit means the motor can't reach commanded speed —
        likely low battery, voltage sag, or stiction. STALLED = setpoint&nbsp;&gt;&nbsp;0 but no motion detected.
        Watch for deficit growing over a run as battery drains.
      </p>

      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '8px' }}>
        <div className="battery-stat">
          <span className="tile-label">Measured</span>
          <span className="tile-value" style={{ color: measuredColor, fontSize: '14px' }}>
            {measured !== null ? `${measured.toFixed(2)} m/s` : '—'}
          </span>
        </div>
        <div className="battery-stat">
          <span className="tile-label">Setpoint</span>
          <span className="tile-value" style={{ fontSize: '14px' }}>
            {setpoint !== null ? `${setpoint.toFixed(2)} m/s` : '—'}
          </span>
        </div>
        <div className="battery-stat">
          <span className="tile-label">Deficit</span>
          <span className="tile-value" style={{
            fontSize: '14px',
            color: deficit !== null && deficit > DEFICIT_WARN ? '#ef4444' : undefined,
          }}>
            {deficit !== null ? `${deficit > 0 ? '+' : ''}${deficit.toFixed(2)} m/s` : '—'}
          </span>
        </div>
      </div>

      <div style={stale ? { display: 'none' } : undefined}>
        <div className="sparkline-head" style={{ marginBottom: '2px' }}>
          <span className="sparkline-label">Measured vs Setpoint</span>
          <span className="sparkline-value" style={{ color: '#94a3b8', fontSize: '11px' }}>
            — setpoint
          </span>
        </div>
        <div style={{ width: '100%', overflow: 'hidden' }}>
          <div ref={hostRef} className="sparkline-host" />
        </div>
      </div>
    </section>
  )
})
