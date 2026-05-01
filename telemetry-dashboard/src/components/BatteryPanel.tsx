import { memo, useMemo } from 'react'
import type { AlignedData } from 'uplot'
import type { BatterySample, BatteryState } from '../telemetry'
import { Sparkline } from './Sparkline'

type Props = {
  battery: BatteryState | null
  arrivedAt: number
  samples?: BatterySample[]
  stalenessMs?: number
}

// Per-cell LiPo thresholds (works for 2S, 3S, or any S-count)
const VPC_FULL  = 4.2   // fully charged
const VPC_WARN  = 3.75  // getting low
const VPC_BAD   = 3.5   // critically low
const VPC_EMPTY = 3.3   // cutoff

function detectCells(v: number): number {
  return Math.max(1, Math.round(v / 3.7))
}

function vpc(v: number): number {
  return v / detectCells(v)
}

function voltageTone(v: number | undefined): 'ok' | 'warn' | 'bad' | 'idle' {
  if (v === undefined || !Number.isFinite(v) || v <= 0) return 'idle'
  const c = vpc(v)
  if (c < VPC_BAD)  return 'bad'
  if (c < VPC_WARN) return 'warn'
  return 'ok'
}

const TONE_COLOR: Record<string, string> = {
  ok:   '#22c55e',
  warn: '#f59e0b',
  bad:  '#ef4444',
  idle: '#4b5563',
}

function toSeries(samples: BatterySample[], pick: (s: BatterySample) => number): AlignedData {
  const xs = new Float64Array(samples.length)
  const ys = new Float64Array(samples.length)
  for (let i = 0; i < samples.length; i++) {
    xs[i] = samples[i].t
    ys[i] = pick(samples[i])
  }
  return [xs, ys]
}

export const BatteryPanel = memo(function BatteryPanel({ battery, arrivedAt, samples = [], stalenessMs = 3000 }: Props) {
  const now = Date.now()
  const stale = !battery || arrivedAt === 0 || now - arrivedAt > stalenessMs

  const v    = battery && Number.isFinite(battery.voltage)    && battery.voltage > 0  ? battery.voltage    : null
  const amps = battery && Number.isFinite(battery.current)    && battery.current >= 0 ? battery.current    : null
  const pct  = battery && Number.isFinite(battery.percentage) && battery.percentage >= 0 ? battery.percentage : null

  const tone     = voltageTone(v ?? undefined)
  const barColor = TONE_COLOR[tone]
  const cells    = v !== null ? detectCells(v) : null
  const cellV    = v !== null && cells !== null ? (v / cells).toFixed(2) : null
  const watts    = v !== null && amps !== null ? v * amps : null

  const fillFrac = pct !== null
    ? pct
    : v !== null
      ? Math.max(0, Math.min(1, (vpc(v) - VPC_EMPTY) / (VPC_FULL - VPC_EMPTY)))
      : null

  const badgeText = stale
    ? 'no data'
    : `${cells !== null ? `${cells}S · ` : ''}${v !== null ? v.toFixed(2) + ' V' : '—'}`

  // Session-low and sag: derived from the rolling sample buffer.
  // Sag = peak seen - current voltage; a large sag under load is the
  // clearest indicator of a battery with high internal resistance.
  const validSamples = samples.filter(s => s.voltage > 0)
  const sessionLowV  = validSamples.length ? Math.min(...validSamples.map(s => s.voltage)) : null
  const sessionHighV = validSamples.length ? Math.max(...validSamples.map(s => s.voltage)) : null
  const sagV = sessionHighV !== null && v !== null ? sessionHighV - v : null
  const sagTone = sagV !== null && sagV > 0.3 ? 'bad' : sagV !== null && sagV > 0.1 ? 'warn' : 'ok'

  const voltageData = useMemo(() => toSeries(samples, (s) => s.voltage), [samples])
  const currentData = useMemo(() => toSeries(samples, (s) => s.current), [samples])
  const powerData   = useMemo(() => toSeries(samples, (s) => s.power),   [samples])

  return (
    <section className="panel battery-panel">
      <div className="panel-header">
        <h2>Battery</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>{badgeText}</span>
      </div>

      {stale ? (
        <div className="battery-placeholder">Waiting for /battery_state…</div>
      ) : (
        <>
          <div className="battery-bar-track">
            <div
              className="battery-bar-fill"
              style={{ width: fillFrac !== null ? `${(fillFrac * 100).toFixed(1)}%` : '0%', background: barColor }}
            />
          </div>

          <div className="battery-stats">
            <div className="battery-stat">
              <span className="tile-label">Cell V</span>
              <span className="tile-value" style={{ color: barColor }}>
                {cellV !== null ? `${cellV} V` : '—'}
              </span>
            </div>
            <div className="battery-stat">
              <span className="tile-label">Charge</span>
              <span className="tile-value" style={{ color: barColor }}>
                {pct !== null
                  ? `${Math.round(pct * 100)} %`
                  : fillFrac !== null
                    ? `~${Math.round(fillFrac * 100)} %`
                    : '—'}
              </span>
            </div>
            <div className="battery-stat">
              <span className="tile-label">Current</span>
              <span className="tile-value">{amps !== null ? `${amps.toFixed(1)} A` : '—'}</span>
            </div>
            <div className="battery-stat">
              <span className="tile-label">Power</span>
              <span className="tile-value">{watts !== null ? `${watts.toFixed(1)} W` : '—'}</span>
            </div>
          </div>

          <div className="battery-stats">
            <div className="battery-stat">
              <span className="tile-label">Session Low</span>
              <span className="tile-value" style={{ color: sessionLowV !== null ? voltageTone(sessionLowV) !== 'ok' ? TONE_COLOR[voltageTone(sessionLowV)] : undefined : undefined }}>
                {sessionLowV !== null ? `${sessionLowV.toFixed(2)} V` : '—'}
              </span>
            </div>
            <div className="battery-stat">
              <span className="tile-label">Sag (peak–now)</span>
              <span className="tile-value" style={{ color: sagV !== null && sagV > 0.1 ? TONE_COLOR[sagTone] : undefined }}>
                {sagV !== null ? `${sagV.toFixed(2)} V` : '—'}
              </span>
            </div>
            <div className="battery-stat" style={{ gridColumn: 'span 2' }}>
              <span className="tile-label">Session High</span>
              <span className="tile-value">
                {sessionHighV !== null ? `${sessionHighV.toFixed(2)} V` : '—'}
              </span>
            </div>
          </div>
        </>
      )}

      {/* Always mounted so uPlot doesn't re-initialize on first data arrival */}
      <div className="charts" style={stale ? { display: 'none' } : undefined}>
        <Sparkline label="Voltage" unit="V" color={barColor} data={voltageData} />
        <Sparkline label="Current" unit="A" color="#38bdf8" data={currentData} />
        <Sparkline label="Power"   unit="W" color="#c084fc" data={powerData} />
      </div>
    </section>
  )
})
