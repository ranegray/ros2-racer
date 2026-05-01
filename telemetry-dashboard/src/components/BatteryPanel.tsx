import { memo } from 'react'
import type { BatteryState } from '../telemetry'

type Props = {
  battery: BatteryState | null
  arrivedAt: number
  stalenessMs?: number
}

// 3S LiPo thresholds
const WARN_V = 11.4
const BAD_V  = 10.5

function voltageTone(v: number | undefined): 'ok' | 'warn' | 'bad' | 'idle' {
  if (v === undefined || !Number.isFinite(v) || v <= 0) return 'idle'
  if (v < BAD_V)  return 'bad'
  if (v < WARN_V) return 'warn'
  return 'ok'
}

const TONE_COLOR: Record<string, string> = {
  ok:   '#22c55e',
  warn: '#f59e0b',
  bad:  '#ef4444',
  idle: '#4b5563',
}

export const BatteryPanel = memo(function BatteryPanel({ battery, arrivedAt, stalenessMs = 5000 }: Props) {
  const now = Date.now()
  const stale = !battery || arrivedAt === 0 || now - arrivedAt > stalenessMs

  const pct  = battery && Number.isFinite(battery.percentage) ? battery.percentage : null
  const v    = battery && Number.isFinite(battery.voltage) && battery.voltage > 0 ? battery.voltage : null
  const amps = battery && Number.isFinite(battery.current) && battery.current >= 0 ? battery.current : null
  const tone = voltageTone(v ?? undefined)
  const barColor = TONE_COLOR[tone]

  // Clamp bar fill — use percentage if available, otherwise estimate from voltage
  let fillFrac = pct ?? null
  if (fillFrac === null && v !== null) {
    // Linear map: 10.5V=0% .. 12.6V=100% (3S LiPo)
    fillFrac = Math.max(0, Math.min(1, (v - 10.5) / (12.6 - 10.5)))
  }

  return (
    <section className="panel battery-panel">
      <div className="panel-header">
        <h2>Battery</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
          {stale ? 'no data' : `${v !== null ? v.toFixed(2) + ' V' : '—'}`}
        </span>
      </div>

      <div className="battery-bar-track">
        <div
          className="battery-bar-fill"
          style={{ width: fillFrac !== null ? `${fillFrac * 100}%` : '0%', background: barColor }}
        />
      </div>

      <div className="battery-stats">
        <div className="battery-stat">
          <span className="tile-label">Voltage</span>
          <span className="tile-value" style={{ color: barColor }}>
            {v !== null ? `${v.toFixed(2)} V` : '—'}
          </span>
        </div>
        <div className="battery-stat">
          <span className="tile-label">Charge</span>
          <span className="tile-value" style={{ color: barColor }}>
            {pct !== null ? `${Math.round(pct * 100)} %` : fillFrac !== null ? `~${Math.round(fillFrac * 100)} %` : '—'}
          </span>
        </div>
        <div className="battery-stat">
          <span className="tile-label">Current</span>
          <span className="tile-value">
            {amps !== null ? `${amps.toFixed(1)} A` : '—'}
          </span>
        </div>
      </div>
    </section>
  )
})
