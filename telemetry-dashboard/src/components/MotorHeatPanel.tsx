import { memo } from 'react'
import type { MotorTemps } from '../telemetry'

type Props = {
  temps: MotorTemps | null
  arrivedAt: number
  stalenessMs?: number
}

const WARN_C = 60
const BAD_C  = 80
const MAX_C  = 120  // bar scale ceiling

function tempTone(c: number): 'ok' | 'warn' | 'bad' {
  if (c >= BAD_C)  return 'bad'
  if (c >= WARN_C) return 'warn'
  return 'ok'
}

const TONE_COLOR: Record<string, string> = {
  ok:   '#22c55e',
  warn: '#f59e0b',
  bad:  '#ef4444',
  idle: '#4b5563',
}

export const MotorHeatPanel = memo(function MotorHeatPanel({ temps, arrivedAt, stalenessMs = 3000 }: Props) {
  const now = Date.now()
  const stale = !temps || arrivedAt === 0 || now - arrivedAt > stalenessMs
  const motors = Array.from(temps?.data ?? [])

  return (
    <section className="panel motor-heat-panel">
      <div className="panel-header">
        <h2>Motor Heat</h2>
        <span className={`badge ${stale ? 'badge-stale' : 'badge-live'}`}>
          {stale ? 'no ESC data' : `${motors.length} ESC${motors.length !== 1 ? 's' : ''}`}
        </span>
      </div>

      {stale ? (
        <div className="battery-placeholder">Waiting for /motor_temps…</div>
      ) : motors.length === 0 ? (
        <div className="battery-placeholder">No ESC data in message</div>
      ) : (
        <div className="motor-grid">
          {motors.map((c, i) => {
            const tone = c <= 0 ? 'idle' : tempTone(c)
            const color = TONE_COLOR[tone]
            const fill = Math.min(1, c / MAX_C)
            return (
              <div key={i} className="motor-row">
                <span className="motor-label">M{i + 1}</span>
                <div className="motor-bar-track">
                  <div className="motor-bar-fill" style={{ width: `${fill * 100}%`, background: color }} />
                </div>
                <span className="motor-temp" style={{ color }}>{c <= 0 ? 'n/a' : `${c.toFixed(0)}°`}</span>
              </div>
            )
          })}
        </div>
      )}
    </section>
  )
})
