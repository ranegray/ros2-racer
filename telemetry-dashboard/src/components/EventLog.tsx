import { memo } from 'react'
import type { RosLogMsg } from '../telemetry'
import { stampToSeconds } from '../telemetry'

type Props = { events: RosLogMsg[] }

const LEVEL_LABEL: Record<number, string> = { 10: 'DBG', 20: 'INF', 30: 'WRN', 40: 'ERR', 50: 'FAT' }
const LEVEL_COLOR: Record<number, string> = {
  10: '#4b5563',
  20: '#94a3b8',
  30: '#f59e0b',
  40: '#ef4444',
  50: '#ef4444',
}

function levelLabel(l: number) { return LEVEL_LABEL[l] ?? 'INF' }
function levelColor(l: number) { return LEVEL_COLOR[l] ?? LEVEL_COLOR[20] }

function fmtTime(stamp: RosLogMsg['stamp']): string {
  const d = new Date(stampToSeconds(stamp) * 1000)
  return `${d.getHours().toString().padStart(2,'0')}:${d.getMinutes().toString().padStart(2,'0')}:${d.getSeconds().toString().padStart(2,'0')}`
}

export const EventLog = memo(function EventLog({ events }: Props) {
  const hasErrors = events.some(e => e.level >= 40)

  return (
    <section className="panel event-log-panel">
      <div className="panel-header">
        <h2>MAVLink Events</h2>
        <span className={`badge ${events.length === 0 ? '' : hasErrors ? 'badge-stale' : 'badge-live'}`}>
          {events.length === 0 ? 'no events' : `${events.length} msgs`}
        </span>
      </div>
      <p className="panel-desc">
        ArduPilot STATUSTEXT stream — failsafes, arm/disarm, sensor faults, and mode changes.
        Sensor health bit-flips from SYS_STATUS are injected as ERR/INF rows.
      </p>
      {events.length === 0 ? (
        <div className="battery-placeholder">Waiting for /mavlink_events…</div>
      ) : (
        <div className="event-list">
          {[...events].reverse().map((e, i) => (
            <div key={i} className="event-row">
              <span className="event-time">{fmtTime(e.stamp)}</span>
              <span className="event-level" style={{ color: levelColor(e.level) }}>{levelLabel(e.level)}</span>
              <span className="event-msg" style={{ color: e.level >= 40 ? levelColor(e.level) : undefined }}>{e.msg}</span>
            </div>
          ))}
        </div>
      )}
    </section>
  )
})
