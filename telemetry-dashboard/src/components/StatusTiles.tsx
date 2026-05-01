import { memo } from 'react'
import type { RacerTelemetry } from '../telemetry'

const VPC_WARN = 3.75
const VPC_BAD  = 3.5

function detectCells(v: number): number { return Math.max(1, Math.round(v / 3.7)) }

function voltageTone(v: number | null): Tone {
  if (v === null || !Number.isFinite(v) || v <= 0) return 'idle'
  const c = v / detectCells(v)
  if (c < VPC_BAD)  return 'bad'
  if (c < VPC_WARN) return 'warn'
  return 'ok'
}

function speedTone(measured: number | null, target: number | null): Tone {
  if (measured === null || target === null) return 'idle'
  if (target > 0.1 && measured < 0.05) return 'bad'
  if (target > 0.1 && (target - measured) > 0.08) return 'warn'
  return 'ok'
}

type Props = {
  connected: boolean
  latest: RacerTelemetry | null
  lapMs: number | null
  lapStopped: boolean
  onLapToggle: () => void
  onLapReset: () => void
  voltage: number | null
  speedMeasured: number | null
  speedTarget: number | null
}

type Tone = 'ok' | 'warn' | 'bad' | 'idle'

function armedTone(armed: boolean | undefined): Tone {
  if (armed === undefined) return 'idle'
  return armed ? 'warn' : 'ok'
}

function obstacleTone(v: string | undefined): Tone {
  if (!v || v === 'NONE') return 'ok'
  return 'bad'
}

function frontDistanceTone(v: number | undefined): Tone {
  if (v === undefined || !Number.isFinite(v) || v <= 0) return 'idle'
  if (v < 0.4) return 'bad'
  if (v < 1.0) return 'warn'
  return 'ok'
}

function stateTone(v: string | undefined): Tone {
  if (!v) return 'idle'
  if (v === 'STOPPED') return 'bad'
  if (v === 'NAVIGATING') return 'warn'
  return 'ok'
}

export function formatLap(ms: number): string {
  const totalS = Math.floor(ms / 1000)
  const m = Math.floor(totalS / 60)
  const s = totalS % 60
  const tenths = Math.floor((ms % 1000) / 100)
  return `${m}:${s.toString().padStart(2, '0')}.${tenths}`
}

function Tile({ label, value, tone }: { label: string; value: string; tone: Tone }) {
  return (
    <div className={`tile tile-${tone}`}>
      <span className="tile-label">{label}</span>
      <span className="tile-value" style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>{value}</span>
    </div>
  )
}

export const StatusTiles = memo(function StatusTiles({ connected, latest, lapMs, lapStopped, onLapToggle, onLapReset, voltage, speedMeasured, speedTarget }: Props) {
  const tone: Tone = connected ? 'ok' : 'bad'

  const lapTone: string = lapMs === null
    ? 'idle'
    : lapStopped
      ? 'warn'
      : lapMs >= 180_000 ? 'bad' : 'ok'

  return (
    <div className="tiles">
      <Tile label="Link" value={connected ? 'ONLINE' : 'OFFLINE'} tone={tone} />
      <Tile
        label="Armed"
        value={latest ? (latest.armed ? 'ARMED' : 'SAFE') : '—'}
        tone={armedTone(latest?.armed)}
      />
      <Tile
        label="State"
        value={latest?.internal_state || '—'}
        tone={stateTone(latest?.internal_state)}
      />
      <Tile
        label="Obstacle"
        value={latest?.obstacle_detection || '—'}
        tone={obstacleTone(latest?.obstacle_detection)}
      />
      <Tile
        label="Front"
        value={
          latest && Number.isFinite(latest.front_distance) && latest.front_distance > 0
            ? `${latest.front_distance.toFixed(2)} m`
            : '—'
        }
        tone={frontDistanceTone(latest?.front_distance)}
      />
      <Tile
        label="Voltage"
        value={voltage !== null && voltage > 0 ? `${voltage.toFixed(2)} V` : '—'}
        tone={voltageTone(voltage)}
      />
      <Tile
        label="Speed"
        value={
          speedMeasured !== null && speedTarget !== null
            ? `${speedMeasured.toFixed(2)} / ${speedTarget.toFixed(2)}`
            : '—'
        }
        tone={speedTone(speedMeasured, speedTarget)}
      />
      <div
        className={`tile tile-${lapTone} tile-lap`}
        onClick={lapMs !== null ? onLapToggle : undefined}
        onDoubleClick={lapMs !== null ? onLapReset : undefined}
        title={
          lapMs === null
            ? 'Waiting for armed + motion…'
            : lapStopped
              ? 'Click to resume · Double-click to reset'
              : 'Click to stop · Double-click to reset'
        }
      >
        <span className="tile-label">Timer{lapStopped ? ' ⏸' : ''}</span>
        <span className="tile-value">{lapMs !== null ? formatLap(lapMs) : '—'}</span>
      </div>
    </div>
  )
})
