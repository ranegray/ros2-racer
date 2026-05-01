import { memo } from 'react'
import type { RacerTelemetry } from '../telemetry'

type Props = {
  connected: boolean
  latest: RacerTelemetry | null
  lapMs: number | null
  onLapReset: () => void
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

function formatLap(ms: number): string {
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
      <span className="tile-value">{value}</span>
    </div>
  )
}

export const StatusTiles = memo(function StatusTiles({ connected, latest, lapMs, onLapReset }: Props) {
  const tone: Tone = connected ? 'ok' : 'bad'
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
      <div
        className={`tile ${lapMs !== null ? 'tile-ok' : 'tile-idle'} tile-lap`}
        onClick={lapMs !== null ? onLapReset : undefined}
        title={lapMs !== null ? 'Click to reset' : 'Waiting for motion…'}
      >
        <span className="tile-label">Lap</span>
        <span className="tile-value">{lapMs !== null ? formatLap(lapMs) : '—'}</span>
      </div>
    </div>
  )
})
