import type { RacerTelemetry } from '../telemetry'

type Props = {
  connected: boolean
  latest: RacerTelemetry | null
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

function Tile({ label, value, tone }: { label: string; value: string; tone: Tone }) {
  return (
    <div className={`tile tile-${tone}`}>
      <span className="tile-label">{label}</span>
      <span className="tile-value">{value}</span>
    </div>
  )
}

export function StatusTiles({ connected, latest }: Props) {
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
    </div>
  )
}
