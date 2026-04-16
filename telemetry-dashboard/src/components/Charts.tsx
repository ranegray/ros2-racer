import { memo, useMemo } from 'react'
import type { AlignedData } from 'uplot'
import type { Sample } from '../telemetry'
import { Sparkline } from './Sparkline'

type Props = { samples: Sample[] }

function series(samples: Sample[], pick: (s: Sample) => number): AlignedData {
  const xs = new Float64Array(samples.length)
  const ys = new Float64Array(samples.length)
  for (let i = 0; i < samples.length; i++) {
    xs[i] = samples[i].t
    ys[i] = pick(samples[i])
  }
  return [xs, ys]
}

export const Charts = memo(function Charts({ samples }: Props) {
  const front = useMemo(() => series(samples, (s) => s.front_distance), [samples])
  const heading = useMemo(() => series(samples, (s) => s.heading_error), [samples])
  const yaw = useMemo(() => series(samples, (s) => s.gyro_z), [samples])
  const accel = useMemo(() => series(samples, (s) => s.accel_mag), [samples])

  return (
    <section className="panel charts-panel">
      <div className="panel-header">
        <h2>Telemetry</h2>
        <span className="badge">{samples.length} samples</span>
      </div>
      <div className="charts">
        <Sparkline label="Front distance" unit="m" color="#22c55e" data={front} />
        <Sparkline label="Heading error" unit="rad" color="#c084fc" data={heading} />
        <Sparkline label="Yaw rate" unit="rad/s" color="#38bdf8" data={yaw} />
        <Sparkline label="|Accel|" unit="m/s²" color="#f59e0b" data={accel} />
      </div>
    </section>
  )
})
