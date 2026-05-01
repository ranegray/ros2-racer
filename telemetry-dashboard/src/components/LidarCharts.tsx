import { memo, useMemo } from 'react'
import type { AlignedData } from 'uplot'
import type { LidarSample, Sample } from '../telemetry'
import { Sparkline } from './Sparkline'

type Props = {
  lidarSamples: LidarSample[]
  telSamples: Sample[]
}

function toLidar(samples: LidarSample[], pick: (s: LidarSample) => number): AlignedData {
  const xs = new Float64Array(samples.length)
  const ys = new Float64Array(samples.length)
  for (let i = 0; i < samples.length; i++) { xs[i] = samples[i].t; ys[i] = pick(samples[i]) }
  return [xs, ys]
}

function toTel(samples: Sample[], pick: (s: Sample) => number): AlignedData {
  const xs = new Float64Array(samples.length)
  const ys = new Float64Array(samples.length)
  for (let i = 0; i < samples.length; i++) { xs[i] = samples[i].t; ys[i] = pick(samples[i]) }
  return [xs, ys]
}

export const LidarCharts = memo(function LidarCharts({ lidarSamples, telSamples }: Props) {
  const fwdMean   = useMemo(() => toLidar(lidarSamples, (s) => s.fwdMean),         [lidarSamples])
  const minRange  = useMemo(() => toLidar(lidarSamples, (s) => s.minRange),         [lidarSamples])
  const validPct  = useMemo(() => toLidar(lidarSamples, (s) => s.validPct * 100),   [lidarSamples])
  const frontDist = useMemo(() => toTel(telSamples,    (s) => s.front_distance),    [telSamples])

  return (
    <section className="panel lidar-charts-panel">
      <div className="panel-header">
        <h2>Lidar Analysis</h2>
        <span className="badge">{lidarSamples.length} scans</span>
      </div>
      <div className="charts">
        <Sparkline label="Fwd sector (raw scan)"   unit="m" color="#22c55e" data={fwdMean} />
        <Sparkline label="Front dist (controller)" unit="m" color="#f59e0b" data={frontDist} />
        <Sparkline label="Min range (raw scan)"    unit="m" color="#38bdf8" data={minRange} />
        <Sparkline label="Valid readings"          unit="%" color="#94a3b8" data={validPct} />
      </div>
    </section>
  )
})
