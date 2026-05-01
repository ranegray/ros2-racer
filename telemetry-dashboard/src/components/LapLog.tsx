import { memo } from 'react'
import { formatLap } from './StatusTiles'

type Props = {
  lapTimes: number[]   // total elapsed ms at each recorded split
  currentMs: number | null
  stopped: boolean
}

function fmtDelta(ms: number): string {
  const sign = ms >= 0 ? '+' : '-'
  return `${sign}${formatLap(Math.abs(ms))}`
}

export const LapLog = memo(function LapLog({ lapTimes, currentMs, stopped }: Props) {
  const hasSplits = lapTimes.length > 0
  const running = currentMs !== null && !stopped

  // Split durations: time between consecutive splits (or from start to first split)
  const splits = lapTimes.map((t, i) => t - (i === 0 ? 0 : lapTimes[i - 1]))

  return (
    <section className="panel lap-log-panel">
      <div className="panel-header">
        <h2>Lap Log</h2>
        <span className="badge">{lapTimes.length} split{lapTimes.length !== 1 ? 's' : ''}</span>
      </div>

      <p className="panel-desc">
        Click the Timer tile to stop and record a split. Click again to resume.
        Each row shows total elapsed, split time for that lap, and Δ vs the previous lap.
      </p>
      {!hasSplits && currentMs === null ? (
        <div className="battery-placeholder">No splits recorded yet</div>
      ) : (
        <div className="lap-log">
          {lapTimes.map((total, i) => {
            const split = splits[i]
            const prev = i > 0 ? splits[i - 1] : null
            const delta = prev !== null ? split - prev : null
            return (
              <div key={i} className="lap-row">
                <span className="lap-num">L{i + 1}</span>
                <span className="lap-total">{formatLap(total)}</span>
                <span className="lap-split">{formatLap(split)}</span>
                {delta !== null ? (
                  <span className={`lap-delta ${delta < 0 ? 'lap-delta-faster' : delta > 0 ? 'lap-delta-slower' : ''}`}>
                    {fmtDelta(delta)}
                  </span>
                ) : (
                  <span className="lap-delta" />
                )}
              </div>
            )
          })}

          {/* Current in-progress split */}
          {currentMs !== null && (() => {
            const splitStart = lapTimes.length > 0 ? lapTimes[lapTimes.length - 1] : 0
            const currentSplit = currentMs - splitStart
            return (
              <div className={`lap-row lap-row-live ${stopped ? 'lap-row-stopped' : ''}`}>
                <span className="lap-num">L{lapTimes.length + 1}</span>
                <span className="lap-total">{formatLap(currentMs)}</span>
                <span className="lap-split">{running ? `${formatLap(currentSplit)}…` : formatLap(currentSplit)}</span>
                <span className="lap-delta" />
              </div>
            )
          })()}
        </div>
      )}
    </section>
  )
})
