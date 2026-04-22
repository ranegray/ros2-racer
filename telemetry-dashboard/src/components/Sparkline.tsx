import { useEffect, useRef } from 'react'
import uPlot, { type AlignedData, type Options } from 'uplot'
import 'uplot/dist/uPlot.min.css'

type Props = {
  label: string
  unit: string
  color: string
  data: AlignedData // [xs, ys]
}

export function Sparkline({ label, unit, color, data }: Props) {
  const hostRef = useRef<HTMLDivElement | null>(null)
  const plotRef = useRef<uPlot | null>(null)

  useEffect(() => {
    if (!hostRef.current) return
    const host = hostRef.current
    const opts: Options = {
      width: host.clientWidth || 400,
      height: 90,
      legend: { show: false },
      cursor: { show: false },
      scales: { x: { time: true }, y: { auto: true } },
      axes: [
        { show: false },
        {
          size: 44,
          stroke: getComputedStyle(document.documentElement).getPropertyValue('--text') || '#888',
          grid: { stroke: 'rgba(128,128,128,0.15)' },
          ticks: { show: false },
        },
      ],
      series: [
        {},
        { label, stroke: color, width: 1.5, points: { show: false } },
      ],
    }
    const plot = new uPlot(opts, data, host)
    plotRef.current = plot

    const ro = new ResizeObserver(() => {
      plot.setSize({ width: host.clientWidth, height: 90 })
    })
    ro.observe(host)

    return () => {
      ro.disconnect()
      plot.destroy()
      plotRef.current = null
    }
    // only initialize once
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  useEffect(() => {
    plotRef.current?.setData(data)
  }, [data])

  const latest = data[1]?.length ? data[1][data[1].length - 1] : null

  return (
    <div className="sparkline">
      <div className="sparkline-head">
        <span className="sparkline-label">{label}</span>
        <span className="sparkline-value" style={{ color }}>
          {latest == null || !Number.isFinite(latest) ? '—' : `${latest.toFixed(2)} ${unit}`}
        </span>
      </div>
      <div ref={hostRef} className="sparkline-host" />
    </div>
  )
}
