import { memo, useMemo, useState } from 'react'
import type { RacerTelemetry } from '../telemetry'

type Props = { msg: RacerTelemetry | null }

export const RawJson = memo(function RawJson({ msg }: Props) {
  const [open, setOpen] = useState(false)

  // Only stringify when the user actually has the panel open — otherwise we
  // spend 10 Hz of main-thread time serializing text that nobody sees.
  const text = useMemo(() => {
    if (!open) return ''
    if (!msg) return 'Waiting for data…'
    return JSON.stringify(msg, null, 2)
  }, [msg, open])

  return (
    <details
      className="raw-json"
      open={open}
      onToggle={(e) => setOpen((e.target as HTMLDetailsElement).open)}
    >
      <summary>Raw message</summary>
      {open && <pre>{text}</pre>}
    </details>
  )
})
