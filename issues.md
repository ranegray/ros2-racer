# `wall_nav_node` Review Findings

Date: 2026-04-22

## Findings

1. High: committed right-turn handling bypasses the node's own steering abstraction.

The lost-wall turn publishes `lost_turn_steering` raw and assumes `+2.0` means
"full-lock right" on the current inverted rover. Normal PD steering goes
through `steering_sign`, clamping, and bias, so the lost-turn path is internally
inconsistent and will break if `steering_sign` changes during rewiring or
retuning.

References:
- `src/autonomy/autonomy/wall_nav_node.py:116`
- `src/autonomy/autonomy/wall_nav_node.py:349`
- `src/autonomy/autonomy/wall_nav_node.py:418`

2. High: spike rejection is coupled directly to the wall-loss / corner state machine.

A scan flagged as a spike is immediately treated as `wall_lost_now`, which feeds
straight into the coast-then-turn recovery logic. That means the same mechanism
intended to reject bad measurements can also start a corner sequence from a
transient sensor glitch. Because recovery requires multiple good scans, one bad
jump can hold the node in lost mode longer than the measurement error itself.

References:
- `src/autonomy/autonomy/wall_nav_node.py:278`
- `src/autonomy/autonomy/wall_nav_node.py:307`
- `src/autonomy/autonomy/wall_nav_node.py:336`

3. Medium: single-beam fallback removes angle information exactly when geometry is ambiguous.

When the forward-diagonal ray is missing but the perpendicular ray remains, the
estimator forces `alpha = 0` and `D_ahead = b`. That disables angle feedback and
speed reduction exactly when corners, juts, or partial occlusions are likely.
A degraded-confidence mode would be safer than treating the wall as parallel.

References:
- `src/autonomy/autonomy/wall_nav_node.py:243`

4. Medium: PD state reset does not fully prevent a derivative kick on reacquisition.

Lost mode zeroes `prev_error` and `prev_d_error`, but it also sets
`prev_time = now`. On the first recovered scan, the derivative is still computed
against a synthetic zero-error prior sample. If the recovered error is sizable,
that can create a derivative spike right when the wall comes back into view.

References:
- `src/autonomy/autonomy/wall_nav_node.py:341`
- `src/autonomy/autonomy/wall_nav_node.py:401`

5. Low: `_right_wall_state` docstring does not match the implementation.

The docstring says both outputs are `NaN` only when both beams are missing, but
the code returns `(NaN, NaN)` whenever the perpendicular beam is missing, even
if the forward-diagonal beam is valid. That mismatch makes tuning and debugging
harder because the documented contract is not the actual one.

References:
- `src/autonomy/autonomy/wall_nav_node.py:218`
- `src/autonomy/autonomy/wall_nav_node.py:232`

## Assumptions

- The controller is intended to remain portable across steering inversion
  changes because `steering_sign` is explicitly documented for rewiring.
- False-positive corner commits are considered unacceptable; if this node is
  only meant for one tightly controlled hallway setup, some of these may be
  accepted tradeoffs instead of bugs.

## Suggested Fix Order

1. Decouple "spike rejected" from "wall lost" so bad scans do not trigger turn
   commits.
2. Route committed-turn steering through the same sign/clamp conventions as the
   normal steering path.
3. Re-seed derivative state on reacquisition instead of differentiating from a
   zeroed sample.
4. Revisit single-beam fallback behavior so it degrades more conservatively.
5. Update comments and docstrings to match the implemented behavior.

Resume this context with:  codex resume 019db741-e6f2-7bb2-ab8d-6a56d29861f0

