# Pure Pursuit Controller

## Why Pure Pursuit?
**Proven** - 25+ years in autonomous vehicles
**Diff Drive Suitable** - Works perfectly with TurtleBot3
**Simple** - ~30-50 lines of core algorithm
**Intuitive** - Lookahead distance parameter easy to tune
** Robust** - Handles disturbances naturally

## Algorithm: Lookahead-Based Steering
Simple idea: Robot looks L meters ahead and steers toward that point
Key formula:
```math
\text{Steering Angle} = \frac{2 \cdot \sin(\text{angle\_error})}{L}
```

Advantages:
- Geometrically intuitive
- Works at any speed
- Very fast (< 1 ms)
- Self-correcting

## Alternatives (Why NOT selected)
- Stanley: Better for cars, overkill for diff drive
- LQR: Computationally expensive
- PID: Less robust to trajectory changes
