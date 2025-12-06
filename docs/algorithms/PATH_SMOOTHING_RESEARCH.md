# Path Smoothing: Cubic Splines

## Why Cubic Splines?
**C^2^ Continuous** - Smooth motion, no jerks
**Efficient** - O(n) computation, 100+ Hz capable
**Industry Stable** - Used in robotics/animation/CAD
**Numerical Stable** - Handles 5 to 1000+ waypoints
**Natural Behavior** - Intutive, smooth curves

## Algorithm: Natural Cubic Spline

Given waypoints: P~0~, P~1~, ..., P~n~
Create cubic segments: S(t) = a + b*t + c*t^2^ + d*t^3^

Connected with continuous position, velocity, acceleration

## Alternatives (Why NOT selected)
- Bezier: Less efficient, degree grows with points
- B-Splines: Overkill, more complex
- Linear: Not smooth, bad for robots