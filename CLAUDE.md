# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

---

# tbh26 — LVAD Biventricular Drive Unit

## Mechanism type
**Crank-and-slotted-arm with radial slot** (NOT a scotch yoke).
The motor crank pin sits in a slot that runs *along* the paddle arm (radial direction).
This gives a **quick-return** characteristic — the mechanism is a crank-rocker.

The correct kinematic formula is:
```
theta(phi) = arctan( x*sin(phi) / (a - x*cos(phi)) )
```
NOT `arcsin(r*sin(phi))` — that formula only applies to a translating scotch yoke.

## Key geometry (Appendix D)
| Parameter | Value | Meaning |
|-----------|-------|---------|
| `x` | 10 mm | Crank arm length |
| `a` | 21 mm | Crank centre to fulcrum distance |
| `r = x/a` | 0.476 | Crank ratio |
| `alpha = arcsin(r)` | ≈ 28.4° | Max paddle deflection |
| `phi_pk = arccos(r)` | ≈ 61.6° | Crank angle at max paddle angle (**not 90°**) |
| `rpm_max` | 234 rpm | Max motor speed |

## Quick-return ratio
```
(a + x) / (a - x) = 31/11 ≈ 2.82
```
LV compression stroke is ~2.82× faster than RV compression stroke.
Peak angular velocity at neutral: LV side ≈ 1271°/s, RV side ≈ 451°/s.

## Biventricular operation
The paddle compresses two blood bags alternately each revolution:
- **LV**: paddle sweeps 0° → +alpha (crank 0° → 61.6°, positive stroke)
- **RV**: paddle sweeps 0° → −alpha (crank 180° → 241.6°, negative stroke)
- At theta = 0° (neutral), **neither** bag is compressed
- The paddle starts returning from max angle **before** the crank reaches 90°

## Overlap parameter `b` [0, 1]
`b` is a **geometric** bag overlap fraction — NOT a time fraction of the cycle.
- `b = 0`: each bag first contacts the paddle exactly at theta = 0°
- `b = 1`: RV bag starts compressing the instant LV reaches full compression (+alpha)
- Each bag physically encroaches `b * alpha` degrees past the neutral position

Ejection window boundaries (crank angle):
```
phi_RV_start = 180 - arcsin(sin(b*alpha)/r) - b*alpha
phi_LV_start = 360 - arcsin(sin(b*alpha)/r) - b*alpha
```

## Flow rate model
```
Q [mL/s] = K_geom * dtheta/dt [rad/s] / 1000
K_geom   = w * (f_contact * L)^2 / 2   [mm³/rad]
```
Flow is **directly proportional to angular velocity** — peak flow at the START of each
compression stroke (not at peak angle where dtheta/dt = 0).

### Paddle geometry defaults
| Parameter | Value | Meaning |
|-----------|-------|---------|
| `L` | 40 mm | Paddle length (radial from pivot) |
| `w` | 66 mm | Paddle width |
| `f_contact` | 0.7 | Fraction of paddle length compressing bag |

## Files
| File | Description |
|------|-------------|
| `paddle_angle_plot.m` | Main kinematic + flow rate plots (2 figures) |
| `tbh26_mechanism_v1.m` | Mechanism design analysis |
| `appendix_d_ejection_calc.m` | Ejection calculations (Appendix D) |
| `tbh27_mechanism_archived.m` | Previous iteration, archived |
