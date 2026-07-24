# CLAUDE.md

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

# tbh26 — LVAD Biventricular Drive Unit

## Critical: mechanism is NOT a scotch yoke
The slot is radial (along the arm). The formula `arcsin(r*sin(phi))` is wrong for this geometry — it only applies to a translating yoke. The correct formula is in the code header. Don't revert to arcsin.

## Quick-return ratio (not stated in code)
`(a + x) / (a - x)` — LV stroke is faster than RV by this ratio.
Current params (x=9.9mm, a=28mm): QR ≈ 2.09 (straight slot), ~1.5 (teardrop r1=0.2x).
Earlier params (x=7.6mm, a=22.4mm): QR ≈ 2.03.

## Flow rate contact zone
`L_contact` is measured **from the tip** of the paddle inward, not from the pivot.
Contact zone: radius `(L - L_contact)` to `L`. This matters because tip contact produces more flow per mm than pivot-end contact (larger moment arm). `K_geom = w * (L² − (L − L_contact)²) / 2`.

## Torque model
`T_p = F_total × r_moment` where `F_total = p × A_contact + F_e` (constant during ejection).
`r_moment = L − L_contact/2` (midpoint of contact zone from pivot, in metres).
`A_contact = w × L_contact` in m². Overlap b shifts torque onset via the existing ejection masks — no extra logic needed.

LV and RV now have **separate pressures**: `p_LV_mmHg` and `p_RV_mmHg` (converted via `133.322 Pa/mmHg`), giving `F_total_LV` and `F_total_RV`. Typical values: LV = 120 mmHg, RV = 25 mmHg. Each is applied only during its own ejection mask.

## `b` parameter
`b` is a **geometric** bag overlap fraction in **[0, 0.5]**, not a time fraction. It was previously misimplemented as `b*T` — don't repeat that mistake.

Physical meaning: **fill at θ=0 = (1−b)×100%**. b=0 → both bags 100% full at neutral; b=0.5 → both at 50%.
- `gamma = α·b/(1−b)` — contact extension angle (deg); bag first contacts paddle at θ = −γ for LV, +γ for RV
- `phi_LV_start = 360 + γ − arcsin(sin(γ)/r)` — crank angle where θ = −γ on return stroke
- `phi_RV_start = 180 − γ − arcsin(sin(γ)/r)` — symmetric for RV
- Stroke volume = K_geom · α/(1−b) · π/180 — the sweep from −γ to +α
- b = 0.5 is the maximum: γ = α, bags span the full ±α stroke; b > 0.5 is undefined (sin(γ)/r > 1)
- **RV peak crank angle = 360° − phi_pk ≈ 298.4°**, NOT 180°+phi_pk. Using 180°+phi_pk is a common mistake — the mechanism is asymmetric.

## LV vs RV asymmetry
Same stroke volume per ventricle, but different flow profiles due to quick-return:
- LV ejection crank span ≈ phi_pk + delta + b·α (short, fast)
- RV ejection crank span ≈ (360−phi_pk) − phi_RV_start (long, slow — ~81% longer for b=0.1)
- Peak Q_LV / Peak Q_RV = (a+x)/(a−x) ≈ 2.82 (quick-return ratio)
- Total CO per ventricle is equal; the asymmetry is in the flow *profile*, not volume

## `lv_fast` parameter
All `teardrop_` files have `lv_fast = true` (boolean). Controls which physical stroke the LV bag sits on:
- `true` → LV on fast stroke (phi~0, positive dtheta). Physiologically natural. Higher peak torque: `T_g_peak ∝ p_LV × x/(a−x)`.
- `false` → LV on slow stroke (phi~180). Lower peak torque (~58% less for 120/25 mmHg): `T_g_peak ∝ p_LV × x/(a+x)`. Better for motor sizing.
- Work per cycle is identical either way; only the peak torque/power changes.
- When `lv_fast` is toggled, the ejection masks, flow signs, Tg signs, plot shading, and Figure 3 labels all update automatically.
- `lv_fast` is meaningless at QR = 1 (both strokes symmetric).

## Gearbox torque full-cycle formula
`T_g(φ) = T_p · x · (a·cos(φ) − x) / (a² − 2ax·cos(φ) + x²)` (virtual work derivation)
- LV peak at φ=0: `T_g = x·T_p/(a−x)` (maximum — pin closest to pivot)
- RV peak at φ=180°: `T_g = x·T_p/(a+x)` (smaller — pin farthest from pivot)
- Zero at φ=phi_pk and φ=360−phi_pk (paddle momentarily stationary)

## Power chain
`omega_gb` = crank shaft speed = 2π·rpm_max/60 (NOT motor speed)
`omega_motor = omega_gb × GR` (motor shaft, ~14,508 rpm at rpm_max=234)
`P_mech = Tm × omega_motor` (mechanical power at motor shaft)
`P_elec = P_mech / e_motor` (electrical input power)

## Teardrop slot variant
Reshapes the **near-pivot end** of the radial slot from a sharp point into a
teardrop (rounded arc, tangent to the two straight sides) to reduce the
quick-return ratio. Full dynamics (kinematics, flow, torque, power) are
implemented in `paddle_teardrop_plot.m`.

### `r1` (crank pin radius) vs `r2` (teardrop/wall radius)
The crank pin has a real physical radius — it isn't a point. The pin's
**centre** does not ride on the slot wall itself; it stays a constant
distance `r1` inside it. So:
- `r1` — crank pin radius (mm), **fixed** (a manufactured constant, e.g. 2mm).
- `r2` — teardrop radius (mm), the **actual rounded wall** of the slot — this
  is the free design/slider variable (was previously just called `r1`,
  before this distinction existed).
- `r_eff = r2 − r1` — the pin **centre's** effective arc radius. This is what
  actually drives `theta(phi)` (i.e. `teardrop_theta(phi, x, a, r_eff)`), not
  `r2` directly — the wall is what you cut, `r_eff` is what the kinematics see.
- `r2 = r1` (smallest wall that can even contain the pin) gives `r_eff = 0` —
  i.e. **zero teardrop effect**, identical to the pure straight slot. Any real
  quick-return benefit requires `r2 > r1` by a real margin, not just meeting
  the floor.
- For the **drawn/machined wall** (not the kinematics): it's the `r_eff`-arc
  geometry scaled outward by `k = r2/r_eff` about the arc centre `ci` — same
  tangent angles, concentric `r2` arc, apex pushed out past `a+x` (a finite
  pin can never get its centre all the way into a mathematical point — see
  `teardrop_viz.m`, which also caps the far tip with its own `r1` fillet for
  the same reason: at `phi=180` the pin centre sits at `(0,a+x)`, tangent to
  *both* wall lines at once, like a ball wedged in a V).
- **Not all teardrop_ files have this split yet.** Currently applied in
  `teardrop_plot.m`, `teardrop_optimise.m`, `teardrop_viz.m`, `teardrop_sweep.m`
  (all four non-double teardrop_ files). The `teardrop_double_*` variants
  still use the older model where the single arc-radius parameter feeds
  `teardrop_theta` directly (i.e. what is now `r_eff`, with no separate pin
  radius) — treat their "r1" as `r_eff`, not as a wall/pin radius, until
  they're updated the same way.

Geometry (centreline = original slot axis through O4; all in terms of the
kinematic `r_eff`, unless computing the drawn wall as above):
- `y = a − x` — bottom of the r_eff-circle, distance from O4
- `c = y + r_eff` — r_eff-circle centre, distance from O4
- apex sits at `a + x` from O4 (teardrop span `z = 2x`, motor centre O2 at the
  midpoint of `z`)
- `y = a − x` (⟺ `a = y + x`) is required for continuity — it's what makes
  `theta(0) = theta(180) = 0` hold, same as the straight slot. (Without it the
  geometry is over-constrained.)

`R(phi) = sqrt(a² + x² − 2ax·cos(phi))` (pure crank fact, unchanged by slot shape):
- `R ≤ R_T` → pin centre rides the **r_eff arc** → `theta = theta_orig − beta(arc)`
- `R > R_T` → pin centre rides the **tangent line** → `theta = theta_orig − beta(tan)`
- `beta` = angle of P off the centreline in the body frame (two-circle
  intersection for the arc branch, quadratic tangent-point solve for the line
  branch — see `teardrop_theta()`)
- `R_T` = distance O4→tangent point between the r_eff-circle and the line from the apex
- Valid range: `0 ≤ r_eff < x`. `r_eff = 0` recovers the original straight-slot
  `theta(phi)` exactly; `r_eff → x` degenerates to a full circle (no straight side)

**Anti-symmetry is preserved**: `theta(360−phi) = −theta(phi)` still holds for
any `r_eff`, so `theta_min = −alpha_new` at `phi = 360 − phipk_new`, and the
existing stroke-volume formula `SV = K_geom·α/(1−b)·π/180` carries over
unchanged — just substitute `alpha_new`.

**The closed-form `gamma`/`phi_LV_start`/`phi_RV_start` formulas above are
specific to the original `theta(phi)` and don't generalise.** The teardrop
file finds them numerically by inverting `theta(phi)` — see
`ejection_windows()` / `solve_phi_for_theta()`. (Verified to reproduce the
closed-form result at `b=0.5`, where `gamma=alpha` and both windows collapse
to `phipk`/`360−phipk`.)

### r_eff sweep results (OLD params: x=7.6mm, a=22.4mm, b=0.5)
Run before the r1/r2 split existed — the table's "r1" column is what is now
`r_eff` (the arc radius fed directly to `teardrop_theta`), with no separate
pin radius. Re-run under the current model as `r2 = r_eff + r1` for a given
pin radius `r1`.

| r_eff | α (deg) | QR ratio | SV (mL) | CO (L/min) |
|---|---|---|---|---|
| original | 19.83 | 2.03 | 34.27 | 9.94 |
| 0.2x (1.52mm) | 17.36 | 1.49 | 30.00 | 8.70 |
| 0.4x (3.04mm) | 14.73 | 1.15 | 25.46 | 7.38 |
| 0.6x (4.56mm) | 11.80 | 0.89 | 20.39 | 5.91 |
| 0.8x (6.08mm) | 8.20 | 0.69 | 14.16 | 4.11 |

- `r_eff` is a strong lever: even `r_eff=0.2x` cuts the QR ratio by ~26% but costs ~12% of α/SV/CO (SV scales linearly with α).
- QR ratio **inverts around r_eff≈0.5x** — past that point the φ≈0 side becomes the *slower* side.
- **QR and α are coupled** — the teardrop arc both redistributes speed between strokes AND truncates total angular travel. Reducing QR always reduces α (and thus SV and CO). To target QR=1 you must compensate with larger L, w, or L_contact.
- `teardrop_optimise` has no QR constraint. Options to add one: hard equality in `ceq`, tolerance band in `c`, or soft penalty in objective. QR=1 occurs around `r_eff/x ≈ 0.4–0.5`, i.e. `f = r2/x ≈ 0.4–0.5 + r1/x`.
- Not yet done: re-tuning `x`/`a` alongside `r2` to recover α/SV/CO while keeping the QR reduction.

### teardrop_theta() implementation pitfalls

**Quadratic root selection at φ=180°**: at φ=180°, R=a+x exactly, so the
correct tangent-line parameter is t=1 (pin at apex). Floating-point rounding
can push t1 to 1+ε, which must NOT fall back to t2≈−(a+x)/(a−x) (wildly
wrong). The correct selection:
```matlab
bad = t < 0 | t > 1;
d1  = max(0, t1-1) + max(0, -t1);   % distance of t1 from [0,1]
d2  = max(0, t2-1) + max(0, -t2);
t(bad & d2 < d1) = t2(bad & d2 < d1);  % use t2 only if it is closer
t   = max(min(t, 1), 0);               % clamp residual float error
```
Naively replacing t1→t2 then clamping gives t=0 (pin at tangent point,
~5° error); not replacing at all and clamping gives t=1 (correct).

**Units in the dynamics scripts**: `dth_dphi_t` from `gradient(theta, phi)` is
deg/deg = rad/rad = dimensionless. Therefore:
```matlab
dtheta_dt_rad = dth_dphi_t * omega_gb;      % rad/s  — NO pi/180 needed
dtheta_dt     = dtheta_dt_rad * (180/pi);   % °/s    — for display only
```
Applying `* pi/180` to `dth_dphi_t` before multiplying by `omega_gb` scales
all dynamic quantities (dθ/dt, Q, Tg, Tm, P_elec) down by 57× — a previously
present bug, now fixed in `paddle_teardrop_plot.m`.

## Files
| File | Description |
|------|-------------|
| `teardrop_plot.m` | Full teardrop dynamics at a single param set: kinematics, flow, torque, power (3 figures). Has the r1(pin)/r2(wall) split. |
| `teardrop_sweep.m` | r2 sweep: kinematics, flow rate + slot wall geometry across r2 values. Has the r1(pin)/r2(wall) split. |
| `teardrop_optimise.m` | Optimises x,a,r2,L,w,L_contact for CO/power targets. Has the r1(pin)/r2(wall) split. |
| `teardrop_viz.m` | Interactive animation with sliders + live power overlay. Has the r1(pin)/r2(wall) split, incl. drawing the actual wall (not the pin-centre path) with fillets at both ends. |
| `teardrop_double_plot.m` | Full dynamics for double-teardrop slot variant. Pre-split model. |
| `teardrop_double_sweep.m` | r1 sweep for double-teardrop variant. Pre-split model. |
| `teardrop_double_optimise.m` | Optimiser for double-teardrop variant. Pre-split model. |
| `teardrop_double_viz.m` | Interactive animation for double-teardrop variant. Pre-split model. |
| `slot_plot.m` | Full dynamics, straight slot |
| `slot_optimise.m` | Optimiser for straight slot |
| `slot_viz.m` | Interactive animation, straight slot |

### Parameters common to all teardrop_ files
| Parameter | What it does |
|-----------|-------------|
| `p_LV_mmHg` | LV peak bag pressure (mmHg); default 120 |
| `p_RV_mmHg` | RV peak bag pressure (mmHg); default 25 |
| `lv_fast` | `true` = LV on fast stroke (phi~0); `false` = LV on slow stroke |
| `b` | Geometric bag overlap [0, 0.5] |

`r1`/teardrop-radius meaning differs by file (see the split note above):
| Parameter | What it does | Where |
|-----------|-------------|-------|
| `r1` | Crank pin radius (mm), fixed; `r2` cannot be smaller than this | `teardrop_plot.m`, `teardrop_optimise.m`, `teardrop_viz.m`, `teardrop_sweep.m` |
| `r2` | Teardrop (wall) radius (mm), the design/slider/sweep variable; `r_eff = r2−r1` drives kinematics | same four files |
| `r1` | Teardrop arc radius (mm) fed directly to `teardrop_theta`; 0 = straight slot, must be < x | `teardrop_double_*.m` (pre-split model) |
