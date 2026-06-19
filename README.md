# tbh26 — LVAD Biventricular Drive Unit

MATLAB analysis and optimisation of the crank-and-slotted-arm mechanism for a biventricular LVAD. The crank rotates at `rpm_max` and drives a paddle arm via a slot, alternately squeezing LV and RV blood bags.

---

## Slot geometry variants

Three slot shapes are implemented, each building on the last. Files are grouped by prefix.

### `slot_*` — Straight slot (baseline)

The radial slot is a plain straight channel. The crank pin travels in a straight line in the arm's body frame. This gives the maximum quick-return ratio `QR = (a+x)/(a−x)`, meaning the LV stroke is significantly faster than the RV stroke.

| File | Purpose |
|------|---------|
| `slot_plot.m` | Kinematic and full dynamics plots (paddle angle, flow rate, torque, power) over two cycles |
| `slot_optimise.m` | Optimises `x`, `a`, `L`, `w`, `L_contact` to minimise peak electrical power subject to CO target |
| `slot_viz.m` | Interactive animation with sliders for all mechanism parameters |

---

### `teardrop_*` — Single-radius teardrop slot

The near-pivot end of the straight slot is replaced by a circular arc of radius `r1`. This rounds off the sharp point of the slot and reduces the quick-return ratio (shorter, slower LV stroke; longer, more even RV stroke). `r1 → 0` recovers the straight slot exactly.

The arc introduces a C1 kink in `dθ/dφ` at the arc/wall junction, causing a brief spike in flow rate and power at that transition.

| File | Purpose |
|------|---------|
| `teardrop_plot.m` | Full dynamics plots for a single set of parameters |
| `teardrop_sweep.m` | Sweeps `r1` across four fractions of `x` to show the QR ratio and stroke-volume trade-off |
| `teardrop_optimise.m` | Optimises `x`, `a`, `r1`, `L`, `w`, `L_contact` to minimise mechanism size (`a+L`) subject to CO target, power budget, and slot clearance |
| `teardrop_viz.m` | Interactive animation with sliders including `r1` and paddle thickness `t` |

**Key parameters:** `r1` (teardrop arc radius, mm). Valid range: `0 < r1 < x`. Typical: `r1 = 0.2x`.

---

### `teardrop_double_*` — Double-radius slot (teardrop + curved side walls)

Extends the teardrop slot by replacing the straight side walls with a second circular arc of radius `r2`. The `r2` arc is externally tangent to the `r1` arc and passes through the slot apex. This eliminates the C1 kink at the arc/wall junction, smoothing `dθ/dφ` and reducing the power spike.

`r2 = 0` falls back to the single teardrop. `r2` must exceed a minimum `r2* = 2x(x−r1)/r1` for the slot outline to be geometrically valid (below this the arc crosses the centreline).

| File | Purpose |
|------|---------|
| `teardrop_double_plot.m` | Full dynamics plots for a single set of parameters |
| `teardrop_double_sweep.m` | Sweeps `r2` at fixed `r1` to show the smoothing effect on `dθ/dφ` and power |
| `teardrop_double_optimise.m` | Optimises `x`, `a`, `r1`, `r2`, `L`, `w`, `L_contact` to minimise mechanism size (`a+L`) + smoothness penalty, subject to CO target and power budget |
| `teardrop_double_viz.m` | Interactive animation with sliders for all parameters including `r1`, `r2`, and paddle thickness `t` |

**Key parameters:** `r1` (teardrop arc radius), `r2` (side-wall arc radius). `r2 > r2*` required for valid geometry.

---

## Shared parameters

| Parameter | Symbol | Typical value | Description |
|-----------|--------|---------------|-------------|
| Crank arm length | `x` | 9.9 mm | Eccentricity of crank pin from motor axis |
| Crank–pivot distance | `a` | 28 mm | Distance from motor centre O2 to arm pivot O4 |
| Paddle length | `L` | 40–57 mm | Radial extent of paddle from pivot |
| Paddle width | `w` | 75–100 mm | Out-of-plane dimension (contacts blood bag) |
| Paddle thickness | `t` | 4 mm | In-plane dimension perpendicular to arm (structural; must fit slot) |
| Contact length | `Lc` | 35–50 mm | Length of paddle face in contact with bag, measured from tip |
| Bag overlap | `b` | 0.5 | Geometric pre-fill fraction [0, 0.5]; `b=0.5` means bags are 50% full at neutral |
| Max crank speed | `rpm_max` | 145 rpm | Crank shaft speed |
| Bag pressure | `p_bag` | 16 kPa | Blood bag working pressure |

---

## Quick-return ratio

`QR = (a+x)/(a−x)` for the straight slot. The teardrop reduces this; the double-radius slot reduces it further while eliminating the associated power spike. `QR < 1` means the RV stroke becomes faster than the LV stroke (inversion occurs around `r1 ≈ 0.5x`).

Current params (`x=9.9 mm`, `a=28 mm`): QR ≈ 2.09 (straight), ≈ 1.5 (`r1=0.2x` teardrop).

---

## Optimisers

All three optimisers use MATLAB's `fmincon` (SQP algorithm) with a 3–4 point multi-start to reduce sensitivity to the initial guess. Each runs silently across all seeds, picks the best feasible result, then re-runs that seed with iteration display on so you can watch convergence.

---

### How they differ

| File | Objective | Design variables | What it finds |
|------|-----------|-----------------|---------------|
| `slot_optimise.m` | Minimise `P_elec_peak` | `x, a, L, w, Lc` | Lowest power for the straight slot |
| `teardrop_optimise.m` | Minimise `size_weight·(a+L) + λ·kink` | `x, a, f=r1/x, L, w, Lc` | Smallest mechanism that fits in power budget |
| `teardrop_double_optimise.m` | Minimise `size_weight·(a+L) + λ·kink` | `x, a, f=r1/x, g=r2/x, L, w, Lc` | Smallest mechanism with smoothest power profile |

The teardrop and double-radius optimisers treat **power as a hard constraint** (`P_elec_peak ≤ P_max`) and use the power headroom to shrink the mechanism. The straight-slot optimiser simply minimises peak power directly.

---

### Hard constraints (always enforced)

All three optimisers share these:

- **CO = CO_target** — cardiac output per ventricle must exactly hit the target (equality constraint)
- **alpha ≥ alpha_min** — paddle swing angle must stay above a minimum to ensure adequate stroke volume
- **x < a** — crank arm must be shorter than crank-to-pivot distance (mechanism validity)
- **L_contact ≤ L − L_paddle_pin_radius** — contact zone inner edge must clear the pivot pin

The teardrop and double optimisers also enforce:
- **P_elec_peak ≤ P_max** — peak electrical power must not exceed the budget

---

### Weights and targets — what to change and where

All the numbers you'd want to adjust are declared at the top of each file, before the bounds section:

```matlab
%% Fixed parameters
rpm_max = 145;        % crank speed — change if motor speed changes
CO_target = 5;        % L/min per ventricle — change for different cardiac output target
P_max = 15.6;         % W — change to reflect revised power budget

alpha_min = 8;        % deg — raise to force larger paddle swing (safer stroke volume margin)
L_paddle_pin_radius = 3;  % mm — raise if pivot pin is larger

% teardrop / double only:
lambda = 100;         % smoothness weight: raise to prioritise flatter dθ/dφ over size
size_weight = 0.2;    % W/mm: raise to push harder for smaller a+L
```

**`lambda` vs `size_weight` trade-off:**  raising `lambda` makes the optimiser prefer smoother `dθ/dφ` (less power spike) at the cost of larger mechanism dimensions; raising `size_weight` shrinks the mechanism but tolerates more kink.

---

### Bounds — what to change and where

Immediately after the parameters, each file has `lb` and `ub` vectors that set the search range for each design variable:

```matlab
% teardrop_double_optimise.m example:
%  v = [x,   a,    f,    g,    L,   w,   Lc  ]
lb = [ 5,   12,  0.05, 0.00,  15,  33,   5  ];
ub = [20,   25,  0.90, 30.0,  60,  75,  50  ];
```

Common reasons to change bounds:

| Bound | What changes | Why |
|-------|-------------|-----|
| `ub(2)` (`a_max`) | Maximum crank-pivot distance | Physical envelope of the device |
| `ub(1)` (`x_max`) | Maximum crank arm | Motor shaft eccentricity limit |
| `lb(5)` / `ub(5)` (`L` range) | Paddle length | Device footprint constraints |
| `ub(6)` (`w_max`) | Paddle width | Blood bag width limit |
| `lb(3)` (`f_min`) | Minimum `r1` fraction | Keep `f ≥ 0.05` so the teardrop always exists |
| `ub(4)` (`g_max`) | Maximum `r2` fraction | Raise if optimiser hits the upper wall |

---

### Seeds — what to change and where

Below the bounds, a `v0_list` cell array provides starting points for the multi-start. If the optimiser converges to an infeasible or poor solution, add a seed closer to the expected answer:

```matlab
v0_list = {
    [9.9, 28, 0.20,  0.0, 33, 75, 33],   % baseline
    [9.9, 28, 0.20, 10.0, 33, 75, 33],   % with r2
    ...
};
```

Each row is a `[x, a, f, g, L, w, Lc]` vector (or `[x, a, f, L, w, Lc]` for the single teardrop). Seeds that violate linear constraints are automatically rejected by `fmincon`.

---

### Changing the objective

The objective function is a one-liner anonymous function:

```matlab
% Current (teardrop / double):
objective = @(v) size_weight * (v(2) + v(4)) + lambda * kink_penalty(v, phi_deg);

% To minimise peak power instead (like slot_optimise):
objective = @(v) p_elec_peak(v, omega_gb, e_gb, e_mech, e_motor, p_bag, F_e, phi_deg);

% To minimise only mechanism size (ignore smoothness):
objective = @(v) v(2) + v(4);   % a + L
```

If you switch to minimising peak power, remove the `P - P_max` line from `nlcon` (otherwise the power constraint fights the objective).

---

## Archived files (`old/`)

| File | Notes |
|------|-------|
| `tbh26_mechanism_v1.m` | Early mechanism design analysis |
| `tbh27_mechanism_archived.m` | Previous iteration (tbh27), archived |
| `appendix_d_ejection_calc.m` | Ejection calculations from Appendix D |
