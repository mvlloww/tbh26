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

## Archived files (`old/`)

| File | Notes |
|------|-------|
| `tbh26_mechanism_v1.m` | Early mechanism design analysis |
| `tbh27_mechanism_archived.m` | Previous iteration (tbh27), archived |
| `appendix_d_ejection_calc.m` | Ejection calculations from Appendix D |
