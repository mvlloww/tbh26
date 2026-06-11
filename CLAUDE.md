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
`(a + x) / (a - x) = 31/11 ≈ 2.82` — LV stroke is 2.82× faster than RV.
Peak angular velocity at neutral: LV ≈ 1271°/s, RV ≈ 451°/s.

## Flow rate contact zone
`L_contact` is measured **from the tip** of the paddle inward, not from the pivot.
Contact zone: radius `(L - L_contact)` to `L`. This matters because tip contact produces more flow per mm than pivot-end contact (larger moment arm). `K_geom = w * (L² − (L − L_contact)²) / 2`.

## Torque model
`T_p = F_total × r_moment` where `F_total = p_bag × A_contact + F_e` (constant during ejection).
`r_moment = L − L_contact/2` (midpoint of contact zone from pivot, in metres).
`A_contact = w × L_contact` in m². Overlap b shifts torque onset via the existing ejection masks — no extra logic needed.

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

## Files
| File | Description |
|------|-------------|
| `paddle_angle_plot.m` | Main kinematic + flow rate plots (2 figures) |
| `paddle_optimise.m` | Optimises x,a,L,w,L_contact for CO/power targets |
| `tbh26_mechanism_v1.m` | Mechanism design analysis |
| `appendix_d_ejection_calc.m` | Ejection calculations (Appendix D) |
| `tbh27_mechanism_archived.m` | Previous iteration, archived |
