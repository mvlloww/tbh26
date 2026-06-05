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

## `b` parameter footgun
`b` is a **geometric** bag overlap fraction, not a time fraction of the cycle. It was previously misimplemented as `b*T` — don't repeat that mistake.

## Files
| File | Description |
|------|-------------|
| `paddle_angle_plot.m` | Main kinematic + flow rate plots (2 figures) |
| `tbh26_mechanism_v1.m` | Mechanism design analysis |
| `appendix_d_ejection_calc.m` | Ejection calculations (Appendix D) |
| `tbh27_mechanism_archived.m` | Previous iteration, archived |
