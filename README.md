# hingekey-cert-lab

**One end-effector budget, minimum total keyframes.** On a 6-joint hinge rig (241 keys/joint, 1,446 total, 2.59 m reach): an end-effector budget of **ε = 0.167 world units** keeps **296 keys instead of 370** for the equal-Δθ baseline (**−20.0%**) and **296 instead of 387** for a 1°/joint plan (−23.5%), with every joint's continuous-time angle error **exactly** bounded (no sampling) and the tip bound **certified** at 0.1622 ≤ ε.

Upstream: none

## What it does

Two different problems live here, and only the second one is the product:

1. **Inner loop (exact, per joint).** Given one angle tolerance, find the *provably minimum-cardinality* subset of original keyframes such that linear interpolation never exceeds the tolerance. Both the original and the reduced angle are piecewise linear on the original knots, so `|θ(t) − θ̂(t)|` is piecewise linear too: its supremum is attained at an original knot. The check is exact — no sampling, no Lipschitz fudge. The subset comes from an interval DP over feasible chords, and is checked against exhaustive enumeration on small tracks.
2. **Outer loop (the new part).** A rig does not care about per-joint tolerances; it cares about the **end effector**. Joint `m` consumes `|Δθ_m| · R_m` of tip-space budget (`R_m` = downstream reach), and its key count is a step function obtained from the exact DP. `allocateBudgets()` solves the cross-joint knapsack by DP over a quantized tip budget and returns the **minimum total keys** for one ε. Equal-Δθ is the baseline it beats.

The browser plays the original and the allocated plan side by side and shows the same numbers, recomputed by the same `src/core.mjs`, plus a fingerprint match against the node-baked plan.

## Run it

```bash
node --version                      # v24.x
node tools/gen-sample.mjs           # writes data/rig.json (deterministic)
node bench/verify.mjs               # 41 assertions, exit 0
node bench/bench.mjs                # timing + bench/result.json
python3 -m http.server 4173          # open http://localhost:4173
```

No install step: no dependencies, no `node_modules`, three.js comes from a pinned CDN importmap (`three@0.169.0`).

## Evidence (real runs, `bench/result.json`)

| item | measured |
| --- | --- |
| original keys | 1,446 (6 joints × 241) |
| per-joint 1° plan (exact DP) | **387 keys (−73.24%)**, worst joint error 1.0000° |
| greedy RDP at 1° | 462 keys (**+19.4%** vs DP), feasible |
| uniform stride at 1° | 966 keys (**+149.6%** vs DP), feasible |
| allocation ε=0.167 | **296 keys**, certified tip bound **0.1622**, sampled FK error 0.0720 |
| equal-budget baseline ε=0.167 | 370 keys (Δθ = 1.087°/joint), **−20.0%** for allocation |
| allocation ε=0.05 / 0.25 | 700 / 209 keys vs 772 / 283 equal-budget (−9.3% / −26.1%) |
| per-joint DP timing | cold 42.9 ms, median **31.4 ms**, p95 35.2, relSD 5.21% (7 reps) |
| allocation timing | cold 342.5 ms, median **338.6 ms**, p95 339.0, relSD 0.22% (5 reps) |
| sampled-tip / certified bound | ≤ 1.0 at all 4 budgets; 200 random perturbations: 0 violations |
| scale invariance | angle × {1e-6…1e6}: identical keep sets; tip bound scales exactly |
| generator determinism | two runs, same sha256 `6cf6a08f939562a0…` |

`bench/verify.mjs` prints every asserted value (41 passed / 0 failed), including DP-vs-exhaustive (512 subsets, n=11) and allocation-DP-vs-exhaustive over 7⁶ = 117,649 grid combinations.

## 本轮创新

**B 档.** Existing keyframe reducers are per-channel: glTF-Transform `resample()` prunes greedily and can exceed tolerance; Blender's *Decimate (Allowed Change)* is a per-F-curve heuristic; even our own earlier `motion-prune-lab` minimizes **each track separately** against its own tolerance. This repo keeps the per-track exactness but changes the optimization: **one tip-space budget is distributed across joints by a DP that trades each joint's exact key-count step function against its reach**, which is why it beats the equal-Δθ plan by 20% at the same ε. Nobody in the ledger (or in the tools above) produces a cross-joint allocation with a certified end-effector bound. The DP itself is classical (Imai–Iri minimum-link approximation); the new object is the rig-level allocation + certified FK propagation.

## Known limitations (read before trusting a number)

- **Hinge joints only.** The linear-in-angle slerp identity holds for a rotation about one fixed local axis. Ball joints / multi-axis quaternion tracks are rejected, not approximated.
- **Keys are a subset of the originals.** No new key times are synthesized; that keeps the guarantee exact and the format glTF-friendly, but it can leave keys on the table versus free-time fitting.
- **Optimality is over a disclosed discretization.** 12 log-spaced tolerances (1e-6…8°) and a tip budget quantized to ε/400. Finer grids give fewer keys (e.g. the 7-level check finds 328, the 12-level plan 296); the claim is minimum *over the disclosed grid*, verified against exhaustive search on the same grid.
- **Certified tip bound is an upper bound**, `Σ|Δθ_m|·R_m` (triangle inequality + `|R(Δθ)p−p| ≤ |Δθ||p|`). The sampled FK error is below it (39.7–44.4% of bound on this rig), so the bound is conservative, not tight.
- **Procedural rig, not a shipped glTF asset.** No real URDF/glTF file was tested; export-format edge cases (cubic spline samplers, negative times) are out of scope.
- **Browser layer is verified only statically** (HTTP 200, importmap, `node --check`, shared core). No browser automation is used, per project rules; the HUD fingerprint is displayed for a human to compare.

## Layout

```
index.html main.js          # render layer, pinned three@0.169.0 importmap
src/core.mjs                # pure math: DP, exact error, FK, allocation (no three/DOM)
tools/gen-sample.mjs        # deterministic rig generator
baseline/naive.mjs          # greedy RDP + uniform stride comparators
bench/verify.mjs            # 41 assertions
bench/bench.mjs bench/result.json
```

MIT licensed. No upstream code copied.