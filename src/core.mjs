// hingekey core — pure ES module. No three, no DOM, no I/O, no env.
// Hinge-joint keyframe decimation with an exact continuous-time error guarantee.
//
// Model: a joint track is a list of keys (t_i, angle_i) with linear interpolation
// between keys (exactly what a hinge rotation about a fixed axis does: slerp of
// q(theta) about one axis is linear in theta). A decimated track keeps a subset
// of the ORIGINAL keys and interpolates linearly between kept keys.
// Since both the original and the decimated angle are piecewise linear on the
// original knot set, |original(t) - decimated(t)| is piecewise linear too, so its
// supremum over the whole interval is attained at an original knot. Hence the
// error check is exact — no sampling, no Lipschitz fudge.

export const EPS = 1e-12;

/** Max |angle_k - chord(i,j)(t_k)| over interior knots of [i,j]. */
export function segmentMaxError(times, angles, i, j) {
  const t0 = times[i], t1 = times[j];
  const a0 = angles[i], a1 = angles[j];
  const dt = t1 - t0;
  if (!(dt > 0)) return { maxError: Infinity, atKnot: -1 };
  const inv = 1 / dt;
  let m = 0, arg = -1;
  for (let k = i + 1; k < j; k++) {
    const s = (times[k] - t0) * inv;
    const e = Math.abs(angles[k] - (a0 + s * (a1 - a0)));
    if (e > m) { m = e; arg = k; }
  }
  return { maxError: m, atKnot: arg };
}

/** Exact sup of the reduction error over the entire time interval (at knots). */
export function exactMaxError(times, angles, keep) {
  let worst = 0, atKnot = -1, seg = -1;
  for (let s = 0; s + 1 < keep.length; s++) {
    const i = keep[s], j = keep[s + 1];
    const r = segmentMaxError(times, angles, i, j);
    if (r.maxError > worst) { worst = r.maxError; atKnot = r.atKnot; seg = s; }
  }
  return { maxError: worst, atKnot, segment: seg };
}

/** Feasibility matrix: F[i*n+j] === 1 iff chord i->j stays within tol at all knots. */
function feasibleMatrix(times, angles, n, tol) {
  const F = new Uint8Array(n * n);
  for (let i = 0; i < n - 1; i++) {
    F[i * n + i + 1] = 1;
    for (let j = i + 2; j < n; j++) {
      if (segmentMaxError(times, angles, i, j).maxError <= tol) F[i * n + j] = 1;
    }
  }
  return F;
}

/**
 * Provably minimum-cardinality subset of original keys (endpoints included)
 * such that every chord stays within tol. Exact interval DP on the DAG of
 * feasible chords: best[j] = 1 + min best[i] over feasible i->j.
 */
export function optimalSubset(times, angles, tol) {
  const n = times.length;
  if (n <= 2) return { keep: n === 2 ? [0, 1] : [0], optimal: true, segments: Math.max(0, n - 1) };
  const F = feasibleMatrix(times, angles, n, tol);
  const best = new Int32Array(n).fill(0x7fffffff);
  const prev = new Int32Array(n).fill(-1);
  best[0] = 1; prev[0] = -1;
  for (let j = 1; j < n; j++) {
    for (let i = j - 1; i >= 0; i--) {
      if (F[i * n + j] && best[i] + 1 < best[j]) { best[j] = best[i] + 1; prev[j] = i; }
    }
  }
  if (prev[n - 1] < 0) return { keep: [0, n - 1], optimal: false, segments: 1 };
  const keep = [];
  for (let j = n - 1; j >= 0; j = prev[j]) { keep.push(j); if (j === 0) break; }
  keep.reverse();
  return { keep, optimal: best[n - 1] < 0x7fffffff, segments: best[n - 1] - 1, dpKeys: best[n - 1] };
}

/** Greedy Ramer–Douglas–Peucker style split (heuristic baseline, not minimal). */
export function greedySubset(times, angles, tol) {
  const n = times.length;
  if (n <= 2) return { keep: n === 2 ? [0, 1] : [0] };
  const keep = new Uint8Array(n);
  keep[0] = 1; keep[n - 1] = 1;
  const stack = [[0, n - 1]];
  while (stack.length) {
    const [i, j] = stack.pop();
    const r = segmentMaxError(times, angles, i, j);
    if (r.maxError > tol && r.atKnot > i && r.atKnot < j) {
      keep[r.atKnot] = 1;
      stack.push([i, r.atKnot], [r.atKnot, j]);
    }
  }
  const out = [];
  for (let k = 0; k < n; k++) if (keep[k]) out.push(k);
  return { keep: out };
}

/** Uniform stride baseline: smallest stride whose certified error is within tol. */
export function uniformSubset(times, angles, tol) {
  const n = times.length;
  if (n <= 2) return { keep: n === 2 ? [0, 1] : [0], stride: 1 };
  for (let s = 2; s < n; s++) {
    const keep = [];
    for (let k = 0; k < n; k += s) keep.push(k);
    if (keep[keep.length - 1] !== n - 1) keep.push(n - 1);
    if (exactMaxError(times, angles, keep).maxError <= tol) return { keep, stride: s };
  }
  return { keep: [...Array(n).keys()], stride: 1 };
}

/** Evaluate a (possibly decimated) track at time t, clamping outside the range. */
export function evalTrack(times, angles, keep, t) {
  const m = keep.length;
  if (t <= times[keep[0]]) return angles[keep[0]];
  if (t >= times[keep[m - 1]]) return angles[keep[m - 1]];
  let lo = 0, hi = m - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (times[keep[mid]] <= t) lo = mid; else hi = mid;
  }
  const i = keep[lo], j = keep[hi];
  const s = (t - times[i]) / (times[j] - times[i]);
  return angles[i] + s * (angles[j] - angles[i]);
}

/** Dense sampled max error — independent of the knot argument, used by verify as a cross-check. */
export function denseMaxError(times, angles, keep, samplesPerSegment = 32) {
  let worst = 0, at = -1;
  for (let s = 0; s + 1 < keep.length; s++) {
    const i = keep[s], j = keep[s + 1];
    for (let q = 0; q <= samplesPerSegment; q++) {
      const t = times[i] + (times[j] - times[i]) * (q / samplesPerSegment);
      const e = Math.abs(angleAt(times, angles, t) - evalTrack(times, angles, keep, t));
      if (e > worst) { worst = e; at = t; }
    }
  }
  return { maxError: worst, at };
}

/** Original track evaluated with a linear scan (deliberately simple, independent path). */
export function angleAt(times, angles, t) {
  const n = times.length;
  if (t <= times[0]) return angles[0];
  if (t >= times[n - 1]) return angles[n - 1];
  let i = 0;
  while (i + 1 < n && times[i + 1] <= t) i++;
  const s = (t - times[i]) / (times[i + 1] - times[i]);
  return angles[i] + s * (angles[i + 1] - angles[i]);
}

// ---------- rigid chain FK (hinge joints about arbitrary local axes) ----------

function rotAxis(axis, angle, out) {
  const [x, y, z] = axis;
  const c = Math.cos(angle), s = Math.sin(angle), C = 1 - c;
  out[0] = c + x * x * C; out[1] = x * y * C - z * s; out[2] = x * z * C + y * s;
  out[3] = y * x * C + z * s; out[4] = c + y * y * C; out[5] = y * z * C - x * s;
  out[6] = z * x * C - y * s; out[7] = z * y * C + x * s; out[8] = c + z * z * C;
  return out;
}

function mul3(a, b, out) {
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
    out[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
  }
  return out;
}

function apply3(m, v, out) {
  out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
  out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
  out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
  return out;
}

/**
 * Forward kinematics: chain.joints[m] = {axis:[x,y,z], offset:[x,y,z]} where
 * offset is the translation from the previous joint origin to this joint origin.
 * chain.tip = end-effector offset in the last joint's frame.
 * Returns { joints: [pos...], tip: [x,y,z] } in world space.
 */
export function fk(chain, angles) {
  let R = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const p = [0, 0, 0];
  const joints = [];
  const tmpM = new Array(9), tmpR = new Array(9), tmpV = [0, 0, 0];
  for (let m = 0; m < chain.joints.length; m++) {
    const j = chain.joints[m];
    apply3(R, j.offset, tmpV);
    p[0] += tmpV[0]; p[1] += tmpV[1]; p[2] += tmpV[2];
    joints.push([p[0], p[1], p[2]]);
    rotAxis(j.axis, angles[m], tmpM);
    mul3(R, tmpM, tmpR);
    R = tmpR.slice();
  }
  apply3(R, chain.tip, tmpV);
  return { joints, tip: [p[0] + tmpV[0], p[1] + tmpV[1], p[2] + tmpV[2]], rotation: R };
}

/** Certified downstream reach R_j: |offset_{j+1}|+...+|tip| (triangle inequality). */
export function reachWeights(chain) {
  const jn = chain.joints.length;
  const out = new Array(jn).fill(0);
  let reach = Math.hypot(...chain.tip);
  for (let m = jn - 1; m >= 0; m--) {
    out[m] = reach;
    reach += m > 0 ? Math.hypot(...chain.joints[m].offset) : 0;
  }
  return out;
}

/**
 * Certified end-effector position bound: sum_j |dTheta_j| * R_j.
 * Valid for ALL times because |R(dTheta)p - p| <= |dTheta| |p| and the chain
 * telescopes. Absolute (world) units.
 */
export function certifiedTipBound(chain, angleErrors) {
  const w = reachWeights(chain);
  let b = 0;
  for (let m = 0; m < w.length; m++) b += Math.abs(angleErrors[m]) * w[m];
  return b;
}

/** Dense FK sample of tip error between two angle sets for the same chain. */
export function sampledTipError(chain, angleA, angleB) {
  const a = fk(chain, angleA).tip, b = fk(chain, angleB).tip;
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/** Reduce every joint of a rig to tol radians; returns per-joint results + certified tip bound. */
export function reduceRig(rig, tol, exactTipSamples = []) {
  const perJoint = [];
  const errs = [];
  for (let m = 0; m < rig.joints.length; m++) {
    const tr = rig.joints[m];
    const r = optimalSubset(tr.times, tr.angles, tol);
    const e = exactMaxError(tr.times, tr.angles, r.keep);
    perJoint.push({ name: tr.name, keep: r.keep, keyCount: r.keep.length, maxError: e.maxError, atKnot: e.atKnot });
    errs.push(e.maxError);
  }
  const bound = certifiedTipBound(rig.chain, errs);
  let sampled = 0;
  for (const t of exactTipSamples) {
    const a = rig.joints.map((tr) => angleAt(tr.times, tr.angles, t));
    const b = rig.joints.map((tr, m) => evalTrack(tr.times, tr.angles, perJoint[m].keep, t));
    const d = sampledTipError(rig.chain, a, b);
    if (d > sampled) sampled = d;
  }
  return { perJoint, totalKeys: perJoint.reduce((s, p) => s + p.keyCount, 0), tipBound: bound, sampledTipError: sampled, tol };
}

/** Stable fingerprint of a reduction result (order-sensitive FNV-1a over 32-bit ints). */
export function fingerprint(obj) {
  const s = JSON.stringify(obj);
  let h = 0x811c9dc5;
  for (let i = 0; i < s.length; i++) { h ^= s.charCodeAt(i); h = Math.imul(h, 0x01000193) >>> 0; }
  return h.toString(16).padStart(8, '0');
}
// ---------- rig-level tip-space budget allocation ----------
// The per-channel problem (smallest subset per joint at a fixed angle tolerance)
// is only the inner loop. The rig-level question is different: given ONE budget
// epsilon on the end-effector position, distribute angle budgets across joints
// so that TOTAL keys are minimal. Joint m costs |dTheta_m| * R_m of tip-space
// budget (R_m = downstream reach), and its key count is a step function of
// dTheta_m obtained from the exact DP above.

/** Log-spaced candidate angle tolerances (radians), ascending. */
export function deltaGrid(maxDelta, levels = 12, minDelta = 1e-5) {
  const out = [];
  for (let i = 0; i < levels; i++) {
    const s = levels === 1 ? 0 : i / (levels - 1);
    out.push(minDelta * Math.pow(maxDelta / minDelta, s));
  }
  return out;
}

/** For each candidate tolerance: exact minimum key count and exact attained error. */
export function budgetCurve(times, angles, deltas) {
  return deltas.map((d) => {
    const r = optimalSubset(times, angles, d);
    const e = exactMaxError(times, angles, r.keep);
    return { delta: d, keep: r.keep, keys: r.keep.length, error: e.maxError };
  });
}

/**
 * Minimum total key count subject to sum_m error_m * R_m <= epsTip.
 * Budget is quantized to epsTip/quantum (ceil per joint => always feasible);
 * optimality is claimed only over the disclosed grid + quantization.
 */
export function allocateBudgets(rig, epsTip, opts = {}) {
  const levels = opts.levels ?? 12;
  const Q = opts.quantum ?? 400;
  const minDelta = opts.minDelta ?? 1e-6;
  const maxDelta = opts.maxDelta ?? (8 * Math.PI) / 180;
  const grid = opts.grid ?? deltaGrid(maxDelta, levels, minDelta);
  const R = reachWeights(rig.chain);
  const curves = rig.joints.map((tr) => budgetCurve(tr.times, tr.angles, grid));
  const unit = epsTip / Q;
  const INF = 0x3fffffff;
  const J = rig.joints.length;
  const dp = Array.from({ length: J + 1 }, () => new Int32Array(Q + 1).fill(INF));
  const choice = Array.from({ length: J + 1 }, () => new Int16Array(Q + 1).fill(-1));
  dp[0][0] = 0;
  for (let m = 0; m < J; m++) {
    for (let q = 0; q <= Q; q++) {
      if (dp[m][q] >= INF) continue;
      for (let l = 0; l < grid.length; l++) {
        const c = curves[m][l];
        const spend = Math.max(0, Math.ceil((c.error * R[m]) / unit - 1e-12));
        if (spend > Q - q) continue;
        const nq = q + spend;
        const nk = dp[m][q] + c.keys;
        if (nk < dp[m + 1][nq]) { dp[m + 1][nq] = nk; choice[m + 1][nq] = l; }
      }
    }
  }
  let bestQ = -1, bestKeys = INF;
  for (let q = 0; q <= Q; q++) if (dp[J][q] < bestKeys) { bestKeys = dp[J][q]; bestQ = q; }
  if (bestQ < 0) throw new Error('no feasible allocation');
  const picks = new Array(J);
  let q = bestQ;
  for (let m = J - 1; m >= 0; m--) {
    const l = choice[m + 1][q];
    if (l < 0) throw new Error(`allocation reconstruct failed at joint ${m}`);
    picks[m] = l;
    const c = curves[m][l];
    q -= Math.max(0, Math.ceil((c.error * R[m]) / unit - 1e-12));
  }
  const perJoint = picks.map((l, m) => ({
    name: rig.joints[m].name,
    deltaRad: grid[l],
    deltaDeg: grid[l] / (Math.PI / 180),
    keys: curves[m][l].keys,
    errorRad: curves[m][l].error,
    spend: curves[m][l].error * R[m],
    reach: R[m],
    keep: curves[m][l].keep,
  }));
  const certifiedBound = perJoint.reduce((s, p) => s + p.spend, 0);
  return { epsTip, grid, quantum: Q, unit, totalKeys: bestKeys, perJoint, certifiedBound, reach: R, usedUnits: bestQ, curves };
}

/** Apply an allocation (or any per-joint deltas) and measure the sampled tip error. */
export function reduceWithBudgets(rig, perJoint, sampleTimes) {
  const keeps = perJoint.map((p) => p.keep);
  const errs = perJoint.map((p) => p.errorRad);
  let sampled = 0;
  for (const t of sampleTimes) {
    const a = rig.joints.map((tr) => angleAt(tr.times, tr.angles, t));
    const b = rig.joints.map((tr, m) => evalTrack(tr.times, tr.angles, keeps[m], t));
    const d = sampledTipError(rig.chain, a, b);
    if (d > sampled) sampled = d;
  }
  return { perJointKeys: keeps.map((k) => k.length), totalKeys: keeps.reduce((s, k) => s + k.length, 0), certifiedBound: certifiedTipBound(rig.chain, errs), sampledTipError: sampled };
}

/** Equal-budget baseline: one angle tolerance for every joint, largest that fits eps. */
export function uniformBudgetBaseline(rig, epsTip) {
  const R = reachWeights(rig.chain);
  const sumR = R.reduce((a, b) => a + b, 0);
  const delta = epsTip / sumR;
  let totalKeys = 0;
  const perJoint = [];
  for (let m = 0; m < rig.joints.length; m++) {
    const tr = rig.joints[m];
    const r = optimalSubset(tr.times, tr.angles, delta);
    const e = exactMaxError(tr.times, tr.angles, r.keep);
    totalKeys += r.keep.length;
    perJoint.push({ name: tr.name, delta, keys: r.keep.length, errorRad: e.maxError, keep: r.keep });
  }
  return { epsTip, delta, totalKeys, perJoint, certifiedBound: certifiedTipBound(rig.chain, perJoint.map((p) => p.errorRad)) };
}
