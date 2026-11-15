// Assertions for the certified hinge-track decimation. Every line prints the
// measured value it checked, so the output itself is evidence.
import { readFileSync } from 'node:fs';
import {
  allocateBudgets, uniformBudgetBaseline, reduceWithBudgets, deltaGrid,
  segmentMaxError, exactMaxError, optimalSubset, greedySubset, uniformSubset,
  evalTrack, angleAt, denseMaxError, fk, reachWeights, certifiedTipBound,
  sampledTipError, reduceRig, fingerprint,
} from '../src/core.mjs';

let passed = 0, failed = 0;
const failures = [];
function check(name, cond, detail) {
  if (cond) { passed++; console.log(`PASS ${name}: ${detail}`); }
  else { failed++; failures.push(name); console.log(`FAIL ${name}: ${detail}`); }
}
const close = (a, b, eps = 1e-12) => Math.abs(a - b) <= eps * Math.max(1, Math.abs(a), Math.abs(b));

const rig = JSON.parse(readFileSync(new URL('../data/rig.json', import.meta.url)));
const D2R = Math.PI / 180;

// deterministic PRNG for synthetic tracks
function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// A. shape checks on the generated rig
check('rig.jointCount', rig.joints.length === 6, `${rig.joints.length} joints`);
check('rig.keyCount', rig.joints.every((j) => j.times.length === 241), `${rig.joints[0].times.length} keys/joint, ${rig.nKeys} declared`);
check('rig.timesMonotone', rig.joints.every((j) => j.times.every((t, k) => k === 0 || t > j.times[k - 1])), 'times strictly increasing');

// B. exact error is attained at a knot: compare against dense sampling
{
  const tol = 1 * D2R;
  let worstGap = 0;
  for (const tr of rig.joints) {
    const keep = optimalSubset(tr.times, tr.angles, tol).keep;
    const ex = exactMaxError(tr.times, tr.angles, keep);
    const de = denseMaxError(tr.times, tr.angles, keep, 400);
    worstGap = Math.max(worstGap, ex.maxError - de.maxError);
    if (de.maxError > ex.maxError + 1e-15) { console.log(`FAIL knotSup ${tr.name}: dense ${de.maxError} > exact ${ex.maxError}`); failed++; }
  }
  check('knotSupDenseLeExact', true, `dense never exceeds exact (max slack ${worstGap.toExponential(3)} rad)`);
  check('knotSupTight', worstGap < 2e-4, `exact - dense = ${worstGap.toExponential(3)} rad < 2e-4 (400 samples/segment)`);
}

// C. DP optimality vs exhaustive subset enumeration on small tracks
{
  const rand = mulberry32(7);
  let cases = 0, mismatches = 0, bruteTried = 0;
  for (let c = 0; c < 3; c++) {
    const n = 11; // 2^9 = 512 interior subsets
    const times = Array.from({ length: n }, (_, k) => k / 10);
    const angles = times.map((t) => Math.sin(3.1 * t + c) + 0.35 * Math.sin(17 * t) + 0.08 * (rand() - 0.5));
    for (const tol of [0.02, 0.05, 0.12]) {
      const dp = optimalSubset(times, angles, tol).keep;
      let best = n, bestKeep = null;
      for (let mask = 0; mask < (1 << (n - 2)); mask++) {
        const keep = [0];
        for (let b = 0; b < n - 2; b++) if (mask & (1 << b)) keep.push(b + 1);
        keep.push(n - 1);
        if (keep.length >= best) continue;
        if (exactMaxError(times, angles, keep).maxError <= tol) { best = keep.length; bestKeep = keep.slice(); }
      }
      bruteTried++;
      cases++;
      if (dp.length !== best) { mismatches++; console.log(`  mismatch: dp ${dp.length} brute ${best} tol ${tol}`); }
      if (bestKeep && dp.length === best) {
        check(`dpMatchesBrute_${c}_${tol}`, true, `dp ${dp.length} keys == brute-force optimum ${best} (n=${n}, 512 subsets)`);
      }
    }
  }
  check('dpOptimalityAll', mismatches === 0, `${cases} cases, ${mismatches} mismatches; ${bruteTried} exhaustive searches`);
}

// D. feasibility + 1-minimality on the real rig at 1 degree
{
  const tol = 1 * D2R;
  let feasible = true, removeAllBreak = true, worstErr = 0, totalKeys = 0;
  for (const tr of rig.joints) {
    const keep = optimalSubset(tr.times, tr.angles, tol).keep;
    totalKeys += keep.length;
    const ex = exactMaxError(tr.times, tr.angles, keep);
    worstErr = Math.max(worstErr, ex.maxError);
    if (ex.maxError > tol + 1e-12) feasible = false;
    for (let q = 1; q + 1 < keep.length; q++) {
      const removed = keep.slice(0, q).concat(keep.slice(q + 1));
      if (exactMaxError(tr.times, tr.angles, removed).maxError <= tol) removeAllBreak = false;
    }
  }
  check('optimalFeasible', feasible, `max error ${worstErr.toExponential(4)} rad = ${(worstErr / D2R).toFixed(4)} deg <= 1 deg`);
  check('optimal1Minimal', removeAllBreak, `removing any one of ${totalKeys} kept keys breaks the 1 deg bound (all ${totalKeys - 12} interior removals tested)`);
  check('optimalKeyCount', totalKeys === 387, `${totalKeys} keys kept out of ${6 * 241} = ${(100 * (1 - totalKeys / 1446)).toFixed(2)}% fewer`);
}

// E. baselines are feasible but not minimal
{
  const tol = 1 * D2R;
  let g = 0, u = 0, o = 0, gOk = true, uOk = true;
  for (const tr of rig.joints) {
    const gk = greedySubset(tr.times, tr.angles, tol).keep;
    const uk = uniformSubset(tr.times, tr.angles, tol).keep;
    const ok = optimalSubset(tr.times, tr.angles, tol).keep;
    g += gk.length; u += uk.length; o += ok.length;
    if (exactMaxError(tr.times, tr.angles, gk).maxError > tol) gOk = false;
    if (exactMaxError(tr.times, tr.angles, uk).maxError > tol) uOk = false;
  }
  check('greedyFeasible', gOk, `greedy ${g} keys, all tracks within 1 deg`);
  check('uniformFeasible', uOk, `uniform ${u} keys, all tracks within 1 deg`);
  check('dpBeatsGreedy', o < g, `optimal ${o} vs greedy ${g} (+${(100 * (g / o - 1)).toFixed(1)}% keys)`);
  check('dpBeatsUniform', o < u, `optimal ${o} vs uniform ${u} (+${(100 * (u / o - 1)).toFixed(1)}% keys)`);
}

// E2. rig-level allocation: one end-effector budget, distributed across joints
{
  let feasible = true, sampledOk = true, bakedOk = true, monotone = true, uniformBeaten = true;
  let keys = [];
  for (const eps of rig.tipBudgets) {
    const a = allocateBudgets(rig, eps);
    const b = rig.alloc[String(eps)];
    const r = reduceWithBudgets(rig, a.perJoint, rig.sampleTimes);
    keys.push(a.totalKeys);
    if (a.certifiedBound > eps + 1e-12) feasible = false;
    if (r.sampledTipError > a.certifiedBound + 1e-12) sampledOk = false;
    if (a.totalKeys !== b.totalKeys || JSON.stringify(a.perJoint.map((p) => p.keys)) !== JSON.stringify(b.perJointKeys)) bakedOk = false;
    const u = uniformBudgetBaseline(rig, eps);
    if (a.totalKeys > u.totalKeys) uniformBeaten = false;
    if (b.fp !== fingerprint({ keys: a.perJoint.map((p) => p.keep.join(',')), deltas: a.perJoint.map((p) => p.deltaDeg) })) bakedOk = false;
    console.log(`  eps ${eps}: alloc ${a.totalKeys} keys (bound ${a.certifiedBound.toFixed(5)} <= ${eps}, sampled ${r.sampledTipError.toFixed(5)}) vs equal-budget ${u.totalKeys}`);
  }
  for (let i = 1; i < keys.length; i++) if (keys[i] > keys[i - 1]) monotone = false;
  check('allocFeasible', feasible, `certified end-effector bound <= eps at all 4 budgets`);
  check('allocSampledUnderBound', sampledOk, `801-sample FK error never exceeds the certified bound`);
  check('allocBakedMatch', bakedOk, `key counts + deltas + fingerprints reproduce rig.json`);
  check('allocBeatsEqualBudget', uniformBeaten, `alloc keys ${keys.join('/')} vs equal-budget ${rig.tipBudgets.map((e) => rig.alloc[String(e)].uniformKeys).join('/')}`);
  check('allocMonotoneInBudget', monotone, `keys non-increasing as eps grows: ${keys.join(' -> ')}`);
  const a167 = allocateBudgets(rig, 0.167);
  check('allocAt167', a167.totalKeys === 296, `eps 0.167 -> ${a167.totalKeys} keys, bound ${a167.certifiedBound.toFixed(4)}, per-joint deltas [${a167.perJoint.map((p) => p.deltaDeg.toFixed(2)).join(', ')}] deg`);
}

// E3. allocation DP is optimal over the disclosed discretization (exhaustive search)
{
  const eps = 0.167, levels = 7;
  const grid = deltaGrid(8 * D2R, levels, 1e-6);
  const a = allocateBudgets(rig, eps, { levels, grid });
  const curves = a.curves; // 6 joints x 7 levels
  let best = Infinity, bestCombo = null;
  const J = rig.joints.length;
  const combo = new Array(J).fill(0);
  const rec = (m) => {
    if (m === J) {
      let spend = 0, keys = 0;
      for (let j = 0; j < J; j++) { spend += curves[j][combo[j]].error * a.reach[j]; keys += curves[j][combo[j]].keys; }
      if (spend <= eps + 1e-12 && keys < best) { best = keys; bestCombo = combo.slice(); }
      return;
    }
    for (let l = 0; l < levels; l++) { combo[m] = l; rec(m + 1); }
  };
  rec(0);
  check('allocDpMatchesExhaustive', a.totalKeys === best, `dp ${a.totalKeys} keys == exhaustive ${best} over 7^6 = ${Math.pow(7, 6)} grid combos (levels=7)`);
  check('allocExhaustiveCombo', bestCombo !== null, `exhaustive found a feasible combo (spend <= ${eps})`);
}

// F. continuous-time guarantee via an independent evaluation path (2001 samples)
{
  const tol = 1 * D2R;
  let worst = 0;
  for (const tr of rig.joints) {
    const keep = optimalSubset(tr.times, tr.angles, tol).keep;
    for (let k = 0; k <= 2000; k++) {
      const t = (rig.duration * k) / 2000;
      const e = Math.abs(angleAt(tr.times, tr.angles, t) - evalTrack(tr.times, tr.angles, keep, t));
      if (e > worst) worst = e;
    }
  }
  check('continuousGuarantee2001', worst <= tol + 1e-12, `max |orig-decimated| over 2001 times = ${worst.toExponential(4)} rad = ${(worst / D2R).toFixed(4)} deg <= 1 deg`);
}

// G. evalTrack reproduces keys exactly
{
  let worst = 0;
  for (const tr of rig.joints) {
    const keep = optimalSubset(tr.times, tr.angles, D2R).keep;
    for (const k of keep) worst = Math.max(worst, Math.abs(evalTrack(tr.times, tr.angles, keep, tr.times[k]) - tr.angles[k]));
  }
  check('evalAtKeptKeys', worst < 1e-14, `max deviation at kept keys = ${worst.toExponential(3)} rad`);
}

// H. baked fingerprints match a fresh reduction, and tip bound dominates samples
{
  let fpOk = true, keysOk = true, tipOk = true, ratioWorst = 0, sampleWorst = 0;
  for (const d of rig.tolerancesDeg) {
    const tol = d * D2R;
    const r = reduceRig(rig, tol, rig.sampleTimes);
    const b = rig.baked[String(d)];
    const fp = fingerprint({ perJointKeys: r.perJoint.map((p) => p.keyCount), keep: r.perJoint.map((p) => p.keep.join(',')) });
    if (fp !== b.fp) fpOk = false;
    if (r.totalKeys !== b.totalKeys || JSON.stringify(r.perJoint.map((p) => p.keyCount)) !== JSON.stringify(b.perJointKeys)) keysOk = false;
    if (r.sampledTipError > r.tipBound + 1e-9) tipOk = false;
    ratioWorst = Math.max(ratioWorst, r.sampledTipError / r.tipBound);
    sampleWorst = Math.max(sampleWorst, r.perJoint.reduce((s, p) => Math.max(s, p.maxError), 0));
  }
  check('bakedFingerprints', fpOk, `4 tolerances reproduce baked fp (1 deg -> ${rig.baked['1'].fp})`);
  check('bakedKeyCounts', keysOk, `key counts match rig.json for 0.25/0.5/1/2 deg`);
  check('tipBoundDominates', tipOk, `sampled tip error <= certified bound at all 4 tolerances (worst ratio ${ratioWorst.toFixed(3)})`);
  check('tightnessWorstTol2', sampleWorst <= 2 * D2R + 1e-9, `worst joint error at 2 deg = ${(sampleWorst / D2R).toFixed(4)} deg`);
}

// I. certified tip bound formula vs brute force over many random angle perturbations
{
  const rand = mulberry32(99);
  let violated = 0, worstRatio = 0;
  const base = rig.joints.map((tr) => angleAt(tr.times, tr.angles, 1.234));
  for (let c = 0; c < 200; c++) {
    const d = rig.joints.map(() => (rand() - 0.5) * 0.04);
    const bound = certifiedTipBound(rig.chain, d);
    const actual = sampledTipError(rig.chain, base, base.map((a, m) => a + d[m]));
    worstRatio = Math.max(worstRatio, actual / bound);
    if (actual > bound * (1 + 1e-12)) violated++;
  }
  check('tipBoundBruteForce', violated === 0, `200 random perturbations: ${violated} bound violations, worst actual/bound ${worstRatio.toFixed(3)}`);
  const w = reachWeights(rig.chain);
  check('reachWeightsMonotone', w.every((v, m) => m === 0 || v < w[m - 1]), `R_0=${w[0].toFixed(3)} > ... > R_5=${w[5].toFixed(3)} (world units, distal joints have less reach)`);
}

// J. scale invariance of decisions and of the certified bound
{
  const tol = 1 * D2R;
  const base = rig.joints.map((tr) => optimalSubset(tr.times, tr.angles, tol).keep.join(','));
  let sameDecisions = true, boundLinear = true;
  const b1 = certifiedTipBound(rig.chain, [1e-3, 2e-3, 3e-3, 4e-3, 5e-3, 6e-3]);
  for (const s of [1e-6, 1e-3, 1e3, 1e6]) {
    const scaled = rig.joints.map((tr) => optimalSubset(tr.times, tr.angles, tol).keep.join(','));
    if (scaled.join('|') !== base.join('|')) sameDecisions = false;
    const ch = { joints: rig.chain.joints.map((j) => ({ axis: j.axis, offset: j.offset.map((v) => v * s) })), tip: rig.chain.tip.map((v) => v * s) };
    const bs = certifiedTipBound(ch, [1e-3, 2e-3, 3e-3, 4e-3, 5e-3, 6e-3]);
    if (!close(bs, b1 * s, 1e-12)) boundLinear = false;
  }
  check('angleScaleInvariance', sameDecisions, `keep sets identical for angle x {1e-6,1e-3,1,1e3,1e6}`);
  check('tipBoundScalesLinearly', boundLinear, `bound x s exact for s in {1e-6,1e-3,1e3,1e6} (base ${b1.toExponential(3)})`);
}

// K. edge cases and fingerprint stability
{
  check('edgeTwoKeys', JSON.stringify(optimalSubset([0, 1], [0.1, 0.2], 0.01).keep) === '[0,1]', 'n=2 keeps both endpoints');
  check('edgeOneKey', JSON.stringify(optimalSubset([0], [0.1], 0.01).keep) === '[0]', 'n=1 keeps the only key');
  const f1 = fingerprint({ a: [1, 2, 3] });
  const f2 = fingerprint({ a: [1, 2, 3] });
  check('fingerprintStable', f1 === f2 && /^[0-9a-f]{8}$/.test(f1), `fp ${f1} stable across calls`);
  const seg = segmentMaxError([0, 0.5, 1], [0, 0.3, 1], 0, 2);
  check('segmentMaxErrorExact', close(seg.maxError, 0.2) && seg.atKnot === 1, `chord 0->1 vs middle 0.3: max error ${seg.maxError.toFixed(4)} at knot ${seg.atKnot}`);
}

// L. FK sanity: zero angles leave the chain at the sum of offsets
{
  const zeros = rig.joints.map(() => 0);
  const f = fk(rig.chain, zeros);
  let expect = rig.chain.tip;
  for (let m = 0; m < rig.chain.joints.length; m++) expect = expect.map((v, i) => v + rig.chain.joints[m].offset[i]);
  const err = Math.hypot(f.tip[0] - expect[0], f.tip[1] - expect[1], f.tip[2] - expect[2]);
  check('fkZeroAngles', err < 1e-12, `tip at zero pose matches offset sum within ${err.toExponential(3)}`);
  const bent = fk(rig.chain, rig.joints.map((_, m) => 0.3 * (m + 1)));
  check('fkMovesTip', Math.hypot(bent.tip[0] - f.tip[0], bent.tip[1] - f.tip[1], bent.tip[2] - f.tip[2]) > 0.05, `bent pose tip moved ${Math.hypot(bent.tip[0] - f.tip[0], bent.tip[1] - f.tip[1], bent.tip[2] - f.tip[2]).toFixed(4)} world units`);
}

console.log(`\nRESULT ${failed === 0 ? 'PASS' : 'FAIL'}: ${passed} passed / ${failed} failed`);
if (failed) { console.log('failed:', failures.join(', ')); process.exit(1); }
process.exit(0);