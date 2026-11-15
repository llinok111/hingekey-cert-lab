// Timing: cold start (first reduceRig call in a fresh process) + 7 warm reps.
// Writes bench/result.json. No tuning, no sleeps: real measurements only.
import { readFileSync, writeFileSync } from 'node:fs';
import { reduceRig, greedySubset, uniformSubset, exactMaxError, fingerprint, reachWeights, allocateBudgets, uniformBudgetBaseline, reduceWithBudgets } from '../src/core.mjs';

const rig = JSON.parse(readFileSync(new URL('../data/rig.json', import.meta.url)));
const tolDeg = 1;
const tol = (tolDeg * Math.PI) / 180;
const reps = 7;

function ms(fn) { const t0 = process.hrtime.bigint(); const v = fn(); return { v, ms: Number(process.hrtime.bigint() - t0) / 1e6 }; }

// cold start: first reduction in this process
const cold = ms(() => reduceRig(rig, tol, rig.sampleTimes));
const warm = [];
for (let r = 0; r < reps; r++) warm.push(ms(() => reduceRig(rig, tol, rig.sampleTimes)).ms);

const sorted = [...warm].sort((a, b) => a - b);
const median = (arr) => arr.length % 2 ? arr[(arr.length - 1) / 2] : (arr[arr.length / 2 - 1] + arr[arr.length / 2]) / 2;
const pct = (arr, p) => arr[Math.min(arr.length - 1, Math.max(0, Math.ceil((p / 100) * arr.length) - 1))];
const mean = warm.reduce((a, b) => a + b, 0) / warm.length;
const stddev = Math.sqrt(warm.reduce((a, b) => a + (b - mean) ** 2, 0) / warm.length);
const med = median(sorted);

// baselines, once each (their cost is not the claim, but record it)
const g = ms(() => rig.joints.reduce((s, tr) => s + greedySubset(tr.times, tr.angles, tol).keep.length, 0));
const u = ms(() => rig.joints.reduce((s, tr) => s + uniformSubset(tr.times, tr.angles, tol).keep.length, 0));

// rig-level allocation: one tip-space budget, 6 joints, 12-level angle grid
const eps = 0.167;
const allocCold = ms(() => allocateBudgets(rig, eps));
const allocWarm = [];
for (let k = 0; k < 5; k++) allocWarm.push(ms(() => allocateBudgets(rig, eps)).ms);
const as = [...allocWarm].sort((a, b) => a - b);
const aMed = median(as);
const aMean = allocWarm.reduce((a, b) => a + b, 0) / allocWarm.length;
const aStd = Math.sqrt(allocWarm.reduce((a, b) => a + (b - aMean) ** 2, 0) / allocWarm.length);
const alloc = allocCold.v;
const allocSampled = reduceWithBudgets(rig, alloc.perJoint, rig.sampleTimes);
const uni = uniformBudgetBaseline(rig, eps);

const r = cold.v;
const perJoint = r.perJoint.map((p, m) => {
  const tr = rig.joints[m];
  const ex = exactMaxError(tr.times, tr.angles, p.keep);
  return { name: p.name, keys: p.keyCount, maxErrorRad: ex.maxError, maxErrorDeg: ex.maxError / (Math.PI / 180), fingerprint: fingerprint(p.keep) };
});
const bal = rig.baked[String(tolDeg)];
const fp = fingerprint({ perJointKeys: r.perJoint.map((p) => p.keyCount), keep: r.perJoint.map((p) => p.keep.join(',')) });

const result = {
  generatedAt: new Date().toISOString(),
  node: process.version,
  toleranceDeg: tolDeg,
  rig: { joints: rig.joints.length, keysPerJoint: rig.nKeys, totalKeys: rig.joints.length * rig.nKeys, fps: rig.fps, durationSec: rig.duration },
  decimation: {
    keysOriginal: rig.joints.length * rig.nKeys,
    keysKept: r.totalKeys,
    reductionPct: 100 * (1 - r.totalKeys / (rig.joints.length * rig.nKeys)),
    keysPerJoint: perJoint.map((p) => p.keys),
    worstJointErrorDeg: Math.max(...perJoint.map((p) => p.maxErrorDeg)),
    greedyKeys: g.v,
    greedyOverheadPct: 100 * (g.v / r.totalKeys - 1),
    uniformKeys: u.v,
    uniformOverheadPct: 100 * (u.v / r.totalKeys - 1),
    bakedMatch: fp === bal.fp,
    fingerprint: fp,
    bakedFingerprint: bal.fp,
  },
  tip: {
    certifiedBoundWorld: r.tipBound,
    sampledTipErrorWorld: r.sampledTipError,
    sampledOverBound: r.sampledTipError / r.tipBound,
    samples: rig.sampleTimes.length,
    reachWeights: reachWeights(rig.chain).map((v) => Number(v.toFixed(6))),
  },
  allocation: {
    epsTipWorld: eps,
    reachSum: reachWeights(rig.chain).reduce((a, b) => a + b, 0),
    keys: alloc.totalKeys,
    equalBudgetKeys: uni.totalKeys,
    savingVsEqualBudgetPct: 100 * (1 - alloc.totalKeys / uni.totalKeys),
    perJointKeys: alloc.perJoint.map((p) => p.keys),
    deltasDeg: alloc.perJoint.map((p) => p.deltaDeg),
    certifiedBound: alloc.certifiedBound,
    sampledTipError: allocSampled.sampledTipError,
    sampledOverBound: allocSampled.sampledTipError / alloc.certifiedBound,
    equalBudgetDeltaDeg: uni.delta / (Math.PI / 180),
    gridLevels: alloc.grid.length,
    quantum: alloc.quantum,
    fingerprint: fingerprint({ keys: alloc.perJoint.map((p) => p.keep.join(',')), deltas: alloc.perJoint.map((p) => p.deltaDeg) }),
    bakedFingerprint: bal.allocFp ?? rig.alloc[String(eps)].fp,
    timingMs: {
      coldStart: allocCold.ms,
      samples: allocWarm,
      n: allocWarm.length,
      min: as[0],
      median: aMed,
      p95: pct(as, 95),
      max: as[as.length - 1],
      mean: aMean,
      stddev: aStd,
      relStddevPct: (100 * aStd) / aMean,
    },
  },
  timingMs: {
    samples: warm,
    n: warm.length,
    coldStart: cold.ms,
    min: sorted[0],
    median: med,
    p25: pct(sorted, 25),
    p75: pct(sorted, 75),
    p95: pct(sorted, 95),
    max: sorted[sorted.length - 1],
    mean,
    stddev,
    relStddevPct: (100 * stddev) / mean,
    greedyTotal: g.ms,
    uniformTotal: u.ms,
  },
  notes: [
    'coldStart is the first reduceRig call in a fresh node process (includes data already parsed).',
    'median/p95 are over 7 repetitions of the same 6-track reduction at 1 degree.',
    'uniform baseline found the smallest feasible stride, so its key count is the cheapest uniform track that still meets the bound.',
  ],
};
writeFileSync(new URL('./result.json', import.meta.url), JSON.stringify(result, null, 2));

console.log(`cold start ${result.timingMs.coldStart.toFixed(2)} ms (${r.totalKeys} keys)`);
console.log(`warm reduceRig: median ${med.toFixed(2)} ms, p95 ${result.timingMs.p95.toFixed(2)}, stddev ${stddev.toFixed(2)} (${result.timingMs.relStddevPct.toFixed(2)}%)`);
console.log(`keys ${r.totalKeys}/${result.rig.totalKeys} (${result.decimation.reductionPct.toFixed(2)}% fewer) | greedy ${g.v} (+${result.decimation.greedyOverheadPct.toFixed(1)}%) | uniform ${u.v} (+${result.decimation.uniformOverheadPct.toFixed(1)}%)`);
console.log(`worst joint error ${result.decimation.worstJointErrorDeg.toFixed(4)} deg <= ${tolDeg} deg | tip bound ${r.tipBound.toFixed(4)} vs sampled ${r.sampledTipError.toFixed(4)} (${(100 * result.tip.sampledOverBound).toFixed(1)}% of bound)`);
console.log(`fp ${fp} baked ${bal.fp} match=${result.decimation.bakedMatch}`);
console.log(`alloc @eps ${eps}: ${alloc.totalKeys} keys vs equal-budget ${uni.totalKeys} (-${result.allocation.savingVsEqualBudgetPct.toFixed(1)}%), bound ${alloc.certifiedBound.toFixed(4)}, sampled ${allocSampled.sampledTipError.toFixed(4)}`);
console.log(`alloc timing: cold ${allocCold.ms.toFixed(1)} ms, median ${aMed.toFixed(1)}, p95 ${result.allocation.timingMs.p95.toFixed(1)}, relSD ${result.allocation.timingMs.relStddevPct.toFixed(2)}%`);