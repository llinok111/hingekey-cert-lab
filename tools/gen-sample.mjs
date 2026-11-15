// Deterministic sample generator: 6 hinge-joint tracks at 60 Hz, 4 s (241 keys).
// Non-stationary on purpose: slow base motion + amplitude-modulated mid band +
// small fast ripple, so uniform stride oversamples quiet spans and overshoots
// fast ones (that is the baseline we compare against).
import { writeFileSync, mkdirSync } from 'node:fs';
import { reduceRig, exactMaxError, fingerprint, allocateBudgets, reduceWithBudgets, uniformBudgetBaseline } from '../src/core.mjs';

function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const seed = 0x51fb848;
const rand = mulberry32(seed);
const fps = 60, duration = 4, nKeys = fps * duration + 1;
const times = Array.from({ length: nKeys }, (_, k) => k / fps);

const axes = [[0, 1, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0]];
const joints = [];
for (let m = 0; m < 6; m++) {
  // Heterogeneous on purpose: proximal joints are slow and heavy (large reach),
  // distal joints are fast and light. That is what makes rig-level budget
  // allocation beat one equal angle tolerance for every joint.
  const A = 0.55 + rand() * 0.55;
  const B = 0.08 + 0.10 * (m / 5) + rand() * 0.05;
  const C = 0.02 + 0.09 * (m / 5) + rand() * 0.02;
  const f1 = 0.18 + rand() * 0.22;          // Hz, slow
  const f2 = 0.8 + 1.2 * (m / 5);           // Hz, mid (faster distally)
  const f3 = 3 + 3 * (m / 5);               // Hz, ripple (faster distally)
  const p1 = rand() * 6.283, p2 = rand() * 6.283, p3 = rand() * 6.283;
  const envRate = 0.25 + rand() * 0.3, envPhase = rand() * 6.283;
  const angles = times.map((t) => {
    const env = 0.55 + 0.45 * Math.sin(2 * Math.PI * envRate * t + envPhase);
    return A * Math.sin(2 * Math.PI * f1 * t + p1)
      + B * env * Math.sin(2 * Math.PI * f2 * t + p2)
      + C * Math.sin(2 * Math.PI * f3 * t + p3);
  });
  joints.push({
    name: `j${m}`,
    axis: axes[m],
    offset: m === 0 ? [0, 0, 0] : [0.34 + rand() * 0.26, 0, 0],
    times: times.slice(),
    angles,
  });
}

const chain = {
  joints: joints.map((j) => ({ axis: j.axis, offset: j.offset })),
  tip: [0.28 + 0.1 * rand(), 0, 0],
};

const tolerancesDeg = [0.25, 0.5, 1, 2];
const tipBudgets = [0.02, 0.05, 0.167, 0.25];
const sampleTimes = Array.from({ length: 801 }, (_, k) => (duration * k) / 800);
const baked = {};
for (const d of tolerancesDeg) {
  const tol = (d * Math.PI) / 180;
  const r = reduceRig({ joints, chain }, tol, sampleTimes);
  baked[String(d)] = {
    perJointKeys: r.perJoint.map((p) => p.keyCount),
    totalKeys: r.totalKeys,
    maxErrorRad: Math.max(...r.perJoint.map((p) => p.maxError)),
    tipBound: r.tipBound,
    sampledTipError: r.sampledTipError,
    fp: fingerprint({
      perJointKeys: r.perJoint.map((p) => p.keyCount),
      keep: r.perJoint.map((p) => p.keep.join(',')),
    }),
  };
}

// Rig-level allocation: one end-effector budget, distributed across joints.
const alloc = {};
for (const eps of tipBudgets) {
  const a = allocateBudgets({ joints, chain }, eps);
  const r = reduceWithBudgets({ joints, chain }, a.perJoint, sampleTimes);
  const u = uniformBudgetBaseline({ joints, chain }, eps);
  if (a.certifiedBound > eps + 1e-12) throw new Error(`alloc bound ${a.certifiedBound} > eps ${eps}`);
  if (r.sampledTipError > a.certifiedBound + 1e-12) throw new Error(`sampled ${r.sampledTipError} > bound ${a.certifiedBound}`);
  alloc[String(eps)] = {
    totalKeys: a.totalKeys,
    perJointKeys: a.perJoint.map((p) => p.keys),
    deltasDeg: a.perJoint.map((p) => p.deltaDeg),
    certifiedBound: a.certifiedBound,
    sampledTipError: r.sampledTipError,
    uniformKeys: u.totalKeys,
    uniformDeltaDeg: u.delta / (Math.PI / 180),
    fp: fingerprint({ keys: a.perJoint.map((p) => p.keep.join(',')), deltas: a.perJoint.map((p) => p.deltaDeg) }),
  };
}

// Independent re-check of the baked headline before it is written.
for (const d of tolerancesDeg) {
  const tol = (d * Math.PI) / 180;
  const r = reduceRig({ joints, chain }, tol, sampleTimes);
  for (let m = 0; m < joints.length; m++) {
    const e = exactMaxError(joints[m].times, joints[m].angles, r.perJoint[m].keep);
    if (e.maxError > tol + 1e-12) throw new Error(`baked track ${m} violates tol ${d}`);
  }
  if (r.sampledTipError > r.tipBound + 1e-9) throw new Error(`tip sample ${r.sampledTipError} exceeds bound ${r.tipBound}`);
}

mkdirSync(new URL('../data/', import.meta.url), { recursive: true });
const payload = {
  seed, fps, duration, nKeys, tolerancesDeg,
  joints, chain, sampleTimes, baked, tipBudgets, alloc,
};
writeFileSync(new URL('../data/rig.json', import.meta.url), JSON.stringify(payload));
console.log(`wrote data/rig.json: ${joints.length} joints x ${nKeys} keys, tolerances ${tolerancesDeg.join('/')} deg`);
for (const d of tolerancesDeg) console.log(`  tol ${d}deg -> totalKeys ${baked[String(d)].totalKeys} fp ${baked[String(d)].fp}`);