// Baseline comparators: uniform stride and greedy RDP split. Both are common
// practice; both are feasible but not minimum-cardinality. Run directly:
//   node baseline/naive.mjs
import { readFileSync } from 'node:fs';
import { exactMaxError, greedySubset, uniformSubset, optimalSubset } from '../src/core.mjs';

const rig = JSON.parse(readFileSync(new URL('../data/rig.json', import.meta.url)));
const tolDeg = 1;
const tol = (tolDeg * Math.PI) / 180;

let totalOpt = 0, totalGreedy = 0, totalUniform = 0;
for (const tr of rig.joints) {
  const o = optimalSubset(tr.times, tr.angles, tol).keep;
  const g = greedySubset(tr.times, tr.angles, tol).keep;
  const ur = uniformSubset(tr.times, tr.angles, tol);
  const u = ur.keep;
  totalOpt += o.length; totalGreedy += g.length; totalUniform += u.length;
  console.log(
    `${tr.name}: optimal ${o.length} (${exactMaxError(tr.times, tr.angles, o).maxError.toExponential(3)} rad) |` +
    ` greedy ${g.length} (${exactMaxError(tr.times, tr.angles, g).maxError.toExponential(3)}) |` +
    ` uniform ${u.length} stride ${ur.stride} (${exactMaxError(tr.times, tr.angles, u).maxError.toExponential(3)})`
  );
}
const nTotal = rig.joints.reduce((s, t) => s + t.times.length, 0);
console.log(`totals @${tolDeg}deg: optimal ${totalOpt} (${(100 * (1 - totalOpt / nTotal)).toFixed(2)}% fewer) | greedy ${totalGreedy} (+${(100 * (totalGreedy / totalOpt - 1)).toFixed(1)}%) | uniform ${totalUniform} (+${(100 * (totalUniform / totalOpt - 1)).toFixed(1)}%)`);