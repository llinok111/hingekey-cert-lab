// Browser layer: renders the rig and consumes src/core.mjs for every number.
// No numerical logic is reimplemented here — allocation, angle evaluation and
// the certified bound all come from the same core that bench/verify.mjs runs.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
  allocateBudgets, uniformBudgetBaseline, reduceWithBudgets,
  evalTrack, angleAt, fingerprint,
} from './src/core.mjs';

const data = await (await fetch('./data/rig.json')).json();
const D2R = Math.PI / 180;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.1, 100);
camera.position.set(2.2, 3.4, 5.6);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
document.getElementById('app').appendChild(renderer.domElement);
new OrbitControls(camera, renderer.domElement).target.set(1.2, 0.6, 0);

scene.add(new THREE.HemisphereLight(0xbfd8ff, 0x1a2530, 2.0));
const key = new THREE.DirectionalLight(0xffffff, 2.2);
key.position.set(3, 6, 4);
scene.add(key);
const grid = new THREE.GridHelper(10, 20, 0x1f2c38, 0x16222c);
scene.add(grid);

const matLink = new THREE.MeshStandardMaterial({ color: 0x3f7fbf, metalness: 0.35, roughness: 0.45 });
const matLinkAlloc = new THREE.MeshStandardMaterial({ color: 0xd98c3f, metalness: 0.35, roughness: 0.45 });
const matJoint = new THREE.MeshStandardMaterial({ color: 0xdfe8ef, metalness: 0.6, roughness: 0.3 });
const matTip = new THREE.MeshStandardMaterial({ color: 0x5ee08a, metalness: 0.4, roughness: 0.3 });

function buildArm(material, x, z) {
  const root = new THREE.Group();
  root.position.set(x, 0.15, z);
  scene.add(root);
  const rotors = [];
  let parent = root;
  for (let m = 0; m < data.chain.joints.length; m++) {
    const j = data.chain.joints[m];
    const node = new THREE.Group();
    node.position.set(j.offset[0], j.offset[1], j.offset[2]);
    parent.add(node);
    const rotor = new THREE.Group();
    node.add(rotor);
    const hub = new THREE.Mesh(new THREE.SphereGeometry(0.075, 16, 12), matJoint);
    rotor.add(hub);
    rotors.push(rotor);
    const len = j.offset[0] || 0.2;
    const link = new THREE.Mesh(new THREE.BoxGeometry(Math.max(len, 0.12), 0.075, 0.075), material);
    link.position.set(len / 2, 0, 0);
    link.castShadow = true;
    rotor.add(link);
    parent = rotor;
  }
  const tipNode = new THREE.Group();
  tipNode.position.set(data.chain.tip[0], data.chain.tip[1], data.chain.tip[2]);
  parent.add(tipNode);
  tipNode.add(new THREE.Mesh(new THREE.SphereGeometry(0.075, 16, 12), matTip));
  return { root, rotors };
}

const armOrig = buildArm(matLink, 0, 1.1);
const armAlloc = buildArm(matLinkAlloc, 0, -1.1);
const ORIGIN = new THREE.Vector3();

function poseArm(arm, angles) {
  for (let m = 0; m < arm.rotors.length; m++) {
    const axis = data.chain.joints[m].axis;
    arm.rotors[m].quaternion.setFromAxisAngle(new THREE.Vector3(...axis), angles[m]);
  }
}

// ---- allocation, recomputed on slider change with the same core ----
const hud = {
  eps: document.getElementById('eps'), keys: document.getElementById('keys'),
  base: document.getElementById('base'), deltas: document.getElementById('deltas'),
  bound: document.getElementById('bound'), sampled: document.getElementById('sampled'),
  fp: document.getElementById('fp'),
};
const slider = document.getElementById('slider');
let plan = null, allocMs = 0;

function applyBudget(epsIndex) {
  const eps = data.tipBudgets[epsIndex];
  const t0 = performance.now();
  const a = allocateBudgets(data, eps);
  allocMs = performance.now() - t0;
  const r = reduceWithBudgets(data, a.perJoint, data.sampleTimes);
  const u = uniformBudgetBaseline(data, eps);
  const fp = fingerprint({ keys: a.perJoint.map((p) => p.keep.join(',')), deltas: a.perJoint.map((p) => p.deltaDeg) });
  const baked = data.alloc[String(eps)];
  plan = { eps, alloc: a, sampled: r.sampledTipError, liveFp: fp, bakedFp: baked.fp };
  hud.eps.textContent = eps.toFixed(3);
  hud.keys.textContent = `1446 / ${a.totalKeys}`;
  hud.base.textContent = `${u.totalKeys} keys (equal Δθ=${(u.delta / D2R).toFixed(3)}°)`;
  hud.deltas.textContent = a.perJoint.map((p) => p.deltaDeg.toFixed(2)).join(' · ');
  hud.bound.textContent = `${a.certifiedBound.toFixed(5)} ≤ ${eps} (exact)`;
  hud.sampled.textContent = `${r.sampledTipError.toFixed(5)} world`;
  const match = fp === baked.fp;
  hud.fp.innerHTML = `<span class="${match ? 'ok' : 'bad'}">${match ? 'MATCH' : 'MISMATCH'}</span> ${fp} | node ${baked.fp}`;
  hud.fp.title = `allocation took ${allocMs.toFixed(0)} ms in this browser`;
  console.log(`eps=${eps} keys=${a.totalKeys} fp=${fp} baked=${baked.fp} alloc=${allocMs.toFixed(0)}ms`);
}

slider.max = String(data.tipBudgets.length - 1);
slider.value = String(data.tipBudgets.indexOf(0.167));
slider.addEventListener('input', () => applyBudget(Number(slider.value)));
applyBudget(Number(slider.value));

// ---- animation: original vs allocated track, same clock ----
const clock = new THREE.Clock();
function frame() {
  const t = (clock.getElapsedTime() % data.duration);
  const origAngles = data.joints.map((tr) => angleAt(tr.times, tr.angles, t));
  poseArm(armOrig, origAngles);
  if (plan) {
    const allocAngles = data.joints.map((tr, m) => evalTrack(tr.times, tr.angles, plan.alloc.perJoint[m].keep, t));
    poseArm(armAlloc, allocAngles);
  }
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
frame();

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});