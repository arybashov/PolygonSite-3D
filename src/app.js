import { DEFAULT_PARAMS, MAX_DRONES, PARAM_DEFS } from './constants.js';
import { initResultsPanel } from './charts.js';
import { RuleBasedDronePolicy, TeamWaypointMlDronePolicy } from './policies/dronePolicy.js';
import { CanvasRenderer } from './render.js';
import { Simulation } from './simulation.js';

const canvas       = document.getElementById('c');
const resetButton  = document.getElementById('btnReset');
const policyMode   = document.getElementById('policyMode');
const policyStatus = document.getElementById('policyStatus');
const controls     = new Map(PARAM_DEFS.map((def) => [def.id, document.getElementById(def.id)]));
const valueLabels  = new Map(PARAM_DEFS.map((def) => [def.id, document.getElementById(`${def.id}V`)]));

const sim      = new Simulation({ params: readParams(), seed: Date.now(), dronePolicy: createPolicy() });
const renderer = new CanvasRenderer(canvas, sim);

window.sim = sim;

syncControlLabels();
bindControls();
updateStats();
requestAnimationFrame(loop);

initResultsPanel(
  document.getElementById('btnResults'),
  document.getElementById('resultsOverlay'),
  document.getElementById('btnCloseResults'),
);

function bindControls() {
  resetButton.addEventListener('click', () => resetScenario());
  policyMode.addEventListener('change', () => resetScenario());

  for (const def of PARAM_DEFS) {
    const input = controls.get(def.id);
    input.addEventListener('input', () => {
      valueLabels.get(def.id).textContent = input.value;
    });
  }
}

function loop() {
  sim.setParams(readParams());
  sim.step();
  renderer.draw();
  updateStats();
  requestAnimationFrame(loop);
}

function resetScenario() {
  sim.setParams(readParams());
  sim.setDronePolicy(createPolicy());
  sim.reset({ seed: Date.now() });
  renderer.resetView();
  updateStats();
}

function readParams() {
  const params = { ...DEFAULT_PARAMS };
  for (const def of PARAM_DEFS) {
    const input = controls.get(def.id);
    params[def.id] = Number(input?.value ?? DEFAULT_PARAMS[def.id]);
  }
  return params;
}

function syncControlLabels() {
  for (const def of PARAM_DEFS) {
    const input = controls.get(def.id);
    const label = valueLabels.get(def.id);
    if (input) input.value = DEFAULT_PARAMS[def.id];
    if (label) label.textContent = DEFAULT_PARAMS[def.id];
  }
}

function updateStats() {
  const stats = sim.getStats();
  setText('sDrones', stats.drones);
  setText('sHits',   stats.hits);
  setText('sInt',    stats.intercepted);
  setText('sAD',     stats.activeAnti);
  setText('sTime',   stats.time);
  setText('policyStatus', getPolicyLabel());
  if (policyStatus) policyStatus.textContent = getPolicyLabel();
}

// ── ONNX / ML ──────────────────────────────────────────────────────────────
let _onnxSession = null;
let _onnxCache   = null;
let _onnxBusy    = false;
let _replayEp    = 0;

async function ensureOnnxSession() {
  if (_onnxSession) return;
  _onnxSession = await window.ort.InferenceSession.create('data/model_web.onnx');
}

function makeOnnxActionProvider() {
  _onnxCache = null;
  return (obs) => {
    if (!_onnxBusy && _onnxSession) {
      _onnxBusy = true;
      const t = new window.ort.Tensor('float32', Float32Array.from(obs), [1, obs.length]);
      _onnxSession.run({ obs: t })
        .then((out) => { _onnxCache = Array.from(out.action.data); })
        .catch(() => {})
        .finally(() => { _onnxBusy = false; });
    }
    return _onnxCache;
  };
}

function createPolicy() {
  if (policyMode?.value === 'onnx') {
    ensureOnnxSession().catch((e) => console.error('ONNX load failed:', e));
    return new TeamWaypointMlDronePolicy({ actionProvider: makeOnnxActionProvider() });
  }
  return new RuleBasedDronePolicy();
}

function getPolicyLabel() {
  return policyMode?.value === 'onnx' ? 'ONNX' : 'Rule-based';
}

window.addEventListener('startReplay', async () => {
  _replayEp++;
  setText('replayEp', _replayEp);
  const label = document.getElementById('replayLabel');
  if (label) label.classList.remove('hidden');
  policyMode.value = 'onnx';
  try { await ensureOnnxSession(); } catch (e) {
    console.error('ONNX load failed:', e);
    policyMode.value = 'rule';
    return;
  }
  resetScenario();
});

function setText(id, value) {
  const node = document.getElementById(id);
  if (node) node.textContent = value;
}
