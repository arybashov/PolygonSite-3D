import { DEFAULT_PARAMS, PARAM_DEFS } from './constants.js';
import { RuleBasedDronePolicy } from './policies/dronePolicy.js';
import { CanvasRenderer, ElevationRenderer } from './render.js';
import { Simulation } from './simulation.js';

const canvas       = document.getElementById('c');
const elevCanvas   = document.getElementById('elev');
const resetButton  = document.getElementById('btnReset');
const controls     = new Map(PARAM_DEFS.map((def) => [def.id, document.getElementById(def.id)]));
const valueLabels  = new Map(PARAM_DEFS.map((def) => [def.id, document.getElementById(`${def.id}V`)]));

const sim          = new Simulation({ params: readParams(), seed: Date.now() });
const renderer     = new CanvasRenderer(canvas, sim);
const elevRenderer = new ElevationRenderer(elevCanvas, sim, renderer);

window.sim = sim;

syncControlLabels();
bindControls();
updateStats();
requestAnimationFrame(loop);

function bindControls() {
  resetButton.addEventListener('click', () => resetScenario());

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
  elevRenderer.draw();
  updateStats();
  requestAnimationFrame(loop);
}

function resetScenario() {
  sim.setParams(readParams());
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
}

function setText(id, value) {
  const node = document.getElementById(id);
  if (node) node.textContent = value;
}
