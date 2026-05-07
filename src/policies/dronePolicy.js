import {
  ARRIVAL_R,
  ATTACK_COMMIT_DIST,
  CRUISE_ALT,
  DEFAULT_DRONE_TURN_RATE,
  DRONE_DETECT_R,
  DRONE_FOV,
  MAX_ALT,
  MIN_ALT,
} from '../constants.js';
import { angleDiff, clamp, distance, distance3d, inCone } from '../math.js';

export class RuleBasedDronePolicy {
  getAction(sim, drone) {
    if (!drone.alive || drone.mode === 'hit' || drone.mode === 'intercepted') return { kind: 'idle' };

    if (sim.targets[drone.targetIdx]?.hit) {
      const closest = sim.findClosestUnattackedTarget(drone);
      if (!closest) return { kind: 'deactivate' };
      retarget(drone, closest);
    }

    const cruiseKmh  = sim.getParam('dspeed');
    const turnRadius = (drone.maxSpeed ?? cruiseKmh / 3.6) / (DEFAULT_DRONE_TURN_RATE);

    if (drone.evadeCooldown > 0) drone.evadeCooldown -= sim.dt;

    const distToTarget = distance(drone.x, drone.y, drone.rtx, drone.rty);

    if (distToTarget < ATTACK_COMMIT_DIST) {
      clearEvasion(drone);
      drone.tx = drone.rtx; drone.ty = drone.rty; drone.tz = MIN_ALT + 5; // dive low for final approach
      drone.mode = 'approach';
      if (distToTarget < ARRIVAL_R) return { kind: 'hitTarget' };
    } else {
      const threat = findThreat(sim, drone);
      if (threat) {
        if (!drone.evading) {
          const closest = sim.findClosestUnattackedTarget(drone);
          if (closest) retarget(drone, closest);
        }
        if (drone.threatId !== threat.id || !drone.evadeWpt || drone.evadeCooldown <= 0) {
          const result = computeEvadeWaypoint(sim, drone, threat);
          drone.evadeWpt   = result.wpt;
          drone.predictPt  = result.pred;
          drone.threatId   = threat.id;
          drone.evadeCooldown = 0.5;
        }
        drone.evading = true;
        drone.tx = drone.evadeWpt.x;
        drone.ty = drone.evadeWpt.y;
        drone.tz = drone.evadeWpt.z;
        drone.mode = 'evade';
      } else {
        updateNonThreatFlight(drone, turnRadius);
        if (drone.mode !== 'evade' && drone.mode !== 'reapproach') {
          if (distToTarget < ARRIVAL_R) return { kind: 'hitTarget' };
        }
      }
    }

    return {
      kind:      'guidance',
      tx:        drone.tx,
      ty:        drone.ty,
      tz:        drone.tz ?? CRUISE_ALT,
      intent:    drone.mode === 'evade' ? 'evade' : 'attack',
      mode:      drone.mode,
      cruiseKmh,
    };
  }
}

// ML policies (stubs — observation encoding needs 3D update)
export class TeamWaypointMlDronePolicy {
  constructor({ actionProvider, deploymentProvider, fallback = new RuleBasedDronePolicy() } = {}) {
    this.actionProvider      = actionProvider;
    this.deploymentProvider  = deploymentProvider;
    this.fallback            = fallback;
  }

  getDeployment(sim, droneCount) {
    if (!this.deploymentProvider) return null;
    try {
      const obs = encodeDeploymentObservation(sim, droneCount);
      const out = this.deploymentProvider(obs, sim, droneCount);
      return out ? decodePerimeterDeployment(out, droneCount) : null;
    } catch { return null; }
  }

  getTeamActions(sim) {
    if (!this.actionProvider) return buildFallbackActions(sim, this.fallback);
    try {
      const obs = encodeTeamObservation(sim);
      const out = this.actionProvider(obs, sim);
      if (!out) return buildFallbackActions(sim, this.fallback);
      const actions = decodeTeamWaypointActions(out, sim);
      sim.drones.forEach((drone) => {
        const ma = getMaintenanceAction(sim, drone);
        if (ma) { actions.set(drone.id, ma); return; }
        if (!actions.get(drone.id)) actions.set(drone.id, this.fallback.getAction(sim, drone));
      });
      return actions;
    } catch { return buildFallbackActions(sim, this.fallback); }
  }
}

function buildFallbackActions(sim, fallback) {
  const actions = new Map();
  sim.drones.forEach((d) => actions.set(d.id, fallback.getAction(sim, d)));
  return actions;
}

function getMaintenanceAction(sim, drone) {
  if (!drone.alive || drone.mode === 'hit' || drone.mode === 'intercepted') return { kind: 'idle' };
  if (sim.targets[drone.targetIdx]?.hit) {
    const closest = sim.findClosestUnattackedTarget(drone);
    if (!closest) return { kind: 'deactivate' };
    retarget(drone, closest);
  }
  if (distance(drone.x, drone.y, drone.rtx, drone.rty) < ARRIVAL_R) return { kind: 'hitTarget' };
  return null;
}

function retarget(drone, closest) {
  drone.targetIdx = closest.i;
  drone.rtx = closest.t.x; drone.rty = closest.t.y; drone.rtz = CRUISE_ALT;
  drone.tx  = closest.t.x; drone.ty  = closest.t.y; drone.tz  = CRUISE_ALT;
  drone.mode = 'approach';
}

function clearEvasion(drone) {
  drone.evading   = false;
  drone.evadeWpt  = null;
  drone.predictPt = null;
  drone.threatId  = null;
}

function findThreat(sim, drone) {
  return sim.antidrones.find((anti) => {
    return anti.alive && anti.mode !== 'base'
      && inCone(drone.x, drone.y, drone.angle, DRONE_FOV, anti.x, anti.y, DRONE_DETECT_R);
  }) ?? null;
}

function computeEvadeWaypoint(sim, drone, threat) {
  const droneSpeed = drone.maxSpeed ?? sim.getParam('dspeed') / 3.6;
  const antiSpeed  = sim.getParam('aspeed') / 3.6;
  const avx = Math.cos(threat.angle) * antiSpeed;
  const avy = Math.sin(threat.angle) * antiSpeed;
  const distDA = distance(drone.x, drone.y, threat.x, threat.y);
  const timeToCross = Math.max(1, distDA / (droneSpeed + antiSpeed));
  const predX = threat.x + avx * timeToCross;
  const predY = threat.y + avy * timeToCross;
  const lineAngle = Math.atan2(predY - threat.y, predX - threat.x);
  const perpAngle = lineAngle + Math.PI / 2;
  const safeR  = Math.max(150, distDA * 0.4);
  const midX   = (drone.x + predX) / 2;
  const midY   = (drone.y + predY) / 2;
  const c1x = midX + Math.cos(perpAngle) * safeR;
  const c1y = midY + Math.sin(perpAngle) * safeR;
  const c2x = midX - Math.cos(perpAngle) * safeR;
  const c2y = midY - Math.sin(perpAngle) * safeR;
  const d1 = distance(c1x, c1y, drone.rtx, drone.rty);
  const d2 = distance(c2x, c2y, drone.rtx, drone.rty);

  // Altitude evasion: pick altitude away from threat's current altitude
  const evadeZ = threat.z > (drone.z ?? CRUISE_ALT)
    ? clamp((drone.z ?? CRUISE_ALT) - 40, MIN_ALT, MAX_ALT)
    : clamp((drone.z ?? CRUISE_ALT) + 40, MIN_ALT, MAX_ALT);

  return {
    wpt:  d1 < d2
      ? { x: c1x, y: c1y, z: evadeZ }
      : { x: c2x, y: c2y, z: evadeZ },
    pred: { x: predX, y: predY },
  };
}

function updateNonThreatFlight(drone, turnRadius) {
  if (drone.evading) {
    clearEvasion(drone);
    drone.tx = drone.x + Math.cos(drone.angle) * turnRadius * 2;
    drone.ty = drone.y + Math.sin(drone.angle) * turnRadius * 2;
    drone.tz = CRUISE_ALT;
    drone.mode = 'reapproach';
    return;
  }

  if (drone.mode === 'reapproach') {
    if (distance(drone.x, drone.y, drone.tx, drone.ty) < turnRadius) {
      drone.tx = drone.rtx; drone.ty = drone.rty; drone.tz = drone.rtz ?? CRUISE_ALT;
      drone.mode = 'approach';
    }
    return;
  }

  if (drone.mode !== 'bypass') drone.mode = 'approach';
  const rdist = distance(drone.x, drone.y, drone.rtx, drone.rty);
  const targetAngle = Math.atan2(drone.rty - drone.y, drone.rtx - drone.x);
  const headingError = Math.abs(angleDiff(drone.angle, targetAngle));
  const inDeadZone = rdist < turnRadius * 1.5 && headingError > Math.PI / 3;

  if (inDeadZone && drone.mode !== 'bypass') {
    drone.deadZone = true;
    drone.mode = 'bypass';
    const cross = Math.cos(drone.angle) * (drone.rty - drone.y) - Math.sin(drone.angle) * (drone.rtx - drone.x);
    const side  = cross > 0 ? 1 : -1;
    drone.tx = drone.rtx + Math.sin(drone.angle) * turnRadius * 1.5 * side;
    drone.ty = drone.rty - Math.cos(drone.angle) * turnRadius * 1.5 * side;
    drone.tz = CRUISE_ALT;
    return;
  }

  if (!inDeadZone) {
    drone.deadZone = false;
    if (drone.mode === 'bypass') {
      if (distance(drone.x, drone.y, drone.tx, drone.ty) < ARRIVAL_R * 4) {
        drone.mode = 'approach';
        drone.tx = drone.rtx; drone.ty = drone.rty; drone.tz = drone.rtz ?? CRUISE_ALT;
      }
    } else {
      drone.tx = drone.rtx; drone.ty = drone.rty; drone.tz = drone.rtz ?? CRUISE_ALT;
    }
  }
}

// ── Stub imports (ML observations not yet ported to 3D) ───────────────────
function encodeDeploymentObservation() { return []; }
function encodeTeamObservation()       { return []; }
function decodeTeamWaypointActions()   { return new Map(); }
function decodePerimeterDeployment()   { return []; }
