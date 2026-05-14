import {
  ARRIVAL_R,
  ATTACK_COMMIT_DIST,
  CRUISE_ALT,
  DEFAULT_DRONE_TURN_RATE,
  DRONE_DETECT_R,
  DRONE_MAX_VZ,
  DRONE_TARGET_HIT_ALT,
  MAX_ALT,
  MIN_ALT,
} from '../constants.js';
import { angleDiff, clamp, distance, distance3d } from '../math.js';

const ATTACK_ALT      = 50;   // m  — altitude drone climbs to before diving
const DIVE_START_ALT  = 45;   // m  — minimum altitude before final dive is allowed
const ATTACK_MAX_HEADING_ERROR = 55 * Math.PI / 180;
const ATTACK_REAPPROACH_DIST = 250; // m

export class RuleBasedDronePolicy {
  getTeamActions(sim) {
    const actions = new Map();
    sim.drones.forEach((drone) => actions.set(drone.id, this.getAction(sim, drone)));
    return actions;
  }

  getAction(sim, drone) {
    if (!drone.alive || drone.mode === 'hit' || drone.mode === 'intercepted') return { kind: 'idle' };

    if (sim.targets[drone.targetIdx]?.hit) {
      if (drone.mode === 'climb' || drone.mode === 'dive') return { kind: 'deactivate' };
      const closest = sim.findClosestUnattackedTarget(drone);
      if (!closest) return { kind: 'deactivate' };
      retarget(drone, closest);
    }

    const cruiseKmh  = sim.getParam('dspeed');
    const turnRadius = (drone.maxSpeed ?? cruiseKmh / 3.6) / (DEFAULT_DRONE_TURN_RATE);

    if (drone.evadeCooldown > 0) drone.evadeCooldown -= sim.dt;

    const distToTarget = distance(drone.x, drone.y, drone.rtx, drone.rty);
    const cruiseSpeed = cruiseKmh / 3.6;

    const attackCommitted = drone.mode === 'climb' || drone.mode === 'dive';
    const attackFeasible = canCommitAttack(drone, distToTarget, cruiseSpeed, turnRadius);

    if (attackCommitted || (distToTarget < ATTACK_COMMIT_DIST && attackFeasible)) {
      clearEvasion(drone);
      drone.tx = drone.rtx;
      drone.ty = drone.rty;

      // Final hit check: drone must be close horizontally and near the ground.
      if (distToTarget < ARRIVAL_R && (drone.z ?? 0) <= DRONE_TARGET_HIT_ALT) {
        return { kind: 'hitTarget' };
      }

      // Once in dive — stay in dive (prevents re-climb when z drops back to 0).
      // Start dive when footprint at dive speed covers remaining distance.
      const diveZ = drone.z ?? 0;
      const diveSpeed = cruiseKmh / 3.6;
      const hitDescentAlt = Math.max(0, Math.max(diveZ, DIVE_START_ALT) - DRONE_TARGET_HIT_ALT);
      const cleanDiveFootprint = Math.max(80, hitDescentAlt / DRONE_MAX_VZ * diveSpeed);
      const tooLateForCleanDive = distToTarget < cleanDiveFootprint;
      if (drone.mode !== 'dive' && tooLateForCleanDive) {
        startAttackReapproach(drone, turnRadius);
        return { kind: 'guidance', tx: drone.tx, ty: drone.ty, tz: drone.tz,
                 intent: 'attack', mode: drone.mode, cruiseKmh };
      }
      const diveFootprint = Math.max(80, diveZ / DRONE_MAX_VZ * diveSpeed);
      const onDive = drone.mode === 'dive'
        || (diveZ >= DIVE_START_ALT && distToTarget < diveFootprint);

      if (!onDive) {
        drone.tz   = ATTACK_ALT;
        drone.mode = 'climb';
        return { kind: 'guidance', tx: drone.rtx, ty: drone.rty, tz: ATTACK_ALT,
                 intent: 'attack', mode: 'climb', cruiseKmh };
      }

      // Dive phase: keep max configured horizontal speed, let vz bring drone to ground.
      drone.tz   = 0;
      drone.mode = 'dive';
      return { kind: 'guidance', tx: drone.rtx, ty: drone.rty, tz: 0,
               intent: 'attack', mode: 'dive', cruiseKmh };
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
        if (distToTarget < ATTACK_COMMIT_DIST && !attackFeasible
            && drone.mode !== 'reapproach' && drone.mode !== 'bypass') {
          startAttackReapproach(drone, turnRadius);
        } else {
          updateNonThreatFlight(drone, turnRadius);
        }
        if (drone.mode !== 'evade' && drone.mode !== 'reapproach') {
          if (distToTarget < ARRIVAL_R && (drone.z ?? 0) <= DRONE_TARGET_HIT_ALT) return { kind: 'hitTarget' };
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

function canCommitAttack(drone, distToTarget, cruiseSpeed, turnRadius) {
  if (drone.mode === 'reapproach' || drone.mode === 'bypass') return false;

  const targetYaw = Math.atan2(drone.rty - drone.y, drone.rtx - drone.x);
  const headingError = Math.abs(angleDiff(drone.angle, targetYaw));
  if (headingError > ATTACK_MAX_HEADING_ERROR) return false;

  const z = drone.z ?? CRUISE_ALT;
  const climbAlt = Math.max(0, ATTACK_ALT - z);
  const diveAlt = Math.max(DIVE_START_ALT, z, ATTACK_ALT);
  const climbDistance = climbAlt / DRONE_MAX_VZ * cruiseSpeed;
  const diveDistance = diveAlt / DRONE_MAX_VZ * cruiseSpeed;
  const turnDistance = headingError * turnRadius;
  const requiredDistance = climbDistance + diveDistance + turnDistance + ARRIVAL_R;

  return distToTarget > requiredDistance;
}

function startAttackReapproach(drone, turnRadius) {
  clearEvasion(drone);
  const ahead = Math.max(ATTACK_REAPPROACH_DIST, turnRadius * 3);
  drone.tx = drone.x + Math.cos(drone.angle) * ahead;
  drone.ty = drone.y + Math.sin(drone.angle) * ahead;
  drone.tz = CRUISE_ALT;
  drone.mode = 'reapproach';
}

function findThreat(sim, drone) {
  return sim.antidrones.find((anti) => {
    return anti.alive && anti.mode !== 'base'
      && distance3d(drone.x, drone.y, drone.z ?? 0, anti.x, anti.y, anti.z ?? 0) <= DRONE_DETECT_R;
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

  // Altitude evasion: always relative to cruise alt, not current drone alt (prevents accumulation)
  const preferredEvadeZ = threat.z > CRUISE_ALT
    ? clamp(CRUISE_ALT - 15, MIN_ALT, MAX_ALT)
    : clamp(CRUISE_ALT + 40, MIN_ALT, MAX_ALT);
  const evadeZ = drone.evadeWpt?.z ?? preferredEvadeZ;

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
