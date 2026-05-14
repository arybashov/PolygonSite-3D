import {
  ANTI_FOV,
  ANTI_MAX_VZ,
  ANTI_RESPONSE_TIME,
  ARRIVAL_R,
  BASE_X,
  BASE_Y,
  CAMERA_COUNT,
  CAMERA_Z,
  CAM_ELEV_R,
  CAM_FOV,
  CENTER_ZONE,
  CRUISE_ALT,
  DEFAULT_ANTI_TURN_RATE,
  DEFAULT_DRONE_TURN_RATE,
  DEFAULT_PARAMS,
  DEPLOYMENT_MARGIN,
  DRONE_COLS,
  DRONE_MAX_VZ,
  DRONE_RESPONSE_TIME,
  DRONE_STALL_KMH,
  DRONE_TARGET_HIT_ALT,
  DT,
  FIELD_SIZE,
  INTERCEPT_R,
  MAX_ALT,
  MIN_ALT,
} from './constants.js';
import { angleDiff, clamp, createRng, distance, distance3d, inCone3d, inCone3dFull, lerp } from './math.js';
import { applyPhysics, dragCoefForSpeed, navigateFixedWingToPoint, navigateToPoint } from './physics.js';
import { RuleBasedDronePolicy } from './policies/dronePolicy.js';

const GROUND_CRASH_ALT = 1;  // m
const ANTI_BARO_FLOOR  = MIN_ALT;
const ANTI_SEARCH_ALT  = 45; // m, above camera masts
const ANTI_LOST_VISUAL_GRACE = 3.0; // s

export class Simulation {
  constructor({ params = {}, seed = Date.now(), dronePolicy = new RuleBasedDronePolicy() } = {}) {
    this.dt = DT;
    this.params = { ...DEFAULT_PARAMS, ...params };
    this.seed = seed;
    this.rng = createRng(seed);
    this.dronePolicy = dronePolicy;

    this.simTime = 0;
    this.drones = [];
    this.targets = [];
    this.cameras = [];
    this.antidrones = [];
    this.explosions = [];

    this.reset({ seed });
  }

  setParams(params) {
    for (const [key, value] of Object.entries(params)) {
      if (Number.isFinite(Number(value))) this.params[key] = Number(value);
    }
  }

  getParam(id) { return Number(this.params[id]); }

  setDronePolicy(policy) { this.dronePolicy = policy; }

  reset({ seed = Date.now() } = {}) {
    this.seed = seed;
    this.rng = createRng(seed);
    this.simTime = 0;
    this.drones = [];
    this.targets = [];
    this.cameras = [];
    this.antidrones = [];
    this.explosions = [];

    const droneCount = Math.floor(this.getParam('ndrones'));
    const antiCount  = Math.floor(this.getParam('nanti'));

    const targets = Array.from({ length: Math.max(0, droneCount - 1) }, () => this.spawnTarget());
    targets.push({ x: BASE_X, y: BASE_Y, z: 0, hit: false });
    this.targets.push(...targets);

    this.cameras = this.placeCamerasAroundTargets();

    for (let i = 0; i < antiCount; i++) this.antidrones.push(this.makeAnti(i, antiCount));

    const spawns  = this.getDeploymentSpawns(droneCount);
    const assigned = new Set();
    spawns.forEach((spawn, i) => {
      let bestIdx = -1, bestDist = Infinity;
      targets.forEach((target, ti) => {
        if (assigned.has(ti)) return;
        const d = distance(spawn.x, spawn.y, target.x, target.y);
        if (d < bestDist) { bestDist = d; bestIdx = ti; }
      });
      assigned.add(bestIdx);
      this.drones.push(this.makeDrone(i, spawn, targets[bestIdx], bestIdx));
    });
  }

  step() {
    this.simTime += this.dt;
    const teamActions = this.dronePolicy?.getTeamActions?.(this) ?? null;
    this.drones.forEach((drone) => this.updateDrone(drone, teamActions?.get(drone.id)));
    this.updateCameras();
    this.updateAnti();
    this.updateExplosions();
  }

  getStats() {
    return {
      drones:      this.drones.filter((d) => d.alive && d.mode !== 'hit' && d.mode !== 'intercepted').length,
      hits:        this.drones.filter((d) => d.mode === 'hit').length,
      intercepted: this.drones.filter((d) => d.mode === 'intercepted').length,
      activeAnti:  this.antidrones.filter((a) => a.alive && a.mode !== 'base').length,
      time:        Math.round(this.simTime),
    };
  }

  getState() {
    return {
      seed:      this.seed,
      time:      this.simTime,
      params:    { ...this.params },
      drones:    this.drones.map(copyAgent),
      antidrones: this.antidrones.map(copyAgent),
      targets:   this.targets.map((t, id) => ({ id, ...t })),
      cameras:   this.cameras.map((c) => ({
        id: c.id, x: c.x, y: c.y, angle: c.angle,
        detectedDroneId: c.detected?.id ?? null,
      })),
    };
  }

  findClosestUnattackedTarget(drone) {
    let best = null, bestDist = Infinity;
    this.targets.forEach((target, i) => {
      if (target.hit) return;
      const d = distance(drone.x, drone.y, target.x, target.y);
      if (d < bestDist) { bestDist = d; best = { t: target, i }; }
    });
    return best;
  }

  markDroneHitTarget(drone) {
    if (this.targets[drone.targetIdx]) this.targets[drone.targetIdx].hit = true;
    this.explosions.push({ x: drone.x, y: drone.y, z: drone.z, t: 1.0 });
    drone.alive = false;
    drone.trail  = [];
    drone.mode   = 'hit';
  }

  markDroneIntercepted(drone) {
    this.explosions.push({ x: drone.x, y: drone.y, z: drone.z, t: 1.0 });
    drone.alive = false;
    drone.mode  = 'intercepted';
    drone.trail = [];
  }

  markGroundCrash(agent) {
    this.explosions.push({ x: agent.x, y: agent.y, z: 0, t: 1.0 });
    agent.alive = false;
    agent.mode  = 'crashed';
    agent.trail = [];
  }

  applyAntiBarometer(anti) {
    // Altitude constraint is enforced on gz *before* physics (in updateActiveAnti/updateCirclingAnti).
    // This post-physics pass only prevents the anti from physically penetrating the ground.
    if (anti.mode === 'base' || anti.mode === 'waiting') return;
    if ((anti.z ?? 0) < GROUND_CRASH_ALT) anti.z = GROUND_CRASH_ALT;
  }

  isAntiSafetyOff(anti) {
    return (anti.mode === 'intercept' || anti.mode === 'chase')
      && anti.target?.alive
      && anti.target.mode !== 'hit'
      && anti.target.mode !== 'intercepted'
      && distance(anti.x, anti.y, anti.target.x, anti.target.y) <= INTERCEPT_R;
  }

  getDeploymentSpawns(droneCount) {
    const policySpawns = this.dronePolicy?.getDeployment?.(this, droneCount);
    if (!Array.isArray(policySpawns) || policySpawns.length < droneCount) {
      return Array.from({ length: droneCount }, () => this.spawnEdge());
    }
    return policySpawns.slice(0, droneCount).map((s) => {
      if (!Number.isFinite(s?.x) || !Number.isFinite(s?.y)) return this.spawnEdge();
      return {
        x: clamp(s.x, DEPLOYMENT_MARGIN, FIELD_SIZE - DEPLOYMENT_MARGIN),
        y: clamp(s.y, DEPLOYMENT_MARGIN, FIELD_SIZE - DEPLOYMENT_MARGIN),
      };
    });
  }

  spawnEdge() {
    const side   = this.rng.int(4);
    const margin = DEPLOYMENT_MARGIN;
    const F      = FIELD_SIZE;
    if (side === 0) return { x: this.rng.range(margin, F - margin), y: margin };
    if (side === 1) return { x: this.rng.range(margin, F - margin), y: F - margin };
    if (side === 2) return { x: margin,          y: this.rng.range(margin, F - margin) };
    return             { x: F - margin,          y: this.rng.range(margin, F - margin) };
  }

  spawnTarget() {
    const angle  = this.rng.range(0, Math.PI * 2);
    const radius = this.rng.range(100, CENTER_ZONE);
    return {
      x: FIELD_SIZE / 2 + Math.cos(angle) * radius,
      y: FIELD_SIZE / 2 + Math.sin(angle) * radius,
      z: 0,
      hit: false,
    };
  }

  // ── Agent factories ──────────────────────────────────────────────────────

  makeDrone(id, spawn, target, targetIdx) {
    const maxSpeed  = this.getParam('dspeed') / 3.6;
    const mass      = this.getParam('dmass');
    const maxThrust = mass * 9.81 * 0.35;
    const angle0    = Math.atan2(target.y - spawn.y, target.x - spawn.x);
    return {
      id,
      x: spawn.x, y: spawn.y, z: CRUISE_ALT,
      vx: Math.cos(angle0) * maxSpeed,
      vy: Math.sin(angle0) * maxSpeed,
      vz: 0,
      angle: angle0,
      tx: target.x,  ty: target.y,  tz: CRUISE_ALT,
      rtx: target.x, rty: target.y, rtz: CRUISE_ALT,
      targetIdx,
      trail: [],
      mode:   'approach',
      intent: 'attack',
      alive:  true,
      col:    DRONE_COLS[id % DRONE_COLS.length],
      evading:       false,
      evadeWpt:      null,
      predictPt:     null,
      threatId:      null,
      evadeCooldown: 0,
      deadZone:      false,
      // physics params (injected into applyPhysics)
      mass,
      maxThrust,
      maxSpeed,
      dragCoef:     mass * 0.0018,
      responseTime: DRONE_RESPONSE_TIME,
      maxVz:        DRONE_MAX_VZ,
      yawRate:      DEFAULT_DRONE_TURN_RATE,
      stallSpeed:     DRONE_STALL_KMH / 3.6,
      minSpeedFactor: 0.65,
      maxForwardAcc:  2.8,
      maxBrakeAcc:    1.6,
      maxLateralAcc:  9.81 * 0.85,
      maxClimbRatio:  0.22,
      maxVzAcc:       9.81 * 0.45,
    };
  }

  makeCamera(id, x, y, angle) {
    return { id, x, y, z: CAMERA_Z, angle, detected: null, cooldown: 0 };
  }

  placeCamerasAroundTargets() {
    const radius = FIELD_SIZE * 0.15;
    return Array.from({ length: CAMERA_COUNT }, (_, i) => {
      const angle = (i / CAMERA_COUNT) * Math.PI * 2;
      const x = FIELD_SIZE / 2 + Math.cos(angle) * radius;
      const y = FIELD_SIZE / 2 + Math.sin(angle) * radius;
      return this.makeCamera(i, x, y, angle);
    });
  }

  makeAnti(id, total) {
    const angle  = total > 1 ? (id / total) * Math.PI * 2 : 0;
    const radius = total > 1 ? 40 : 0;
    const maxSpeed = this.getParam('aspeed') / 3.6;
    const mass     = this.getParam('amass');
    const maxThrust = mass * 9.81 * 3.0;  // thrust-to-weight ~3.0 (fast interceptor)
    return {
      id,
      x: BASE_X + Math.cos(angle) * radius,
      y: BASE_Y + Math.sin(angle) * radius,
      z: 0,
      vx: 0, vy: 0, vz: 0,
      angle: 0,
      target: null,
      lastKnownX: null, lastKnownY: null, lastKnownZ: null,
      lastKnownVx: 0, lastKnownVy: 0, lastKnownVz: 0,
      lostVisualTimer: 0,
      mode:         'base',
      trail:        [],
      alive:        true,
      airborne:     false,
      launchDelay:  0,
      pendingTarget: null,
      battery:      600,
      // physics params
      mass,
      maxThrust,
      maxSpeed,
      dragCoef:     dragCoefForSpeed(maxThrust, maxSpeed, mass),
      responseTime:   ANTI_RESPONSE_TIME,
      maxVz:          ANTI_MAX_VZ,
      yawRate:        DEFAULT_ANTI_TURN_RATE,
      minSpeedFactor: 0.18,   // agile, but does not brake to a near-hover in turns
    };
  }

  // ── Drone update ─────────────────────────────────────────────────────────

  updateDrone(drone, action = null) {
    const resolvedAction = action ?? this.dronePolicy.getAction(this, drone);
    this.applyDroneAction(drone, resolvedAction);
  }

  applyDroneAction(drone, action) {
    if (!action || action.kind === 'idle') return;
    if (action.kind === 'hitTarget') { this.markDroneHitTarget(drone); return; }
    if (action.kind === 'deactivate') { drone.alive = false; drone.trail = []; return; }
    if (action.kind !== 'guidance') return;

    drone.intent  = action.intent  ?? drone.intent  ?? 'attack';
    drone.mode    = action.mode    ?? drone.mode;
    drone.evading = action.intent === 'evade';

    const target = this.targets[drone.targetIdx];
    if (target?.hit && (drone.mode === 'climb' || drone.mode === 'dive')) {
      drone.alive = false;
      drone.trail = [];
      return;
    }

    const droneSafetyOff = drone.mode === 'dive' || action.mode === 'dive';
    const rawTz = action.tz ?? CRUISE_ALT;
    const tz = droneSafetyOff ? rawTz : Math.max(rawTz, MIN_ALT);
    let cruiseSpeed = (action.cruiseKmh ?? this.getParam('dspeed')) / 3.6;

    // Refresh physics params when sim params change
    drone.maxSpeed = (action.cruiseKmh ?? this.getParam('dspeed')) / 3.6;
    drone.mass     = this.getParam('dmass');
    drone.maxThrust = drone.mass * 9.81 * 0.35;
    drone.dragCoef  = drone.mass * 0.0018;
    drone.altitudeDamping = !droneSafetyOff;
    navigateFixedWingToPoint(drone, action.tx, action.ty, tz, cruiseSpeed, this.dt, FIELD_SIZE);
    if (drone.mode === 'dive' && target && !target.hit
        && distance(drone.x, drone.y, target.x, target.y) < ARRIVAL_R
        && (drone.z ?? 0) <= DRONE_TARGET_HIT_ALT) {
      this.markDroneHitTarget(drone);
      return;
    }
    if ((drone.z ?? 0) < GROUND_CRASH_ALT) {
      this.markGroundCrash(drone);
      return;
    }

    drone.trail.push({ x: drone.x, y: drone.y, z: drone.z });
    if (drone.trail.length > 500) drone.trail.shift();
  }

  // ── Camera update ─────────────────────────────────────────────────────────

  updateCameras() {
    const camRange = this.getParam('camrange');
    const delay    = this.getParam('adelay');

    this.cameras.forEach((camera) => {
      if (camera.cooldown > 0) { camera.cooldown -= this.dt; camera.detected = null; return; }

      camera.detected = null;
      let best = null, bestDist = camRange;

      this.drones.forEach((drone) => {
        if (!drone.alive || drone.mode === 'hit' || drone.mode === 'intercepted') return;
        // Ground cameras look upward — use 3D distance with elevation range cap
        if (!inCone3d(camera.x, camera.y, camera.z, camera.angle, CAM_FOV, drone.x, drone.y, drone.z, camRange, CAM_ELEV_R)) return;
        const d = distance3d(camera.x, camera.y, camera.z, drone.x, drone.y, drone.z);
        if (d < bestDist) { bestDist = d; best = drone; }
      });

      if (!best) return;
      camera.detected = best;

      const freeAnti = this.antidrones.find((a) => a.alive && a.mode === 'base' && a.launchDelay === 0);
      if (freeAnti) {
        freeAnti.launchDelay   = delay;
        freeAnti.pendingTarget = best;
        freeAnti.mode          = 'waiting';
        camera.cooldown        = 3;
      }
    });
  }

  // ── Antidrone update ──────────────────────────────────────────────────────

  findNewAntiTarget(anti) {
    const antiRange = this.getParam('arange');
    const pitch = Math.atan2(anti.vz ?? 0, Math.hypot(anti.vx ?? 0, anti.vy ?? 0));
    const inView = this.drones.filter((drone) => {
      return drone.alive && drone.mode !== 'hit' && drone.mode !== 'intercepted'
        && inCone3dFull(anti.x, anti.y, anti.z, anti.angle, pitch, ANTI_FOV, drone.x, drone.y, drone.z, antiRange);
    });
    if (inView.length === 0) return null;
    inView.sort((a, b) =>
      distance3d(anti.x, anti.y, anti.z, a.x, a.y, a.z) -
      distance3d(anti.x, anti.y, anti.z, b.x, b.y, b.z));
    return inView[0];
  }

  updateAnti() {
    const antiRange  = this.getParam('arange');

    this.antidrones.forEach((anti) => {
      if (!anti.alive) return;

      if (anti.mode === 'waiting') {
        anti.launchDelay -= this.dt;
        if (anti.launchDelay <= 0) {
          anti.launchDelay = 0;
          const t = anti.pendingTarget;
          if (t && t.alive && t.mode !== 'hit' && t.mode !== 'intercepted') {
            anti.target      = t;
            anti.lastKnownX  = t.x;
            anti.lastKnownY  = t.y;
            anti.lastKnownZ  = t.z;
            anti.lastKnownVx = t.vx ?? 0;
            anti.lastKnownVy = t.vy ?? 0;
            anti.lastKnownVz = t.vz ?? 0;
            anti.z           = Math.max(GROUND_CRASH_ALT, anti.z ?? 0);
            anti.airborne    = true;
            anti.angle       = Math.atan2(t.y - anti.y, t.x - anti.x);
            anti.mode        = 'intercept';
          } else {
            const nt = this.findNewAntiTarget(anti);
            if (nt) {
              anti.target = nt;
              anti.lastKnownX = nt.x; anti.lastKnownY = nt.y; anti.lastKnownZ = nt.z;
              anti.lastKnownVx = nt.vx ?? 0; anti.lastKnownVy = nt.vy ?? 0; anti.lastKnownVz = nt.vz ?? 0;
              anti.z = Math.max(GROUND_CRASH_ALT, anti.z ?? 0);
              anti.airborne = true;
              anti.angle = Math.atan2(nt.y - anti.y, nt.x - anti.x);
              anti.mode  = 'intercept';
            } else { anti.mode = 'base'; }
          }
          anti.pendingTarget = null;
        }
        return;
      }

      if (anti.mode === 'base') return;

      // Refresh physics when sim params change
      const newMaxSpeed = this.getParam('aspeed') / 3.6;
      if (Math.abs(anti.maxSpeed - newMaxSpeed) > 0.1) {
        anti.maxSpeed  = newMaxSpeed;
        anti.mass      = this.getParam('amass');
        anti.maxThrust = anti.mass * 9.81 * 3.0;
        anti.dragCoef  = dragCoefForSpeed(anti.maxThrust, anti.maxSpeed, anti.mass);
      }

      anti.battery -= this.dt;
      if (anti.battery <= 0) { anti.alive = false; return; }

      // Camera always active — scan for drones in FOV (cone tilts with antidrone pitch)
      if (anti.mode !== 'base' && anti.mode !== 'waiting') {
        const pitch = Math.atan2(anti.vz ?? 0, Math.hypot(anti.vx ?? 0, anti.vy ?? 0));
        const inView = this.drones.filter((drone) => {
          return drone.alive && drone.mode !== 'hit' && drone.mode !== 'intercepted'
            && inCone3dFull(anti.x, anti.y, anti.z, anti.angle, pitch, ANTI_FOV, drone.x, drone.y, drone.z, antiRange);
        });
        if (inView.length > 0) {
          inView.sort((a, b) =>
            distance3d(anti.x, anti.y, anti.z, a.x, a.y, a.z) -
            distance3d(anti.x, anti.y, anti.z, b.x, b.y, b.z));
          const closest = inView[0];
          if (anti.target !== closest) {
            anti.target     = closest;
            anti.lastKnownX = closest.x;
            anti.lastKnownY = closest.y;
            anti.lastKnownZ = closest.z;
          }
          anti.lastKnownVx = closest.vx ?? 0;
          anti.lastKnownVy = closest.vy ?? 0;
          anti.lastKnownVz = closest.vz ?? 0;
          anti.lostVisualTimer = 0;
          anti.mode = 'chase';
        }
      }

      if (anti.mode === 'intercept' || anti.mode === 'chase' || anti.mode === 'lastknown') {
        this.updateActiveAnti(anti, antiRange);
      } else if (anti.mode === 'circle' || anti.mode === 'recoverSearch') {
        this.updateCirclingAnti(anti);
      }

      this.applyAntiBarometer(anti);
      if ((anti.z ?? 0) >= GROUND_CRASH_ALT) anti.airborne = true;
      if (anti.airborne && (anti.z ?? 0) < GROUND_CRASH_ALT) {
        this.markGroundCrash(anti);
        return;
      }

      anti.trail.push({ x: anti.x, y: anti.y, z: anti.z });
      if (anti.trail.length > 500) anti.trail.shift();
    });
  }

  updateActiveAnti(anti, antiRange) {
    if (anti.mode !== 'lastknown') {
      if (!anti.target || !anti.target.alive || anti.target.mode === 'hit' || anti.target.mode === 'intercepted') {
        const nt = this.findNewAntiTarget(anti);
        if (nt) {
          anti.target = nt;
          anti.lastKnownX = nt.x; anti.lastKnownY = nt.y; anti.lastKnownZ = nt.z;
          anti.lastKnownVx = nt.vx ?? 0; anti.lastKnownVy = nt.vy ?? 0; anti.lastKnownVz = nt.vz ?? 0;
          anti.mode = 'intercept';
        } else { anti.mode = 'recoverSearch'; anti.target = null; return; }
      } else if (inCone3dFull(anti.x, anti.y, anti.z, anti.angle,
                   Math.atan2(anti.vz ?? 0, Math.hypot(anti.vx ?? 0, anti.vy ?? 0)),
                   ANTI_FOV, anti.target.x, anti.target.y, anti.target.z, antiRange)) {
        anti.lastKnownX = anti.target.x;
        anti.lastKnownY = anti.target.y;
        anti.lastKnownZ = anti.target.z;
        anti.lastKnownVx = anti.target.vx ?? 0;
        anti.lastKnownVy = anti.target.vy ?? 0;
        anti.lastKnownVz = anti.target.vz ?? 0;
        anti.lostVisualTimer = 0;
        anti.mode = 'chase';
      } else if (anti.mode === 'chase') {
        anti.lostVisualTimer = (anti.lostVisualTimer ?? 0) + this.dt;
        if (anti.lostVisualTimer < ANTI_LOST_VISUAL_GRACE) {
          anti.mode = 'chase';
        } else {
          anti.mode = 'recoverSearch';
          return;
        }
      } else {
        anti.mode = 'lastknown';
      }
    }

    // Destination — lead pursuit in chase mode, raw last-known otherwise
    let gx, gy, gz;
    const safetyOff = this.isAntiSafetyOff(anti);
    if (anti.mode === 'chase' && anti.target?.alive && (anti.lostVisualTimer ?? 0) <= 0) {
      const aim = computeAntiAimPoint(anti, anti.target);
      gx = clamp(aim.x, 0, FIELD_SIZE);
      gy = clamp(aim.y, 0, FIELD_SIZE);
      const minGuidanceAlt = anti.mode === 'lastknown' ? ANTI_SEARCH_ALT : ANTI_BARO_FLOOR;
      gz = clamp(aim.z, safetyOff ? 0 : minGuidanceAlt, MAX_ALT);
    } else if (anti.lastKnownX !== null) {
      const tLost = Math.min(ANTI_LOST_VISUAL_GRACE, anti.lostVisualTimer ?? 0);
      gx = anti.lastKnownX + (anti.lastKnownVx ?? 0) * tLost;
      gy = anti.lastKnownY + (anti.lastKnownVy ?? 0) * tLost;
      gz = (anti.lastKnownZ ?? CRUISE_ALT) + (anti.lastKnownVz ?? 0) * tLost;
    } else { anti.mode = 'recoverSearch'; return; }

    const minGuidanceAlt = anti.mode === 'lastknown' ? ANTI_SEARCH_ALT : ANTI_BARO_FLOOR;
    gz = clamp(gz, safetyOff ? 0 : minGuidanceAlt, MAX_ALT);

    // 3D intercept check
    if (anti.target?.alive && anti.target.mode !== 'hit' && anti.target.mode !== 'intercepted') {
      if (distance3d(anti.x, anti.y, anti.z, anti.target.x, anti.target.y, anti.target.z) < INTERCEPT_R) {
        this.markDroneIntercepted(anti.target);
        anti.alive = false;
        return;
      }
    }

    // Arrived at last-known point with no new target → circle
    if (anti.mode === 'lastknown') {
      const dToKnown = distance3d(anti.x, anti.y, anti.z, gx, gy, gz);
      if (dToKnown < 30) {
        const nt = this.findNewAntiTarget(anti);
        if (nt) {
          anti.target = nt;
          anti.lastKnownX = nt.x; anti.lastKnownY = nt.y; anti.lastKnownZ = nt.z;
          anti.lastKnownVx = nt.vx ?? 0; anti.lastKnownVy = nt.vy ?? 0; anti.lastKnownVz = nt.vz ?? 0;
          anti.mode = 'intercept';
        } else { anti.mode = 'recoverSearch'; anti.target = null; return; }
      }
    }

    navigateToPoint(anti, gx, gy, gz, anti.maxSpeed, this.dt, FIELD_SIZE);
    if (anti.target?.alive && anti.target.mode !== 'hit' && anti.target.mode !== 'intercepted') {
      if (distance3d(anti.x, anti.y, anti.z, anti.target.x, anti.target.y, anti.target.z) < INTERCEPT_R) {
        this.markDroneIntercepted(anti.target);
        anti.alive = false;
      }
    }
  }

  updateCirclingAnti(anti) {
    if (anti.z < ANTI_SEARCH_ALT - 3) anti.mode = 'recoverSearch';
    else anti.mode = 'circle';

    // Circle at constant yaw rate (no comms, no base return)
    const circleRate = 15 * Math.PI / 180; // rad/s
    anti.angle += circleRate * this.dt;
    const climbingToSearchAlt = anti.z < ANTI_SEARCH_ALT - 3;
    const circleSpeed = climbingToSearchAlt ? anti.maxSpeed * 0.25 : anti.maxSpeed;
    const targetVx = Math.cos(anti.angle) * circleSpeed;
    const targetVy = Math.sin(anti.angle) * circleSpeed;
    const targetVz = climbingToSearchAlt
      ? clamp((ANTI_SEARCH_ALT - anti.z) * 2.0, 0, anti.maxVz)
      : clamp((ANTI_SEARCH_ALT - anti.z) * 1.5 - (anti.vz ?? 0) * 1.2, -anti.maxVz, anti.maxVz);
    applyPhysics(anti, targetVx, targetVy, targetVz, this.dt);
    anti.x = clamp(anti.x, 0, FIELD_SIZE);
    anti.y = clamp(anti.y, 0, FIELD_SIZE);
  }

  updateExplosions() {
    for (let i = this.explosions.length - 1; i >= 0; i--) {
      this.explosions[i].t -= this.dt * 1.5;
      if (this.explosions[i].t <= 0) this.explosions.splice(i, 1);
    }
  }
}

function computeAntiAimPoint(anti, target) {
  const rx = target.x - anti.x;
  const ry = target.y - anti.y;
  const rz = (target.z ?? CRUISE_ALT) - (anti.z ?? 0);
  const vx = target.vx ?? 0;
  const vy = target.vy ?? 0;
  const vz = target.vz ?? 0;
  const dist3 = Math.hypot(rx, ry, rz);
  const speed = Math.max(1, anti.maxSpeed);

  const vv = vx * vx + vy * vy + vz * vz;
  const rv = rx * vx + ry * vy + rz * vz;
  const rr = rx * rx + ry * ry + rz * rz;
  const a = vv - speed * speed;
  const b = 2 * rv;
  const c = rr;
  let tLead = dist3 / speed;
  if (Math.abs(a) > 1e-6) {
    const disc = b * b - 4 * a * c;
    if (disc >= 0) {
      const sqrtDisc = Math.sqrt(disc);
      const t1 = (-b - sqrtDisc) / (2 * a);
      const t2 = (-b + sqrtDisc) / (2 * a);
      const candidates = [t1, t2].filter((t) => t > 0);
      if (candidates.length) tLead = Math.min(...candidates);
    }
  } else if (Math.abs(b) > 1e-6) {
    const t = -c / b;
    if (t > 0) tLead = t;
  }
  tLead = clamp(tLead, 0, 2.5);
  return {
    x: target.x + vx * tLead,
    y: target.y + vy * tLead,
    z: (target.z ?? CRUISE_ALT) + vz * tLead,
  };
}

function copyAgent(agent) {
  return {
    id:       agent.id,
    x:        agent.x,
    y:        agent.y,
    z:        agent.z,
    vx:       agent.vx,
    vy:       agent.vy,
    vz:       agent.vz,
    angle:    agent.angle,
    mode:     agent.mode,
    intent:   agent.intent ?? null,
    alive:    agent.alive,
    targetId: agent.target?.id ?? agent.targetIdx ?? null,
  };
}
