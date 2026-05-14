// 3D aerodynamics for multicopter interceptors and fixed-wing drones
// Based on DS_3_drone_school_3.html physics model
//
// Model: first-order velocity tracking with shared thrust saturation + quadratic drag
//   responseTime — motor/aerodynamic inertia time constant
//   maxThrust    — total thrust cap; hover consumes mass * gravity
//   dragCoef     — quadratic air drag  (use dragCoefForSpeed() to compute from cruise speed)
//
// Gravity is explicit. Horizontal and vertical control share the same thrust vector:
// aggressive climb/descent reduces horizontal authority and vice versa.

import { angleDiff, clamp } from './math.js';

export const GRAVITY = 9.81;  // m/s²

// dragCoef so that at maxSpeed in level flight: drag = horizontal thrust available
// while still reserving gravity for hover.
export function dragCoefForSpeed(maxThrust, maxSpeed, mass = null) {
  if (maxSpeed <= 0) return 0;
  const maxLevelForce = mass
    ? hoverHorizontalAccel(maxThrust, mass) * mass
    : 0.80 * maxThrust;
  return maxLevelForce / (maxSpeed * maxSpeed);
}

// Core physics step. Modifies agent in place.
// targetVx/Vy: desired horizontal velocity (m/s)
// targetVz:    desired vertical velocity   (m/s)
// agent needs: vx, vy, vz, x, y, z, maxSpeed, maxVz, responseTime, mass, maxThrust, dragCoef
export function applyPhysics(agent, targetVx, targetVy, targetVz, dt) {
  const { responseTime, maxSpeed, maxVz, mass, maxThrust, dragCoef } = agent;
  const thrustLimit = maxThrust / mass;

  // --- Horizontal ---
  const vH     = Math.hypot(agent.vx, agent.vy);
  const dragAcc = dragCoef * vH * vH / mass;
  const dragAx  = vH > 0.01 ? -agent.vx / vH * dragAcc : 0;
  const dragAy  = vH > 0.01 ? -agent.vy / vH * dragAcc : 0;

  // --- Shared thrust vector ---
  // Controllers request a net acceleration. We convert that to required thrust,
  // add gravity compensation on Z, then clamp the total thrust vector.
  const tvz = clamp(targetVz, -maxVz, maxVz);
  const desiredAx = (targetVx - agent.vx) / responseTime;
  const desiredAy = (targetVy - agent.vy) / responseTime;
  const desiredAz = (tvz - agent.vz) / (responseTime * 0.4);

  let thrustAx = desiredAx - dragAx;
  let thrustAy = desiredAy - dragAy;
  let thrustAz = desiredAz + GRAVITY;
  if (thrustAz < 0) thrustAz = 0;

  const thrustMag = Math.hypot(thrustAx, thrustAy, thrustAz);
  if (thrustMag > thrustLimit) {
    const s = thrustLimit / thrustMag;
    thrustAx *= s; thrustAy *= s; thrustAz *= s;
  }

  const ax = thrustAx + dragAx;
  const ay = thrustAy + dragAy;
  const az = thrustAz - GRAVITY;

  agent.vx += ax * dt;
  agent.vy += ay * dt;
  agent.vz += az * dt;
  agent.vz  = clamp(agent.vz, -maxVz, maxVz);

  // Hard speed cap
  const vHNew = Math.hypot(agent.vx, agent.vy);
  if (vHNew > maxSpeed) { const s = maxSpeed / vHNew; agent.vx *= s; agent.vy *= s; }

  // --- Position ---
  agent.x += agent.vx * dt;
  agent.y += agent.vy * dt;
  agent.z  = Math.max(0, agent.z + agent.vz * dt);
}

// Navigate toward a 3D waypoint: update agent.angle (yaw) then call applyPhysics.
// Returns horizontal distance to target.
//
// Turn model: effective yaw rate = min(agent.yawRate, maxAccH / currentSpeed)
//   — at low speed: actuator-limited (can spin nearly in place if no stallSpeed)
//   — at cruise:    centripetal-limited, gives realistic turn radius v²/maxAccH
// Thrust is always in the yaw direction (nose-pointed thrust).
export function navigateToPoint(agent, tx, ty, tz, speed, dt, fieldSize = 0) {
  const dx    = tx - agent.x;
  const dy    = ty - agent.y;
  const distH = Math.hypot(dx, dy);

  // Velocity-dependent yaw rate: min(actuator limit, centripetal physics limit)
  const currentSpeed = Math.hypot(agent.vx, agent.vy);
  const maxAccH      = hoverHorizontalAccel(agent.maxThrust, agent.mass);
  const physicsYawRate = currentSpeed > 0.5 ? maxAccH / currentSpeed : agent.yawRate;
  const effectiveYawRate = Math.min(agent.yawRate, physicsYawRate);

  const desiredYaw = Math.atan2(dy, dx);
  const yawDiff    = angleDiff(agent.angle, desiredYaw);
  agent.angle += clamp(yawDiff, -effectiveYawRate * dt, effectiveYawRate * dt);

  // Speed: reduce when misaligned, never below stall speed.
  // (1+cos)/2 = cos²(yaw/2): smooth, aggressive near 90-180° — forces tight turns.
  // minSpeedFactor is mostly used by the antidrone; the fixed-wing drone has its own navigator.
  const stallSpeed      = agent.stallSpeed ?? 0;
  const minSpeedFactor  = agent.minSpeedFactor ?? 0.30;
  const alignFactor     = Math.max(minSpeedFactor, (1 + Math.cos(yawDiff)) / 2);
  const stopDist        = speed * agent.responseTime * 2.5;
  const speedFactor     = Math.min(1, distH / (stopDist + 1));
  const desiredSpeed    = Math.max(stallSpeed, speed * alignFactor * speedFactor);

  // Thrust along yaw direction (nose-pointed), not raw target vector
  const targetVx = Math.cos(agent.angle) * desiredSpeed;
  const targetVy = Math.sin(agent.angle) * desiredSpeed;

  // Altitude control. Damping on current vz prevents overshoot at altitude floors.
  // Final attack dive can disable the damping to keep a hard descent profile.
  const altErr = tz - agent.z;
  const damping = agent.altitudeDamping === false ? 0 : 1.2;
  const targetVz = clamp(altErr * 1.5 - agent.vz * damping, -agent.maxVz, agent.maxVz);

  applyPhysics(agent, targetVx, targetVy, targetVz, dt);

  if (fieldSize > 0) {
    agent.x = clamp(agent.x, 0, fieldSize);
    agent.y = clamp(agent.y, 0, fieldSize);
  }

  return distH;
}

// Fixed-wing navigation for the strike drone.
// It has no hover mode: thrust is forward-only, altitude is changed by flight path,
// and commanded speed never drops below stall speed.
export function navigateFixedWingToPoint(agent, tx, ty, tz, speed, dt, fieldSize = 0) {
  const dx = tx - agent.x;
  const dy = ty - agent.y;
  const distH = Math.hypot(dx, dy);

  const currentSpeed = Math.max(0.1, Math.hypot(agent.vx, agent.vy));
  const maxLateralAcc = agent.maxLateralAcc ?? GRAVITY * 0.85;
  const physicsYawRate = maxLateralAcc / currentSpeed;
  const effectiveYawRate = Math.min(agent.yawRate, physicsYawRate);

  const desiredYaw = Math.atan2(dy, dx);
  const yawDiff = angleDiff(agent.angle, desiredYaw);
  agent.angle += clamp(yawDiff, -effectiveYawRate * dt, effectiveYawRate * dt);

  const stallSpeed = agent.stallSpeed ?? 0;
  const minSpeedFactor = agent.minSpeedFactor ?? 0.65;
  const alignFactor = Math.max(minSpeedFactor, (1 + Math.cos(yawDiff)) / 2);
  const desiredSpeed = Math.max(stallSpeed, speed * alignFactor);

  const dragAcc = agent.dragCoef * currentSpeed * currentSpeed / agent.mass;
  const forwardErr = desiredSpeed - currentSpeed;
  const thrustAcc = clamp(forwardErr / agent.responseTime, -agent.maxBrakeAcc, agent.maxForwardAcc);
  let nextSpeed = currentSpeed + (thrustAcc - dragAcc) * dt;
  nextSpeed = clamp(nextSpeed, 0, agent.maxSpeed);

  const altErr = tz - agent.z;
  const rawTargetVz = altErr * 0.75 - (agent.vz ?? 0) * 0.9;
  const climbReserve = Math.max(0, nextSpeed - stallSpeed);
  const maxClimbVz = Math.min(agent.maxVz, climbReserve * (agent.maxClimbRatio ?? 0.22));
  let targetVz = clamp(rawTargetVz, -agent.maxVz, maxClimbVz);

  if (nextSpeed < stallSpeed) {
    targetVz -= (stallSpeed - nextSpeed) * 0.8;
  }
  if (agent.altitudeDamping === false) {
    targetVz = clamp(altErr * 1.2, -agent.maxVz, maxClimbVz);
  }

  const vzErr = targetVz - (agent.vz ?? 0);
  const maxVzAcc = agent.maxVzAcc ?? GRAVITY * 0.45;
  agent.vz = clamp((agent.vz ?? 0) + clamp(vzErr / agent.responseTime, -maxVzAcc, maxVzAcc) * dt, -agent.maxVz, agent.maxVz);

  agent.vx = Math.cos(agent.angle) * nextSpeed;
  agent.vy = Math.sin(agent.angle) * nextSpeed;

  agent.x += agent.vx * dt;
  agent.y += agent.vy * dt;
  agent.z = Math.max(0, agent.z + agent.vz * dt);

  if (fieldSize > 0) {
    agent.x = clamp(agent.x, 0, fieldSize);
    agent.y = clamp(agent.y, 0, fieldSize);
  }

  return distH;
}

function hoverHorizontalAccel(maxThrust, mass) {
  const thrustAcc = maxThrust / mass;
  return Math.sqrt(Math.max(0, thrustAcc * thrustAcc - GRAVITY * GRAVITY));
}
