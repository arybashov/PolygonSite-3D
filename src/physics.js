// 3D aerodynamics for multicopter drones
// Based on DS_3_drone_school_3.html physics model
//
// Model: first-order velocity tracking with thrust saturation + quadratic drag
//   responseTime — motor/aerodynamic inertia time constant
//   maxThrust    — caps horizontal + vertical acceleration
//   dragCoef     — quadratic air drag  (use dragCoefForSpeed() to compute from cruise speed)
//
// Gravity is compensated by collective throttle (hover assumption).
// Vertical axis is decoupled from horizontal for simpler control authority split.

import { angleDiff, clamp } from './math.js';

export const GRAVITY = 9.81;  // m/s²

// dragCoef so that at maxSpeed: drag = horizontal thrust (0.80 * maxThrust) → equilibrium
// Without the 0.80 factor, cruise speed would settle at sqrt(0.80) * maxSpeed ≈ 89% of spec.
export function dragCoefForSpeed(maxThrust, maxSpeed) {
  return maxSpeed > 0 ? 0.80 * maxThrust / (maxSpeed * maxSpeed) : 0;
}

// Core physics step. Modifies agent in place.
// targetVx/Vy: desired horizontal velocity (m/s)
// targetVz:    desired vertical velocity   (m/s)
// agent needs: vx, vy, vz, x, y, z, maxSpeed, maxVz, responseTime, mass, maxThrust, dragCoef
export function applyPhysics(agent, targetVx, targetVy, targetVz, dt) {
  const { responseTime, maxSpeed, maxVz, mass, maxThrust, dragCoef } = agent;

  // --- Horizontal ---
  const vH     = Math.hypot(agent.vx, agent.vy);
  const dragAcc = dragCoef * vH * vH / mass;
  const dragAx  = vH > 0.01 ? -agent.vx / vH * dragAcc : 0;
  const dragAy  = vH > 0.01 ? -agent.vy / vH * dragAcc : 0;

  // P-controller on velocity error, thrust-limited
  const maxAccH = maxThrust * 0.80 / mass;
  let ax = (targetVx - agent.vx) / responseTime + dragAx;
  let ay = (targetVy - agent.vy) / responseTime + dragAy;
  const aMagH = Math.hypot(ax, ay);
  if (aMagH > maxAccH) { ax = ax / aMagH * maxAccH; ay = ay / aMagH * maxAccH; }

  agent.vx += ax * dt;
  agent.vy += ay * dt;

  // Hard speed cap
  const vHNew = Math.hypot(agent.vx, agent.vy);
  if (vHNew > maxSpeed) { const s = maxSpeed / vHNew; agent.vx *= s; agent.vy *= s; }

  // --- Vertical (decoupled) ---
  const tvz    = clamp(targetVz, -maxVz, maxVz);
  const maxAccV = maxThrust * 0.20 / mass;
  const errVz  = tvz - agent.vz;
  agent.vz += clamp(errVz / (responseTime * 0.4), -maxAccV, maxAccV) * dt;
  agent.vz  = clamp(agent.vz, -maxVz, maxVz);

  // --- Position ---
  agent.x += agent.vx * dt;
  agent.y += agent.vy * dt;
  agent.z  = Math.max(0, agent.z + agent.vz * dt);
}

// Navigate toward a 3D waypoint: update agent.angle (yaw) then call applyPhysics.
// Returns horizontal distance to target.
export function navigateToPoint(agent, tx, ty, tz, speed, dt, fieldSize = 0) {
  const dx    = tx - agent.x;
  const dy    = ty - agent.y;
  const distH = Math.hypot(dx, dy);

  // Yaw toward horizontal target
  const desiredYaw = Math.atan2(dy, dx);
  const yawDiff    = angleDiff(agent.angle, desiredYaw);
  const maxYaw     = agent.yawRate * dt;
  agent.angle += clamp(yawDiff, -maxYaw, maxYaw);

  // Speed: reduce when turning or near stop-distance
  const alignFactor  = Math.max(0.25, Math.cos(yawDiff));
  const stopDist     = speed * agent.responseTime * 2.5;
  const speedFactor  = Math.min(1, distH / (stopDist + 1));
  const desiredSpeed = speed * alignFactor * speedFactor;

  const targetVx = distH > 0.1 ? (dx / distH) * desiredSpeed : 0;
  const targetVy = distH > 0.1 ? (dy / distH) * desiredSpeed : 0;

  // Altitude proportional control
  const targetVz = clamp((tz - agent.z) * 3.0, -agent.maxVz, agent.maxVz);

  applyPhysics(agent, targetVx, targetVy, targetVz, dt);

  if (fieldSize > 0) {
    agent.x = clamp(agent.x, 0, fieldSize);
    agent.y = clamp(agent.y, 0, fieldSize);
  }

  return distH;
}
