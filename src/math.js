export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function angleDiff(a, b) {
  let d = b - a;
  while (d > Math.PI)  d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return d;
}

export function distance(ax, ay, bx, by) {
  const dx = ax - bx;
  const dy = ay - by;
  return Math.sqrt(dx * dx + dy * dy);
}

// 3D Euclidean distance
export function distance3d(ax, ay, az, bx, by, bz) {
  const dx = ax - bx;
  const dy = ay - by;
  const dz = az - bz;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

// 2D horizontal cone check (ignores z)
export function inCone(ox, oy, angle, fov, tx, ty, range) {
  const dx = tx - ox;
  const dy = ty - oy;
  const d = Math.sqrt(dx * dx + dy * dy);
  return d <= range && Math.abs(angleDiff(angle, Math.atan2(dy, dx))) <= fov / 2;
}

// 3D cone: uses 3D Euclidean distance for range, horizontal angle for bearing
// elevRange: optional max altitude difference (m); use Infinity to ignore
export function inCone3d(ox, oy, oz, angle, fov, tx, ty, tz, range, elevRange = Infinity) {
  const dist = distance3d(ox, oy, oz, tx, ty, tz);
  if (dist > range) return false;
  if (Math.abs(tz - oz) > elevRange) return false;
  const dx = tx - ox;
  const dy = ty - oy;
  return Math.abs(angleDiff(angle, Math.atan2(dy, dx))) <= fov / 2;
}

// True 3D cone with azimuth (yaw) + pitch (elevation tilt).
// Cone axis = unit vector defined by azimuth and pitch; fov is full cone angle.
export function inCone3dFull(ox, oy, oz, azimuth, pitch, fov, tx, ty, tz, range) {
  const dx = tx - ox, dy = ty - oy, dz = tz - oz;
  const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
  if (dist > range) return false;
  const cosP = Math.cos(pitch);
  const ax = Math.cos(azimuth) * cosP;
  const ay = Math.sin(azimuth) * cosP;
  const az = Math.sin(pitch);
  return (ax * dx + ay * dy + az * dz) / dist >= Math.cos(fov / 2);
}

export function createRng(seed = Date.now()) {
  let state = seed >>> 0;
  if (state === 0) state = 0x9e3779b9;

  return {
    next() {
      state ^= state << 13;
      state ^= state >>> 17;
      state ^= state << 5;
      return ((state >>> 0) / 0x100000000);
    },
    range(min, max) {
      return min + this.next() * (max - min);
    },
    int(maxExclusive) {
      return Math.floor(this.next() * maxExclusive);
    },
  };
}
