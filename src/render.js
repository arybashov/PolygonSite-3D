import {
  ANTI_FOV,
  CAM_FOV,
  CENTER_ZONE,
  CRUISE_ALT,
  DRONE_DETECT_R,
  DRONE_FOV,
  FIELD_SIZE,
  MAX_ALT,
  MIN_ALT,
} from './constants.js';

// Color based on altitude: low = green-dim, high = bright cyan
function altColor(z, alpha = 1.0) {
  const t = Math.min(1, (z ?? CRUISE_ALT) / MAX_ALT);
  const r = Math.round(lerp(78,  62,  t));
  const g = Math.round(lerp(203, 210, t));
  const b = Math.round(lerp(113, 232, t));
  return `rgba(${r},${g},${b},${alpha})`;
}

function lerp(a, b, t) { return a + (b - a) * t; }

const ELEVATION_VIEW_MAX_ALT = 120;

export class CanvasRenderer {
  constructor(canvas, sim) {
    this.canvas = canvas;
    this.ctx    = canvas.getContext('2d');
    this.sim    = sim;
    this.width  = 0;
    this.height = 0;
    this.camOffX = 0;
    this.camOffY = 0;
    this.camUserZoom = 1.0;
    this.userPan   = false;
    this.panStart  = { x: 0, y: 0 };
    this.camStart  = { x: 0, y: 0 };

    this.resize();
    this.bindInput();
    window.addEventListener('resize', () => this.resize());
  }

  resetView() { this.camOffX = 0; this.camOffY = 0; this.camUserZoom = 1.0; }

  resize() {
    this.width   = this.canvas.offsetWidth  || 600;
    this.height  = this.canvas.offsetHeight || 600;
    this.canvas.width  = this.width;
    this.canvas.height = this.height;
  }

  baseZoom() { return this.width / FIELD_SIZE * 0.95; }
  getZoom()  { return this.baseZoom() * this.camUserZoom; }

  w2s(wx, wy) {
    const zoom = this.getZoom();
    const cx = FIELD_SIZE / 2 + this.camOffX;
    const cy = FIELD_SIZE / 2 + this.camOffY;
    return { x: (wx - cx) * zoom + this.width / 2, y: (wy - cy) * zoom + this.height / 2 };
  }

  s2w(sx, sy) {
    const zoom = this.getZoom();
    const cx = FIELD_SIZE / 2 + this.camOffX;
    const cy = FIELD_SIZE / 2 + this.camOffY;
    return { x: (sx - this.width / 2) / zoom + cx, y: (sy - this.height / 2) / zoom + cy };
  }

  bindInput() {
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.12 : 0.9;
      const rect   = this.canvas.getBoundingClientRect();
      const mx     = (e.clientX - rect.left) * (this.width  / rect.width);
      const my     = (e.clientY - rect.top)  * (this.height / rect.height);
      const before = this.s2w(mx, my);
      this.camUserZoom = Math.max(0.3, Math.min(8, this.camUserZoom * factor));
      const after  = this.s2w(mx, my);
      this.camOffX -= after.x - before.x;
      this.camOffY -= after.y - before.y;
    }, { passive: false });

    this.canvas.addEventListener('mousedown', (e) => {
      if (e.button !== 2) return;
      this.userPan  = true;
      this.panStart = { x: e.clientX, y: e.clientY };
      this.camStart = { x: this.camOffX, y: this.camOffY };
    });

    this.canvas.addEventListener('mousemove', (e) => {
      if (!this.userPan) return;
      const zoom = this.getZoom();
      this.camOffX = this.camStart.x - (e.clientX - this.panStart.x) / zoom;
      this.camOffY = this.camStart.y - (e.clientY - this.panStart.y) / zoom;
    });

    this.canvas.addEventListener('mouseup',    () => { this.userPan = false; });
    this.canvas.addEventListener('mouseleave', () => { this.userPan = false; });
    this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  draw() {
    const ctx  = this.ctx;
    const zoom = this.getZoom();

    ctx.clearRect(0, 0, this.width, this.height);
    ctx.fillStyle = '#0d0f0e';
    ctx.fillRect(0, 0, this.width, this.height);

    this.drawMap(zoom);
    this.drawCameras(zoom);
    this.drawTrails();
    this.drawTargets(zoom);
    this.drawDrones(zoom);
    this.drawWaitingAntidrones();
    this.drawActiveAntidrones(zoom);
    this.drawExplosions(zoom);

    const p0 = this.w2s(0, 0);
    const p1 = this.w2s(FIELD_SIZE, FIELD_SIZE);
    ctx.fillStyle = 'rgba(78,203,113,0.2)';
    ctx.font = '10px "Share Tech Mono"';
    ctx.fillText('3000m × 3000m  (altitude shown in m)', p0.x + 6, p1.y - 6);
  }

  drawMap(zoom) {
    const ctx = this.ctx;
    const p0  = this.w2s(0, 0);
    const p1  = this.w2s(FIELD_SIZE, FIELD_SIZE);

    ctx.strokeStyle = 'rgba(78,203,113,0.15)';
    ctx.lineWidth   = 1.5;
    ctx.strokeRect(p0.x, p0.y, p1.x - p0.x, p1.y - p0.y);
    ctx.fillStyle   = 'rgba(78,203,113,0.015)';
    ctx.fillRect(p0.x, p0.y, p1.x - p0.x, p1.y - p0.y);

    ctx.strokeStyle = 'rgba(78,203,113,0.06)';
    ctx.lineWidth = 1;
    for (let g = 0; g <= FIELD_SIZE; g += 500) {
      const sx = this.w2s(g, 0);
      const sy = this.w2s(0, g);
      ctx.beginPath(); ctx.moveTo(sx.x, p0.y); ctx.lineTo(sx.x, p1.y); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(p0.x, sy.y); ctx.lineTo(p1.x, sy.y); ctx.stroke();
    }

    const center = this.w2s(FIELD_SIZE / 2, FIELD_SIZE / 2);
    ctx.beginPath();
    ctx.arc(center.x, center.y, CENTER_ZONE * zoom, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(224,75,74,0.15)';
    ctx.setLineDash([5, 4]); ctx.lineWidth = 1; ctx.stroke(); ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(center.x, center.y, 7, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(224,75,74,0.6)'; ctx.fill();
    ctx.fillStyle = 'rgba(78,203,113,0.4)';
    ctx.font = '10px "Share Tech Mono"';
    ctx.fillText('BASE', center.x + 10, center.y + 4);
  }

  drawCameras(zoom) {
    const ctx      = this.ctx;
    const camRange = this.sim.getParam('camrange');
    this.sim.cameras.forEach((camera) => {
      const cs     = this.w2s(camera.x, camera.y);
      const active = camera.detected !== null;
      ctx.beginPath();
      ctx.moveTo(cs.x, cs.y);
      ctx.arc(cs.x, cs.y, camRange * zoom, camera.angle - CAM_FOV / 2, camera.angle + CAM_FOV / 2);
      ctx.closePath();
      ctx.fillStyle   = active ? 'rgba(232,168,48,0.08)' : 'rgba(91,163,232,0.04)';
      ctx.fill();
      ctx.strokeStyle = active ? 'rgba(232,168,48,0.55)' : 'rgba(91,163,232,0.18)';
      ctx.lineWidth   = active ? 1.5 : 0.8;
      ctx.stroke();
      ctx.fillStyle   = active ? '#e8a830' : 'rgba(91,163,232,0.5)';
      ctx.beginPath(); ctx.arc(cs.x, cs.y, Math.max(2, 3 * zoom), 0, Math.PI * 2); ctx.fill();
    });
  }

  drawTrails() {
    const ctx = this.ctx;
    this.sim.drones.forEach((drone) => {
      if (!drone.alive || drone.trail.length < 2) return;
      for (let i = 1; i < drone.trail.length; i++) {
        const s0 = this.w2s(drone.trail[i - 1].x, drone.trail[i - 1].y);
        const s1 = this.w2s(drone.trail[i].x, drone.trail[i].y);
        const alpha = i / drone.trail.length;
        const z = drone.trail[i].z ?? CRUISE_ALT;
        const col = altColor(z, alpha * 0.55);
        ctx.beginPath(); ctx.moveTo(s0.x, s0.y); ctx.lineTo(s1.x, s1.y);
        ctx.strokeStyle = col; ctx.lineWidth = alpha * 2.5; ctx.stroke();
      }
    });

    this.sim.antidrones.forEach((anti) => {
      if (!anti.alive || anti.trail.length < 2) return;
      for (let i = 1; i < anti.trail.length; i++) {
        const s0 = this.w2s(anti.trail[i - 1].x, anti.trail[i - 1].y);
        const s1 = this.w2s(anti.trail[i].x, anti.trail[i].y);
        const alpha = i / anti.trail.length;
        ctx.beginPath(); ctx.moveTo(s0.x, s0.y); ctx.lineTo(s1.x, s1.y);
        ctx.strokeStyle = `rgba(224,75,74,${alpha * 0.55})`; ctx.lineWidth = alpha * 2.5; ctx.stroke();
      }
    });
  }

  drawTargets(zoom) {
    const ctx = this.ctx;
    this.sim.targets.forEach((target, i) => {
      if (target.hit) return;
      const ts    = this.w2s(target.x, target.y);
      const pulse = 0.5 + 0.5 * Math.sin(this.sim.simTime * 3 + i);
      ctx.strokeStyle = `rgba(232,168,48,${0.35 + pulse * 0.3})`;
      ctx.lineWidth = 1.5;
      const size = Math.max(4, 7 * zoom);
      ctx.beginPath();
      ctx.moveTo(ts.x - size, ts.y); ctx.lineTo(ts.x + size, ts.y);
      ctx.moveTo(ts.x, ts.y - size); ctx.lineTo(ts.x, ts.y + size);
      ctx.stroke();
      ctx.beginPath(); ctx.arc(ts.x, ts.y, size * 0.55, 0, Math.PI * 2); ctx.stroke();
    });
  }

  drawDrones(zoom) {
    const ctx = this.ctx;
    this.sim.drones.forEach((drone) => {
      if (!drone.alive || drone.mode === 'intercepted') return;
      const ds    = this.w2s(drone.x, drone.y);
      const scale = Math.max(0.4, Math.min(1.8, zoom * 3.2));

      if (drone.evading && drone.evadeWpt && drone.predictPt) this.drawDroneEvasion(drone, ds, scale, zoom);

      this.drawShape(ds.x, ds.y, drone.angle, scale, drone.col, drone.col);

      // Altitude label
      const z = Math.round(drone.z ?? 0);
      ctx.fillStyle = altColor(drone.z, 0.75);
      ctx.font = `${Math.max(9, 10 * zoom)}px "Share Tech Mono"`;
      ctx.fillText(`D${drone.id + 1} ${z}m`, ds.x + 13 * scale, ds.y - 9 * scale);
    });
  }

  drawDroneEvasion(drone, ds, scale, zoom) {
    const ctx = this.ctx;
    ctx.beginPath();
    ctx.moveTo(ds.x, ds.y);
    ctx.arc(ds.x, ds.y, DRONE_DETECT_R * zoom, drone.angle - DRONE_FOV / 2, drone.angle + DRONE_FOV / 2);
    ctx.closePath();
    ctx.fillStyle = 'rgba(232,168,48,0.07)'; ctx.fill();
    ctx.strokeStyle = 'rgba(232,168,48,0.35)'; ctx.lineWidth = 1; ctx.stroke();

    const pp = this.w2s(drone.predictPt.x, drone.predictPt.y);
    ctx.strokeStyle = 'rgba(224,75,74,0.55)'; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pp.x - 5, pp.y - 5); ctx.lineTo(pp.x + 5, pp.y + 5);
    ctx.moveTo(pp.x + 5, pp.y - 5); ctx.lineTo(pp.x - 5, pp.y + 5);
    ctx.stroke();

    const wp = this.w2s(drone.evadeWpt.x, drone.evadeWpt.y);
    ctx.beginPath(); ctx.arc(wp.x, wp.y, 6, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(91,163,232,0.8)'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.beginPath(); ctx.arc(wp.x, wp.y, 2, 0, Math.PI * 2);
    ctx.fillStyle = '#5ba3e8'; ctx.fill();

    const target = this.w2s(drone.rtx, drone.rty);
    ctx.beginPath(); ctx.moveTo(ds.x, ds.y); ctx.lineTo(wp.x, wp.y); ctx.lineTo(target.x, target.y);
    ctx.strokeStyle = 'rgba(91,163,232,0.25)'; ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#e8a830';
    ctx.font = `bold ${Math.max(10, 11 * zoom)}px "Share Tech Mono"`;
    ctx.fillText('!', ds.x - 3, ds.y - 15 * scale);
  }

  drawWaitingAntidrones() {
    const ctx   = this.ctx;
    const delay = Math.max(1, this.sim.getParam('adelay'));
    this.sim.antidrones.forEach((anti) => {
      if (!anti.alive || anti.mode !== 'waiting') return;
      const ds   = this.w2s(anti.x, anti.y);
      const frac = anti.launchDelay / delay;
      ctx.beginPath();
      ctx.arc(ds.x, ds.y, 12, -Math.PI / 2, Math.PI * 2 * frac - Math.PI / 2);
      ctx.strokeStyle = 'rgba(232,168,48,0.7)'; ctx.lineWidth = 2; ctx.stroke();
      ctx.fillStyle   = 'rgba(232,168,48,0.8)';
      ctx.font = '9px "Share Tech Mono"';
      ctx.fillText(`A${anti.id + 1}:${Math.ceil(anti.launchDelay)}s`, ds.x + 16, ds.y);
    });
  }

  drawActiveAntidrones(zoom) {
    const ctx       = this.ctx;
    const antiRange = this.sim.getParam('arange');
    this.sim.antidrones.forEach((anti) => {
      if (!anti.alive || anti.mode === 'base' || anti.mode === 'waiting') return;
      const ds    = this.w2s(anti.x, anti.y);
      const scale = Math.max(0.4, Math.min(1.8, zoom * 3.2));

      if (anti.mode === 'chase' || anti.mode === 'intercept' || anti.mode === 'lastknown') {
        ctx.beginPath();
        ctx.moveTo(ds.x, ds.y);
        ctx.arc(ds.x, ds.y, antiRange * zoom, anti.angle - ANTI_FOV / 2, anti.angle + ANTI_FOV / 2);
        ctx.closePath();
        ctx.fillStyle   = anti.mode === 'chase' ? 'rgba(224,75,74,0.10)' : 'rgba(224,75,74,0.04)';
        ctx.fill();
        ctx.strokeStyle = anti.mode === 'chase' ? 'rgba(224,75,74,0.40)' : 'rgba(224,75,74,0.18)';
        ctx.lineWidth = 1; ctx.stroke();
      }

      if (anti.target?.alive) {
        const ts = this.w2s(anti.target.x, anti.target.y);
        ctx.beginPath(); ctx.moveTo(ds.x, ds.y); ctx.lineTo(ts.x, ts.y);
        ctx.strokeStyle = 'rgba(224,75,74,0.28)'; ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]); ctx.stroke(); ctx.setLineDash([]);
      } else if (anti.mode === 'lastknown' && anti.lastKnownX !== null) {
        const ls = this.w2s(anti.lastKnownX, anti.lastKnownY);
        ctx.beginPath(); ctx.arc(ls.x, ls.y, 4, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(224,75,74,0.4)'; ctx.fill();
        ctx.beginPath(); ctx.moveTo(ds.x, ds.y); ctx.lineTo(ls.x, ls.y);
        ctx.strokeStyle = 'rgba(224,75,74,0.18)'; ctx.lineWidth = 1;
        ctx.setLineDash([2, 4]); ctx.stroke(); ctx.setLineDash([]);
      }

      this.drawShape(ds.x, ds.y, anti.angle, scale, '#e04b4b', '#c03030');
      const z = Math.round(anti.z ?? 0);
      ctx.fillStyle = 'rgba(224,75,74,0.75)';
      ctx.font = `${Math.max(9, 10 * zoom)}px "Share Tech Mono"`;
      ctx.fillText(`A${anti.id + 1} ${z}m`, ds.x + 13 * scale, ds.y - 9 * scale);
    });
  }

  drawExplosions(zoom) {
    const ctx = this.ctx;
    this.sim.explosions.forEach((exp) => {
      const es = this.w2s(exp.x, exp.y);
      const r  = exp.t;
      ctx.beginPath(); ctx.arc(es.x, es.y, 50 * r * zoom, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(224,75,74,${r * 0.45})`; ctx.fill();
      ctx.beginPath(); ctx.arc(es.x, es.y, 25 * r * zoom, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(232,168,48,${r * 0.65})`; ctx.fill();
      ctx.beginPath(); ctx.arc(es.x, es.y, 10 * r * zoom, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(255,255,255,${r * 0.85})`; ctx.fill();
    });
  }

  drawShape(x, y, angle, scale, bodyColor, wingColor) {
    const ctx = this.ctx;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle);
    ctx.beginPath();
    ctx.moveTo( 16 * scale, 0);
    ctx.lineTo( -9 * scale, -7 * scale);
    ctx.lineTo( -5 * scale,  0);
    ctx.lineTo( -9 * scale,  7 * scale);
    ctx.closePath();
    ctx.fillStyle = bodyColor; ctx.fill();
    ctx.beginPath();
    ctx.moveTo(  3 * scale,   0);
    ctx.lineTo( -4 * scale, -12 * scale);
    ctx.lineTo( -8 * scale, -12 * scale);
    ctx.lineTo( -5 * scale,   0);
    ctx.lineTo( -8 * scale,  12 * scale);
    ctx.lineTo( -4 * scale,  12 * scale);
    ctx.closePath();
    ctx.fillStyle = wingColor; ctx.fill();
    ctx.beginPath();
    ctx.moveTo( -5 * scale,  0);
    ctx.lineTo( -9 * scale, -4 * scale);
    ctx.lineTo(-11 * scale,  0);
    ctx.lineTo( -9 * scale,  4 * scale);
    ctx.closePath();
    ctx.fillStyle = bodyColor; ctx.fill();
    ctx.restore();
  }
}

// ── Elevation (side-view) renderer ───────────────────────────────────────────
// Shows world X on horizontal axis and altitude Z on vertical axis.
// The X axis always spans 0..FIELD_SIZE (independent of top-view zoom).
export class ElevationRenderer {
  constructor(canvas, sim, topRenderer) {
    this.canvas = canvas;
    this.ctx    = canvas.getContext('2d');
    this.sim    = sim;
    this.top    = topRenderer;
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  resize() {
    this.width  = this.canvas.offsetWidth  || 600;
    this.height = this.canvas.offsetHeight || 150;
    this.canvas.width  = this.width;
    this.canvas.height = this.height;
  }

  // Same horizontal transform as top view (zoom + pan), but centered on this canvas's width
  wx(worldX) {
    const zoom = this.top.getZoom();
    const cx   = FIELD_SIZE / 2 + this.top.camOffX;
    return (worldX - cx) * zoom + this.width / 2;
  }

  wy(z) {
    const PAD_T = 16, PAD_B = 16;
    return PAD_T + (1 - Math.min(1, Math.max(0, z) / ELEVATION_VIEW_MAX_ALT)) * (this.height - PAD_T - PAD_B);
  }

  draw() {
    const { ctx, width: w, height: h } = this;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#0c0e0d';
    ctx.fillRect(0, 0, w, h);

    ctx.font = '9px "Share Tech Mono"';

    // Altitude grid lines
    for (let alt = 0; alt <= ELEVATION_VIEW_MAX_ALT; alt += 30) {
      const sy = this.wy(alt);
      ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(w, sy);
      ctx.strokeStyle = alt === 0 ? 'rgba(78,203,113,0.40)' : 'rgba(78,203,113,0.08)';
      ctx.lineWidth = alt === 0 ? 1.5 : 1;
      ctx.stroke();
      ctx.fillStyle = 'rgba(78,203,113,0.35)';
      ctx.fillText(alt === 0 ? 'GND' : alt + 'm', 3, sy - 2);
    }

    // Cruise altitude dashed reference
    const csy = this.wy(CRUISE_ALT);
    ctx.beginPath(); ctx.moveTo(0, csy); ctx.lineTo(w, csy);
    ctx.strokeStyle = 'rgba(78,203,113,0.20)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 5]); ctx.stroke(); ctx.setLineDash([]);

    // Drone trails (X, Z projection)
    this.sim.drones.forEach((drone) => {
      if (drone.trail.length < 2) return;
      for (let i = 1; i < drone.trail.length; i++) {
        const p0 = drone.trail[i - 1], p1 = drone.trail[i];
        const alpha = (i / drone.trail.length) * 0.45;
        ctx.beginPath();
        ctx.moveTo(this.wx(p0.x), this.wy(p0.z ?? CRUISE_ALT));
        ctx.lineTo(this.wx(p1.x), this.wy(p1.z ?? CRUISE_ALT));
        ctx.strokeStyle = `rgba(91,163,232,${alpha})`;
        ctx.lineWidth = 1; ctx.stroke();
      }
    });

    // Antidrone trails
    this.sim.antidrones.forEach((anti) => {
      if (anti.trail.length < 2) return;
      for (let i = 1; i < anti.trail.length; i++) {
        const p0 = anti.trail[i - 1], p1 = anti.trail[i];
        const alpha = (i / anti.trail.length) * 0.45;
        ctx.beginPath();
        ctx.moveTo(this.wx(p0.x), this.wy(p0.z ?? MIN_ALT));
        ctx.lineTo(this.wx(p1.x), this.wy(p1.z ?? MIN_ALT));
        ctx.strokeStyle = `rgba(224,75,74,${alpha})`;
        ctx.lineWidth = 1; ctx.stroke();
      }
    });

    // Drone dots
    this.sim.drones.forEach((drone) => {
      if (!drone.alive || drone.mode === 'intercepted') return;
      const sx = this.wx(drone.x), sy = this.wy(drone.z ?? CRUISE_ALT);
      ctx.beginPath(); ctx.arc(sx, sy, 4.5, 0, Math.PI * 2);
      ctx.fillStyle = drone.col ?? '#5ba3e8'; ctx.fill();
      // altitude label
      ctx.fillStyle = altColor(drone.z, 0.65);
      ctx.fillText(Math.round(drone.z ?? 0) + 'm', sx + 6, sy - 3);
    });

    // Antidrone dots
    this.sim.antidrones.forEach((anti) => {
      if (!anti.alive || anti.mode === 'base' || anti.mode === 'waiting') return;
      const sx = this.wx(anti.x), sy = this.wy(anti.z ?? 0);
      ctx.beginPath(); ctx.arc(sx, sy, 4, 0, Math.PI * 2);
      ctx.fillStyle = '#e04b4b'; ctx.fill();
    });

    // Cameras (on masts at CAMERA_Z height)
    this.sim.cameras.forEach((cam) => {
      const sx = this.wx(cam.x), sy = this.wy(cam.z ?? 0);
      ctx.beginPath(); ctx.arc(sx, sy, 3, 0, Math.PI * 2);
      ctx.fillStyle = cam.detected ? 'rgba(232,168,48,0.8)' : 'rgba(91,163,232,0.45)'; ctx.fill();
      // mast line to ground
      ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx, this.wy(0));
      ctx.strokeStyle = 'rgba(91,163,232,0.15)'; ctx.lineWidth = 1; ctx.stroke();
    });

    // Targets — pin above ground line (cross sits entirely above GND, stem touches GND)
    this.sim.targets.forEach((target) => {
      if (target.hit) return;
      const sx = this.wx(target.x), gnd = this.wy(0);
      ctx.strokeStyle = 'rgba(232,168,48,0.75)'; ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(sx, gnd); ctx.lineTo(sx, gnd - 14);   // stem up from ground
      ctx.moveTo(sx - 6, gnd - 9); ctx.lineTo(sx + 6, gnd - 9); // crossbar
      ctx.stroke();
      // foot dot on ground
      ctx.beginPath(); ctx.arc(sx, gnd, 2, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(232,168,48,0.75)'; ctx.fill();
    });

    // Label
    ctx.fillStyle = 'rgba(78,203,113,0.25)';
    ctx.fillText(`ELEV  X ->  ALT 0-${ELEVATION_VIEW_MAX_ALT}m`, w - 160, h - 4);
  }
}
