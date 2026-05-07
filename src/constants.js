// 3D simulation constants
// Physics model from DS_3_drone_school_3.html: vx/vy/vz, thrust, drag, altitude
export const DT          = 0.02;        // s  — 50 Hz sim tick
export const FIELD_SIZE  = 3000;        // m  — arena size
export const GRAVITY     = 9.81;        // m/s²

// Altitude
export const CRUISE_ALT  = 80;          // m  — default drone cruise altitude
export const MIN_ALT     = 5;           // m
export const MAX_ALT     = 250;         // m

// Geometry
export const ARRIVAL_R   = 25;          // m  — target hit radius
export const INTERCEPT_R = 30;          // m  — 3D intercept radius
export const ATTACK_COMMIT_DIST = 300;  // m  — drone commits to target within this
export const CENTER_ZONE = 900;         // m  — central zone radius (render only)
export const DEPLOYMENT_MARGIN = 60;    // m  — min spawn distance from edge

// FOV
export const CAM_FOV     = 50 * Math.PI / 180;  // rad  — ground camera horizontal FOV
export const ANTI_FOV    = 55 * Math.PI / 180;  // rad  — antidrone camera horizontal FOV
export const DRONE_FOV   = Math.PI * 2;          // rad  — drone threat radar (omnidirectional)
export const DRONE_DETECT_R  = 600;     // m  — drone threat detection range
export const CAM_ELEV_R  = 200;         // m  — ground camera elevation visibility range

// Turn rates (rad/s)
export const DEFAULT_DRONE_TURN_RATE = 45 * Math.PI / 180;
export const DEFAULT_ANTI_TURN_RATE  = 90 * Math.PI / 180;

// Physics response times (s) — how fast velocity tracks the target
export const DRONE_RESPONSE_TIME = 0.35;  // s
export const ANTI_RESPONSE_TIME  = 0.18;  // s
export const DRONE_MAX_VZ  = 12;   // m/s  vertical speed limit
export const ANTI_MAX_VZ   = 20;   // m/s

// Base position
export const BASE_X = FIELD_SIZE / 2;
export const BASE_Y = FIELD_SIZE / 2;
export const CAMERA_COUNT = 10;

export const TRAIL_LEN = 500;
export const MAX_DRONES    = 15;
export const MAX_ANTIDRONES = 15;

export const DEFAULT_PARAMS = Object.freeze({
  dspeed:  200,   // km/h
  aspeed:  350,   // km/h
  dmass:   2.5,   // kg
  amass:   3.0,   // kg
  camrange: 900,  // m   ground camera range
  arange:  650,   // m   antidrone camera range
  ndrones:   5,
  nanti:     5,
  adelay:    5,   // s   launch delay
});

export const PARAM_DEFS = Object.freeze([
  { id: 'dspeed',   label: 'Скорость дрона (км/ч)',      min: 80,  max: 300,  step: 10 },
  { id: 'aspeed',   label: 'Скорость антидрона (км/ч)',  min: 150, max: 500,  step: 10 },
  { id: 'camrange', label: 'Дальность камеры (м)',        min: 300, max: 2500, step: 50 },
  { id: 'arange',   label: 'Дальность антидрона (м)',     min: 200, max: 1000, step: 50 },
  { id: 'ndrones',  label: 'Кол-во дронов',              min: 1,   max: MAX_DRONES,     step: 1 },
  { id: 'nanti',    label: 'Кол-во антидронов',          min: 1,   max: MAX_ANTIDRONES, step: 1 },
  { id: 'adelay',   label: 'Задержка вылета (с)',         min: 0,   max: 30,   step: 1 },
]);

export const DRONE_COLS = Object.freeze([
  '#5ba3e8', '#3ecaa5', '#7ec87a', '#e8a830', '#c87aad',
  '#9a8fe0', '#e87a5b', '#5be8c8', '#e8d85b', '#a0e85b',
  '#5b8be8', '#e85ba0', '#aaa0ff', '#ffaa60', '#60ffcc',
]);
