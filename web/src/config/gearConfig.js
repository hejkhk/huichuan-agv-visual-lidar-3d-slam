// Normal driving keeps the original four gears. Mapping mode uses the two
// measured low-slip profiles from 速度.txt.
const PULSE_PER_REV = 8388608;
const GEAR_RATIO = 25;
const WHEEL_RADIUS_M = 0.0755;
const HALF_TRACK_M = 0.2825;
const MAX_MANUAL_WHEEL_CNT_PER_SEC = 100000000;
const MAX_MANUAL_ANGULAR_RAD_PER_SEC = 2.8;
const BASE_SPEED_CNT_PER_SEC = 15000000;
const TURN_SPEED_CNT_PER_SEC = 10000000;

const cntPerMeter = (PULSE_PER_REV * GEAR_RATIO) / (2 * Math.PI * WHEEL_RADIUS_M);

function cntToMps(cntPerSec) {
  return Math.min(cntPerSec, MAX_MANUAL_WHEEL_CNT_PER_SEC) / cntPerMeter;
}

function mpsToCnt(mps) {
  return Math.round(mps * cntPerMeter);
}

function round3(value) {
  return Math.round(value * 1000) / 1000;
}

function makeNormalCmdVel(multiplier) {
  const linear = cntToMps(BASE_SPEED_CNT_PER_SEC * multiplier);
  const turnWheelSpeed = cntToMps(TURN_SPEED_CNT_PER_SEC * multiplier);
  return {
    linear: round3(linear),
    angular: round3(Math.min(turnWheelSpeed / HALF_TRACK_M, MAX_MANUAL_ANGULAR_RAD_PER_SEC)),
    arcLinear: round3(linear * 0.5),
    arcAngular: round3(Math.min(linear / (2 * HALF_TRACK_M), MAX_MANUAL_ANGULAR_RAD_PER_SEC)),
  };
}

function normalGear(gear, multiplier, color, colorText, hex, name) {
  const speedCntPerSec = Math.min(BASE_SPEED_CNT_PER_SEC * multiplier, MAX_MANUAL_WHEEL_CNT_PER_SEC);
  return {
    label: `${gear}挡`,
    shortLabel: `${gear}`,
    name,
    ledColorName: color,
    ledColorText: colorText,
    ledColorHex: hex,
    multiplier,
    speedCntPerSec,
    speedText: `${speedCntPerSec / 1000000}M cnt/s`,
    profile: "normal",
    cmdVel: makeNormalCmdVel(multiplier),
  };
}

export const GEAR_CONFIG = Object.freeze({
  1: normalGear(1, 1, "green", "绿色", "#16a36a", "Slow"),
  2: normalGear(2, 2, "blue", "蓝色", "#3478f6", "Normal"),
  3: normalGear(3, 5, "amber", "琥珀色", "#f0a126", "Fast"),
  4: normalGear(4, 30, "red", "红色", "#e5484d", "Turbo"),
});

export const MAPPING_GEAR_CONFIG = Object.freeze({
  1: {
    label: "建图 1挡",
    shortLabel: "M1",
    name: "Mapping Smooth",
    ledColorName: "cyan",
    ledColorText: "青色",
    ledColorHex: "#0ea5a8",
    multiplier: 1,
    speedCntPerSec: mpsToCnt(0.20),
    speedText: "0.20 m/s · 9°/s",
    profile: "mapping",
    cmdVel: { linear: 0.20, arcLinear: 0.125, arcAngular: 0.05 / 0.565, angular: Math.PI / 20 },
  },
  2: {
    label: "建图 2挡",
    shortLabel: "M2",
    name: "Mapping Agile",
    ledColorName: "indigo",
    ledColorText: "靛蓝色",
    ledColorHex: "#6366f1",
    multiplier: 2,
    speedCntPerSec: mpsToCnt(0.20),
    speedText: "0.20 m/s · 12°/s",
    profile: "mapping",
    cmdVel: { linear: 0.20, arcLinear: 0.12, arcAngular: 0.08 / 0.565, angular: Math.PI / 15 },
  },
});

export const DRIVE_PROFILES = Object.freeze({
  normal: { label: "常规模式", gears: GEAR_CONFIG, sequence: [1, 2, 3, 4] },
  mapping: { label: "建图模式", gears: MAPPING_GEAR_CONFIG, sequence: [1, 2] },
});

export const DEFAULT_GEAR = 1;
export const DEFAULT_PROFILE = "mapping";
