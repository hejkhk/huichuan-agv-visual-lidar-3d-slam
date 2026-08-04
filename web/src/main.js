import "./style.css";
import {
  ArrowDown,
  ArrowDownLeft,
  ArrowDownRight,
  ArrowUp,
  ArrowUpLeft,
  ArrowUpRight,
  Camera,
  CameraOff,
  CircleStop,
  Eraser,
  FileClock,
  Gamepad2,
  Gauge,
  Map as MapIcon,
  MapPinned,
  Maximize2,
  Minimize2,
  Moon,
  Navigation,
  OctagonX,
  Radio,
  RadioTower,
  Repeat2,
  RotateCcw,
  RotateCw,
  Route,
  Scan,
  ScanLine,
  ShieldCheck,
  Square,
  Sun,
  TriangleAlert,
  Unplug,
  Video,
  createIcons,
} from "lucide";
import { DRIVE_PROFILES, GEAR_CONFIG } from "./config/gearConfig.js";
import {
  connectRosbridge,
  disconnectRosbridge,
  isRosConnected,
  normalizeRosbridgeUrl,
} from "./ros/rosConnection.js";
import { RobotPublishers } from "./ros/publishers.js";
import { RobotSubscribers } from "./ros/subscribers.js";
import { GearController } from "./control/gearController.js";
import { MotionController } from "./control/motionController.js";
import { EstopController } from "./control/estopController.js";
import { VideoStream } from "./video/videoStream.js";
import { MapRenderer } from "./map/mapRenderer.js";
import { formatClock } from "./utils/time.js";
import { normalizeRobotIp, validateRosUrl, validateVideoUrl } from "./utils/validators.js";

const STORAGE = Object.freeze({
  robotIp: "robot-web-console.robot-ip",
  videoUrl: "robot-web-console.video-url",
  rosUrl: "robot-web-console.ros-url",
  theme: "robot-web-console.theme",
});

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
const LUCIDE_ICONS = {
  ArrowDown,
  ArrowDownLeft,
  ArrowDownRight,
  ArrowUp,
  ArrowUpLeft,
  ArrowUpRight,
  Camera,
  CameraOff,
  CircleStop,
  Eraser,
  FileClock,
  Gamepad2,
  Gauge,
  Map: MapIcon,
  MapPinned,
  Maximize2,
  Minimize2,
  Moon,
  Navigation,
  OctagonX,
  Radio,
  RadioTower,
  Repeat2,
  RotateCcw,
  RotateCw,
  Route,
  Scan,
  ScanLine,
  ShieldCheck,
  Square,
  Sun,
  TriangleAlert,
  Unplug,
  Video,
};

const elements = {
  robotIp: $("#robotIp"),
  videoUrl: $("#videoUrl"),
  rosUrl: $("#rosUrl"),
  message: $("#connectionMessage"),
  rosSummary: $("#rosSummary"),
  videoSummary: $("#videoSummary"),
  activeVideoUrl: $("#activeVideoUrl"),
  videoPanelStatus: $("#videoPanelStatus"),
  controlLockStatus: $("#controlLockStatus"),
  baselineStatus: $("#baselineStatus"),
  ledLamp: $("#ledLamp"),
  ledColorText: $("#ledColorText"),
  gearText: $("#gearText"),
  multiplierText: $("#multiplierText"),
  speedText: $("#speedText"),
  driveProfileText: $("#driveProfileText"),
  statusSource: $("#statusSource"),
  statusRos: $("#statusRos"),
  statusVideo: $("#statusVideo"),
  statusGear: $("#statusGear"),
  statusLed: $("#statusLed"),
  statusSpeed: $("#statusSpeed"),
  statusCommand: $("#statusCommand"),
  statusMap: $("#statusMap"),
  statusPose: $("#statusPose"),
  statusEstop: $("#statusEstop"),
  robotMode: $("#robotMode"),
  robotState: $("#robotState"),
  robotBattery: $("#robotBattery"),
  robotMessage: $("#robotMessage"),
  txFrameAge: $("#txFrameAge"),
  txFrameText: $("#txFrameText"),
  echoFrameAge: $("#echoFrameAge"),
  echoFrameList: $("#echoFrameList"),
  naviFrameAge: $("#naviFrameAge"),
  naviFrameText: $("#naviFrameText"),
  eventList: $("#eventList"),
  estopButton: $("#estopButton"),
  resetEstopButton: $("#resetEstopButton"),
  mapResolution: $("#mapResolution"),
  mapSize: $("#mapSize"),
  mapUpdated: $("#mapUpdated"),
  mapZoom: $("#mapZoom"),
  fitMapButton: $("#fitMapButton"),
  resetMapButton: $("#resetMapButton"),
  goalReadout: $("#goalReadout"),
  autoMappingToggle: $("#autoMappingToggle"),
  sendGoalButton: $("#sendGoalButton"),
  clearGoalButton: $("#clearGoalButton"),
  baselineCaptureButton: $("#baselineCaptureButton"),
  motionSerialToggle: $("#motionSerialToggle"),
  obstacleFillToggle: $("#obstacleFillToggle"),
  roiBoxToggle: $("#roiBoxToggle"),
  rgbDebugTextToggle: $("#rgbDebugTextToggle"),
  slamLogToggle: $("#slamLogToggle"),
  slamLogInterval: $("#slamLogInterval"),
  controlModeToggle: $("#controlModeToggle"),
  mappingModeToggle: $("#mappingModeToggle"),
  echoModeToggle: $("#echoModeToggle"),
  themeToggle: $("#themeToggle"),
  videoPanel: $("#videoPanel"),
  mapPanel: $("#mapPanel"),
  videoExpandButton: $("#videoExpandButton"),
  mapExpandButton: $("#mapExpandButton"),
  serialCommandButtons: $$('[data-serial-action]'),
  tabButtons: $$('[data-tab]'),
  tabPanes: $$('[data-pane]'),
};

const state = {
  ros: "disconnected",
  video: "disconnected",
  estop: false,
  baselineReady: false,
  controlMode: "move",
  mappingMode: true,
  echoEnabled: false,
  controlStateSeen: false,
  lastControlSignature: "",
  lastMotionEvent: 0,
  selectedGoal: null,
  focusedPanel: null,
  txStatusUrl: "",
  txPollTimer: null,
  preferRosTx: false,
  echoFrames: [],
  autoMappingEnabled: false,
  autoMappingState: "disabled",
  autoMappingStatusSeen: false,
  autoMappingPending: false,
  autoMappingRequestTimer: null,
  autonomyArmPending: false,
  navGoalSendPending: false,
  lastAutoMappingSignature: "",
  runtimeOptions: {
    motion_serial_enabled: true,
    show_obstacle_fill: false,
    show_roi_polygons: false,
    show_rgb_debug_text: false,
  },
  slamLogEnabled: false,
  slamLogIntervalSec: 3,
};

const SLAM_LOG_INTERVAL_MIN = 0.5;
const SLAM_LOG_INTERVAL_MAX = 60;
const SLAM_LOG_INTERVAL_STEP = 0.5;

function normalizeSlamLogInterval(value) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) return null;
  const clamped = Math.min(SLAM_LOG_INTERVAL_MAX, Math.max(SLAM_LOG_INTERVAL_MIN, parsed));
  return Math.round(clamped / SLAM_LOG_INTERVAL_STEP) * SLAM_LOG_INTERVAL_STEP;
}

function refreshIcons() {
  createIcons({ icons: LUCIDE_ICONS, attrs: { "stroke-width": 1.8 } });
}

function replaceIcon(button, iconName) {
  const oldIcon = button.querySelector("svg, [data-lucide]");
  const nextIcon = document.createElement("i");
  nextIcon.dataset.lucide = iconName;
  if (oldIcon) oldIcon.replaceWith(nextIcon);
  else button.prepend(nextIcon);
  refreshIcons();
}

function setIconLabel(button, iconName, label) {
  button.replaceChildren();
  const icon = document.createElement("i");
  icon.dataset.lucide = iconName;
  button.append(icon, document.createTextNode(label));
  refreshIcons();
}

function setSummary(element, iconName, label, statusClass) {
  element.replaceChildren();
  const icon = document.createElement("i");
  icon.dataset.lucide = iconName;
  const text = document.createElement("span");
  text.textContent = label;
  element.append(icon, text);
  element.className = `status-pill ${statusClass}`;
  refreshIcons();
}

function saveSettings() {
  localStorage.setItem(STORAGE.robotIp, elements.robotIp.value.trim());
  localStorage.setItem(STORAGE.videoUrl, elements.videoUrl.value.trim());
  localStorage.setItem(STORAGE.rosUrl, elements.rosUrl.value.trim());
}

function fillConnectionUrls(ip, { overwrite = true } = {}) {
  if (!ip) return;
  if (overwrite || !elements.videoUrl.value.trim()) {
    elements.videoUrl.value = `http://${ip}:8080/video_feed`;
  }
  if (overwrite || !elements.rosUrl.value.trim()) {
    elements.rosUrl.value = `ws://${ip}:9090`;
  }
}

function loadSettings() {
  const host = window.location.hostname || "192.168.1.201";
  const savedIp = localStorage.getItem(STORAGE.robotIp);
  elements.robotIp.value = savedIp || host;
  elements.videoUrl.value = localStorage.getItem(STORAGE.videoUrl) || "";
  elements.rosUrl.value = localStorage.getItem(STORAGE.rosUrl) || "";
  fillConnectionUrls(elements.robotIp.value, { overwrite: false });

  const savedTheme = localStorage.getItem(STORAGE.theme);
  const preferredTheme = window.matchMedia?.("(prefers-color-scheme: light)").matches ? "light" : "dark";
  document.documentElement.dataset.theme = savedTheme || preferredTheme;
}

function showMessage(text, tone = "neutral") {
  elements.message.textContent = text;
  elements.message.dataset.tone = tone;
}

function addEvent(text) {
  const item = document.createElement("li");
  const time = document.createElement("time");
  const message = document.createElement("span");
  time.textContent = formatClock();
  message.textContent = text;
  item.append(time, message);
  elements.eventList.prepend(item);
  while (elements.eventList.children.length > 5) elements.eventList.lastElementChild.remove();
}

function setStatusValue(element, text, className) {
  element.textContent = text;
  element.className = className ? `status-value ${className}` : "";
}

function setTxFrameWaiting(text = "Waiting for /robot/serial_debug") {
  elements.txFrameAge.textContent = "Waiting";
  elements.txFrameText.textContent = text;
}

function txStatusUrlFromVideoUrl(videoUrl) {
  const url = new URL(videoUrl);
  return `${url.origin}/tx_status`;
}

async function refreshTxFrame() {
  if (!state.txStatusUrl || (state.preferRosTx && state.ros === "connected")) return;
  try {
    const response = await fetch(`${state.txStatusUrl}?t=${Date.now()}`, { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    if (!data.has_tx) {
      elements.txFrameAge.textContent = "No TX";
      elements.txFrameText.textContent = data.error || "No serial frame has been sent yet.";
      return;
    }
    const age = typeof data.age_sec === "number" ? `${data.age_sec.toFixed(1)}s ago` : "Updated";
    elements.txFrameAge.textContent = `${data.status || "tx"} · ${age}`;
    elements.txFrameText.textContent = `${data.command || ""}\n${data.frame || ""}`.trim();
  } catch (error) {
    elements.txFrameAge.textContent = "Offline";
    elements.txFrameText.textContent = `Cannot read ${state.txStatusUrl}\n${error.message}`;
  }
}

function startTxFramePolling(videoUrl) {
  stopTxFramePolling();
  try {
    state.txStatusUrl = txStatusUrlFromVideoUrl(videoUrl);
  } catch {
    state.txStatusUrl = "";
    setTxFrameWaiting("Invalid video URL; cannot build /tx_status URL.");
    return;
  }
  setTxFrameWaiting(`Connecting ${state.txStatusUrl}`);
  refreshTxFrame();
  state.txPollTimer = window.setInterval(refreshTxFrame, 500);
}

function stopTxFramePolling() {
  if (state.txPollTimer) window.clearInterval(state.txPollTimer);
  state.txPollTimer = null;
  state.txStatusUrl = "";
}

function setTxFrameFromSerial(data) {
  state.preferRosTx = true;
  elements.txFrameAge.textContent = `${data.source || data.kind || "serial"} · ${formatClock()}`;
  const status = data.kind === "tx_blocked" ? "blocked" : data.ok === false ? "bad" : "sent";
  const header = `status=${status} cmd=${data.cmd_hex || ""} ${data.cmd_name || ""} len=${data.len ?? "-"} checksum=${data.checksum_hex || ""}`;
  const body = data.line || data.hex || JSON.stringify(data);
  elements.txFrameText.textContent = `${header}\n${body}`.trim();
}

function pushEchoFrame(data) {
  state.echoFrames.unshift(data);
  state.echoFrames = state.echoFrames.slice(0, 12);
  elements.echoFrameAge.textContent = `Latest ${formatClock()}`;
  elements.echoFrameList.replaceChildren();
  state.echoFrames.forEach((frame) => {
    const item = document.createElement("li");
    item.className = frame.ok ? "is-ok" : "is-bad";
    item.textContent = frame.line || frame.hex || JSON.stringify(frame);
    elements.echoFrameList.append(item);
  });
}

function setNaviFrame(data) {
  elements.naviFrameAge.textContent = `Latest ${formatClock()}`;
  elements.naviFrameText.textContent = [
    `yaw=${Number(data.yaw_deg ?? 0).toFixed(2)} deg`,
    `vx=${Number(data.vx_mps ?? 0).toFixed(3)} m/s`,
    `vz=${Number(data.vz_deg_s ?? 0).toFixed(2)} deg/s (${Number(data.vz_rad_s ?? 0).toFixed(3)} rad/s)`,
    `raw=${JSON.stringify(data.raw || [])}`,
    `checksum=${data.checksum_hex || "-"} calc=${data.calc_hex || "-"}`,
    data.hex || "",
  ].join("\n");
}

function handleSerialDebug(data) {
  if (!data || typeof data !== "object") return;
  if (["tx", "tx_blocked", "control"].includes(data.kind)) setTxFrameFromSerial(data);
  else if (data.kind === "echo" || (data.kind === "rx" && data.name === "AA55_20B")) pushEchoFrame(data);
  else if (data.kind === "navi") setNaviFrame(data);
}

function updateBaselineUi() {
  elements.baselineStatus.textContent = state.baselineReady ? "已标定" : "未标定";
  elements.baselineStatus.className = `state-chip ${state.baselineReady ? "is-ready" : "is-warning"}`;
}

function setModeToggle(button, active, label) {
  button.classList.toggle("is-active", active);
  button.querySelector("strong").textContent = label;
  button.setAttribute("aria-pressed", String(active));
}

function updateModeUi() {
  setModeToggle(
    elements.controlModeToggle,
    state.controlMode === "move",
    state.controlMode === "move" ? "MOVE 接管" : "PS2 控制",
  );
  setModeToggle(
    elements.mappingModeToggle,
    state.mappingMode,
    state.mappingMode ? "建图 ON" : "建图 OFF",
  );
  setModeToggle(
    elements.echoModeToggle,
    state.echoEnabled,
    state.echoEnabled ? "回响 ON" : "回响 OFF",
  );
  elements.motionSerialToggle.checked = state.runtimeOptions.motion_serial_enabled;
  elements.obstacleFillToggle.checked = state.runtimeOptions.show_obstacle_fill;
  elements.roiBoxToggle.checked = state.runtimeOptions.show_roi_polygons;
  elements.rgbDebugTextToggle.checked = state.runtimeOptions.show_rgb_debug_text;
  setIconLabel(
    elements.slamLogToggle,
    "file-clock",
    state.slamLogEnabled ? "SLAM Log ON" : "SLAM Log OFF",
  );
  elements.slamLogToggle.classList.toggle("button-on", state.slamLogEnabled);
  if (document.activeElement !== elements.slamLogInterval) {
    elements.slamLogInterval.value = String(state.slamLogIntervalSec);
  }
  updateBaselineUi();
  updateAutoMappingUi();
}

function updateAutoMappingUi() {
  const connected = state.ros === "connected" && isRosConnected();
  const label = state.autoMappingPending
    ? "自动建图切换中"
    : state.autoMappingEnabled ? "自动建图 ON" : "自动建图 OFF";
  setIconLabel(elements.autoMappingToggle, "map-pinned", label);
  elements.autoMappingToggle.classList.toggle(
    "button-on",
    state.autoMappingEnabled && !state.autoMappingPending,
  );
  elements.autoMappingToggle.classList.toggle("is-pending", state.autoMappingPending);
  elements.autoMappingToggle.setAttribute("aria-pressed", String(state.autoMappingEnabled));
  elements.autoMappingToggle.title = state.autoMappingStatusSeen
    ? `自动建图状态：${state.autoMappingState}`
    : "等待 /auto_mapping/status";
  elements.autoMappingToggle.disabled = !connected
    || state.autoMappingPending
    || state.autonomyArmPending;
}

function updateMotionAvailability() {
  const connected = state.ros === "connected" && isRosConnected();
  const enabled =
    connected &&
    !state.estop &&
    state.baselineReady &&
    state.controlMode === "move" &&
    state.runtimeOptions.motion_serial_enabled;
  motionController.setEnabled(enabled);

  let lockText = "ROS required";
  if (connected && state.estop) lockText = "E-STOP LOCKED";
  else if (connected && !state.baselineReady) lockText = "请先采集 Baseline";
  else if (connected && state.controlMode === "ps2") lockText = "PS2 控制中";
  else if (connected && !state.runtimeOptions.motion_serial_enabled) lockText = "运动串口已关闭";
  else if (enabled) lockText = "Controls enabled";

  elements.controlLockStatus.textContent = lockText;
  elements.controlLockStatus.className = `panel-status ${enabled ? "success" : "warning"}`;
  elements.baselineCaptureButton.disabled = !connected;
  elements.controlModeToggle.disabled = !connected || (state.controlMode === "move" && !state.baselineReady);
  elements.mappingModeToggle.disabled = !connected;
  elements.echoModeToggle.disabled = !connected;
  elements.slamLogToggle.disabled = !connected;
  elements.slamLogInterval.disabled = !connected;
  elements.autoMappingToggle.disabled = !connected
    || state.autoMappingPending
    || state.autonomyArmPending;
  elements.estopButton.disabled = !connected || state.estop;
  elements.resetEstopButton.disabled = !connected || !state.estop;
  elements.serialCommandButtons.forEach((button) => { button.disabled = !connected; });
}

function handleAutoMappingStatus(data) {
  if (!data || typeof data !== "object" || typeof data.enabled !== "boolean") return;
  const previousEnabled = state.autoMappingEnabled;
  const hadStatus = state.autoMappingStatusSeen;
  state.autoMappingEnabled = data.enabled;
  state.autoMappingState = String(data.state || (data.enabled ? "waiting" : "disabled"));
  state.autoMappingStatusSeen = true;
  state.autoMappingPending = false;
  if (state.autoMappingRequestTimer) {
    window.clearTimeout(state.autoMappingRequestTimer);
    state.autoMappingRequestTimer = null;
  }

  const signature = JSON.stringify({
    enabled: state.autoMappingEnabled,
    state: state.autoMappingState,
    completed: Boolean(data.completed),
  });
  if (hadStatus && previousEnabled !== state.autoMappingEnabled) {
    addEvent(`自动建图已${state.autoMappingEnabled ? "开启" : "关闭"}`);
  } else if (
    state.lastAutoMappingSignature
    && signature !== state.lastAutoMappingSignature
    && data.completed
  ) {
    addEvent("自动建图探索完成");
  }
  state.lastAutoMappingSignature = signature;
  updateAutoMappingUi();
}

function applyControlState(data) {
  if (!data || typeof data !== "object") return;
  state.controlStateSeen = true;
  if (["move", "ps2"].includes(data.control_mode)) state.controlMode = data.control_mode;
  if (typeof data.echo_enabled === "boolean") state.echoEnabled = data.echo_enabled;
  if (typeof data.software_estop === "boolean") setEstopUi(data.software_estop);
  if (typeof data.mapping_mode === "boolean") state.mappingMode = data.mapping_mode;
  if (typeof data.baseline_ready === "boolean") state.baselineReady = data.baseline_ready;
  for (const key of Object.keys(state.runtimeOptions)) {
    if (typeof data[key] === "boolean") state.runtimeOptions[key] = data[key];
  }
  if (typeof data.slam_log_enabled === "boolean") state.slamLogEnabled = data.slam_log_enabled;
  const slamLogInterval = normalizeSlamLogInterval(data.slam_log_interval_sec);
  if (slamLogInterval !== null) state.slamLogIntervalSec = slamLogInterval;

  const profile = data.web_profile === "normal" || data.web_profile === "mapping"
    ? data.web_profile
    : state.mappingMode ? "mapping" : "normal";
  gearController.setProfile(profile);
  const requestedGear = Number(data.web_gear);
  if (DRIVE_PROFILES[profile].gears[requestedGear]) gearController.setGear(requestedGear);
  else gearController.setGear(gearController.gear);

  const signature = JSON.stringify({
    controlMode: state.controlMode,
    mappingMode: state.mappingMode,
    echoEnabled: state.echoEnabled,
    runtime: state.runtimeOptions,
    slamLogEnabled: state.slamLogEnabled,
    slamLogIntervalSec: state.slamLogIntervalSec,
  });
  if (state.lastControlSignature && signature !== state.lastControlSignature) {
    addEvent("控制状态已从机器人同步");
  }
  state.lastControlSignature = signature;
  elements.statusSource.textContent = "ROS /robot/control_state";
  updateModeUi();
  updateMotionAvailability();
  tryCompleteAutonomyArm();
}

function handleBaselineReady(ready) {
  state.baselineReady = Boolean(ready);
  updateBaselineUi();
  updateMotionAvailability();
}

function updateGoalUi(goal = state.selectedGoal) {
  state.selectedGoal = goal;
  if (!goal) {
    elements.goalReadout.textContent = "点击地图选择目标点";
    elements.sendGoalButton.disabled = true;
    elements.clearGoalButton.disabled = true;
    return;
  }
  elements.goalReadout.textContent = `x=${goal.x.toFixed(2)} m, y=${goal.y.toFixed(2)} m`;
  elements.sendGoalButton.disabled = !(
    state.ros === "connected"
    && isRosConnected()
    && !state.autonomyArmPending
    && !state.navGoalSendPending
  );
  elements.clearGoalButton.disabled = false;
}

function readRuntimeOptions() {
  return {
    motion_serial_enabled: elements.motionSerialToggle.checked,
    show_obstacle_fill: elements.obstacleFillToggle.checked,
    show_roi_polygons: elements.roiBoxToggle.checked,
    show_rgb_debug_text: elements.rgbDebugTextToggle.checked,
  };
}

function publishRuntimeOptions() {
  const nextOptions = readRuntimeOptions();
  if (!publishers.available()) {
    updateModeUi();
    showMessage("ROSBridge 未连接，运行开关没有发送。", "warning");
    return;
  }
  state.runtimeOptions = nextOptions;
  publishers.publishRuntimeOptions(nextOptions);
  elements.statusCommand.textContent = nextOptions.motion_serial_enabled ? "Motion TX On" : "Motion TX Off";
  updateMotionAvailability();
  addEvent(`运行开关已更新 · 运动串口${nextOptions.motion_serial_enabled ? "开" : "关"}`);
}

function publishPreviewGoal(goal = state.selectedGoal) {
  if (!goal || !(state.ros === "connected" && isRosConnected())) return;
  if (publishers.publishPreviewGoal(goal)) {
    addEvent(`请求路径预览 x=${goal.x.toFixed(2)}, y=${goal.y.toFixed(2)}`);
  }
}

function clearSelectedGoal() {
  mapRenderer.clearGoal();
  if (state.ros === "connected" && isRosConnected()) publishers.publishClearPreviewGoal();
  showMessage("已清除导航标记和预览路径。", "success");
}

const mapRenderer = new MapRenderer({
  canvas: $("#mapCanvas"),
  viewport: $("#mapViewport"),
  placeholder: $("#mapPlaceholder"),
  onMetaChange(meta) {
    if (meta.resolution) elements.mapResolution.textContent = meta.resolution;
    if (meta.size) elements.mapSize.textContent = meta.size;
    if (meta.updated) elements.mapUpdated.textContent = meta.updated;
    if (meta.zoom) elements.mapZoom.textContent = meta.zoom;
  },
  onGoalSelected(goal) {
    updateGoalUi(goal);
    if (goal) {
      addEvent(`Selected goal x=${goal.x.toFixed(2)}, y=${goal.y.toFixed(2)}`);
      publishPreviewGoal(goal);
    }
  },
});

const publishers = new RobotPublishers();

const gearController = new GearController({
  publishers,
  onChange(gear, config, { publish }) {
    const profile = DRIVE_PROFILES[config.profile];
    elements.ledLamp.style.setProperty("--led-color", config.ledColorHex);
    elements.ledLamp.setAttribute("aria-label", `RGB LED ${config.ledColorText}`);
    elements.ledColorText.textContent = `${config.ledColorName} / ${config.ledColorText}`;
    elements.gearText.textContent = config.label;
    elements.multiplierText.textContent = config.shortLabel || `x${config.multiplier}`;
    elements.speedText.textContent = config.speedText;
    elements.driveProfileText.textContent = profile.label;
    elements.statusGear.textContent = config.label;
    elements.statusLed.textContent = config.ledColorName;
    elements.statusSpeed.textContent = config.speedText;
    if (publish) {
      elements.statusCommand.textContent = "Gear Change";
      addEvent(`切换至 ${config.label} · ${config.speedText}`);
    }
  },
  onUnavailable() {
    showMessage("挡位只更新在当前页面，ROS 命令没有发送。", "warning");
  },
});

function setCommand(command, label) {
  elements.statusCommand.textContent = label;
  if (["stop", "emergency_stop", "reset_estop"].includes(command)) {
    addEvent(label);
    return;
  }
  const now = Date.now();
  if (now - state.lastMotionEvent > 900) {
    addEvent(label);
    state.lastMotionEvent = now;
  }
}

const motionController = new MotionController({
  buttons: $$(".motion-button"),
  publishers,
  getGear: () => ({ gear: gearController.gear, config: gearController.config }),
  onCommand: setCommand,
});

function setEstopUi(active) {
  state.estop = active;
  if (active && pendingAutonomyArm) clearAutonomyArm();
  document.body.classList.toggle("estop-active", active);
  setStatusValue(elements.statusEstop, active ? "Active" : "Inactive", active ? "danger" : "online");
  updateMotionAvailability();
}

const estopController = new EstopController({
  publishers,
  getGear: () => ({ gear: gearController.gear, config: gearController.config }),
  cancelMotion: () => motionController.cancel(),
  onCommand: setCommand,
  onChange: setEstopUi,
});

const subscribers = new RobotSubscribers({
  onMap(map) {
    mapRenderer.setMap(map);
    setStatusValue(elements.statusMap, "Received", "online");
    elements.fitMapButton.disabled = false;
    elements.resetMapButton.disabled = false;
  },
  onPose(pose) {
    mapRenderer.setPose(pose);
    setStatusValue(elements.statusPose, "Received", "online");
  },
  onPreviewPath(path) { mapRenderer.setPreviewPath(path); },
  onNavPlan(path) { mapRenderer.setNavPath(path); },
  onSerialDebug(data) { handleSerialDebug(data); },
  onSerialDebugError() { addEvent("/robot/serial_debug JSON 解析失败"); },
  onControlState(data) { applyControlState(data); },
  onControlStateError() { addEvent("/robot/control_state JSON 解析失败"); },
  onAutoMappingStatus(data) { handleAutoMappingStatus(data); },
  onAutoMappingStatusError() { addEvent("/auto_mapping/status JSON 解析失败"); },
  onBaselineReady(ready) { handleBaselineReady(ready); },
  onStatus(status) {
    elements.robotMode.textContent = status.mode ?? "—";
    elements.robotState.textContent = status.state ?? "—";
    elements.robotBattery.textContent =
      typeof status.battery_voltage === "number" ? `${status.battery_voltage.toFixed(1)} V` : "—";
    elements.robotMessage.textContent = status.message || "已收到机器人状态";
    if (GEAR_CONFIG[status.gear] && !state.mappingMode) {
      gearController.setProfile("normal");
      gearController.setGear(status.gear);
    }
    if (typeof status.estop === "boolean") setEstopUi(status.estop);
  },
  onStatusError() { addEvent("/robot/status JSON 解析失败"); },
});

function setRosState(connectionState, error = null) {
  state.ros = connectionState;
  const connected = connectionState === "connected";
  if (!connected && connectionState !== "connecting") {
    if (pendingAutonomyArm) clearAutonomyArm();
    state.autoMappingPending = false;
    if (state.autoMappingRequestTimer) {
      window.clearTimeout(state.autoMappingRequestTimer);
      state.autoMappingRequestTimer = null;
    }
  }
  const summary = {
    connecting: ["ROS 连接中", "is-waiting"],
    connected: ["ROS 已连接", "is-online"],
    error: ["ROS 连接失败", "is-offline"],
    disconnected: ["ROS 离线", "is-offline"],
  }[connectionState];
  setSummary(elements.rosSummary, "radio", summary[0], summary[1]);
  setStatusValue(
    elements.statusRos,
    connected ? "Connected" : connectionState === "connecting" ? "Connecting" : "Disconnected",
    connected ? "online" : connectionState === "connecting" ? "waiting" : "offline",
  );
  if (connected) {
    addEvent("ROS Bridge 已连接，等待机器人共享状态");
    showMessage("ROSBridge 连接成功。", "success");
  } else if (connectionState === "error") {
    const detail = error?.message || error?.type || "";
    addEvent("ROS Bridge 连接失败");
    showMessage(`ROSBridge 连接失败${detail ? `：${detail}` : "。"}`, "error");
  }
  updateMotionAvailability();
  updateGoalUi();
  updateAutoMappingUi();
}

const rosCallbacks = {
  onConnecting() { setRosState("connecting"); },
  onConnection(ros) {
    try {
      publishers.initialize(ros);
      subscribers.initialize(ros);
      state.controlStateSeen = false;
      setRosState("connected");
      publishPreviewGoal();
    } catch (error) {
      console.error("[ROS] initialization error", error);
      publishers.clear();
      subscribers.clear();
      setRosState("error", error);
    }
  },
  onError(error) {
    publishers.clear();
    subscribers.clear();
    setRosState("error", error);
  },
  onClose() {
    publishers.clear();
    subscribers.clear();
    state.preferRosTx = false;
    state.controlStateSeen = false;
    setRosState("disconnected");
  },
};

const AUTONOMY_ARM_TIMEOUT_MS = 3000;
let pendingAutonomyArm = null;
let autonomyArmTimer = null;

function autonomousControlReady() {
  return publishers.available()
    && state.baselineReady
    && !state.estop
    && state.controlMode === "move"
    && state.runtimeOptions.motion_serial_enabled
    && !state.echoEnabled;
}

function clearAutonomyArm() {
  if (autonomyArmTimer) window.clearInterval(autonomyArmTimer);
  autonomyArmTimer = null;
  pendingAutonomyArm = null;
  state.autonomyArmPending = false;
  updateGoalUi();
  updateAutoMappingUi();
}

function tryCompleteAutonomyArm() {
  const pending = pendingAutonomyArm;
  if (!pending) return;
  if (autonomousControlReady()) {
    const run = pending.run;
    const label = pending.label;
    clearAutonomyArm();
    addEvent(`${label}已取得 MOVE 控制权`);
    run();
    return;
  }
  if (Date.now() >= pending.deadline) {
    const label = pending.label;
    clearAutonomyArm();
    showMessage(
      `${label}未启动：上位机未在 3 秒内确认 MOVE 帧已写入串口，请检查 TX。`,
      "error",
    );
  }
}

function requestAutonomousControl(label, run) {
  if (!publishers.available()) {
    showMessage("ROSBridge 未连接，无法启动自动运动。", "warning");
    return false;
  }
  if (!state.baselineReady) {
    showMessage(`请先采集深度 Baseline，再启动${label}。`, "warning");
    return false;
  }
  if (state.estop) {
    showMessage(`急停仍处于激活状态，无法启动${label}。`, "error");
    return false;
  }
  if (autonomousControlReady()) {
    run();
    return true;
  }

  clearAutonomyArm();
  pendingAutonomyArm = {
    label,
    run,
    deadline: Date.now() + AUTONOMY_ARM_TIMEOUT_MS,
  };
  state.autonomyArmPending = true;
  updateGoalUi();
  updateAutoMappingUi();
  if (state.echoEnabled && !sendSerialCommand("echo_off")) {
    clearAutonomyArm();
    return false;
  }
  if (!sendSerialCommand("enable_move")) {
    clearAutonomyArm();
    return false;
  }
  showMessage(`正在请求 MOVE 接管，确认后自动启动${label}。`, "success");
  autonomyArmTimer = window.setInterval(tryCompleteAutonomyArm, 100);
  return true;
}

function publishSelectedGoal(goal) {
  if (state.navGoalSendPending) return;
  state.navGoalSendPending = true;
  updateGoalUi();
  if (!publishers.publishGoalPose(goal)) {
    state.navGoalSendPending = false;
    updateGoalUi();
    showMessage("导航目标发送失败。", "error");
    return;
  }
  elements.statusCommand.textContent = "Nav Goal";
  addEvent(`发送导航目标 x=${goal.x.toFixed(2)}, y=${goal.y.toFixed(2)}`);
  showMessage("导航目标已发送；路径与避障继续由融合节点实时更新。", "success");
  window.setTimeout(() => {
    state.navGoalSendPending = false;
    updateGoalUi();
  }, 800);
}

function sendSelectedGoal() {
  const selected = state.selectedGoal;
  if (!selected) return showMessage("请先在地图上点击一个导航目标点。", "warning");
  const goal = {
    ...selected,
    orientation: selected.orientation ? { ...selected.orientation } : undefined,
  };
  requestAutonomousControl("导航", () => publishSelectedGoal(goal));
}

function publishAutoMappingRequest(enable) {
  if (!publishers.publishAutoMappingControl(enable)) {
    showMessage("自动建图命令发送失败。", "error");
    return;
  }

  state.autoMappingPending = true;
  updateAutoMappingUi();
  elements.statusCommand.textContent = enable ? "Auto Mapping On" : "Auto Mapping Off";
  addEvent(`请求${enable ? "开启" : "关闭"}自动建图`);
  showMessage(
    enable
      ? "正在开启自动建图，机器人将由 frontier 与 Nav2 自动探索。"
      : "正在关闭自动建图，并取消当前自动探索目标。",
    "success",
  );

  if (state.autoMappingRequestTimer) window.clearTimeout(state.autoMappingRequestTimer);
  state.autoMappingRequestTimer = window.setTimeout(() => {
    state.autoMappingPending = false;
    state.autoMappingRequestTimer = null;
    updateAutoMappingUi();
    showMessage("未收到自动建图状态回执，请检查 frontier_web_bridge 和 frontier_explorer。", "warning");
  }, 3500);
}

function toggleAutoMapping() {
  if (!publishers.available()) {
    return showMessage("ROSBridge 未连接，无法控制自动建图。", "warning");
  }
  const enable = !state.autoMappingEnabled;
  if (enable) {
    requestAutonomousControl("自动建图", () => publishAutoMappingRequest(true));
  } else {
    publishAutoMappingRequest(false);
  }
}

function sendSerialCommand(action) {
  if (!publishers.available()) {
    showMessage("ROSBridge 未连接，无法发送 STM32 串口命令。", "warning");
    return false;
  }
  if (!publishers.publishSerialCommand(action)) {
    showMessage("STM32 串口命令发送失败。", "error");
    return false;
  }
  const labels = {
    enable_move: "MOVE 接管 0x01",
    zero_move: "零速 MOVE 0x01",
    stop: "STOP 0x02",
    estop: "ESTOP 0x03",
    ps2: "归还 PS2 0x04",
    echo_on: "回响 ON 0x05",
    echo_off: "回响 OFF 0x06",
  };
  elements.statusCommand.textContent = labels[action] || action;
  addEvent(`STM32 ${labels[action] || action}`);
  return true;
}

function markTogglePending(button) {
  button.classList.add("is-pending");
  button.disabled = true;
  window.setTimeout(() => {
    button.classList.remove("is-pending");
    updateMotionAvailability();
  }, 700);
}

function toggleControlMode() {
  if (state.controlMode === "move" && !state.baselineReady) {
    showMessage("必须先采集深度 Baseline，才能把控制权归还 PS2。", "warning");
    return;
  }
  const action = state.controlMode === "move" ? "ps2" : "enable_move";
  if (sendSerialCommand(action)) markTogglePending(elements.controlModeToggle);
}

function toggleMappingMode() {
  if (!publishers.available()) {
    showMessage("ROSBridge 未连接，无法切换网页速度模式。", "warning");
    return;
  }
  const nextProfile = state.mappingMode ? "normal" : "mapping";
  gearController.setProfile(nextProfile);
  gearController.setGear(1, { publish: true });
  elements.statusCommand.textContent = nextProfile === "mapping" ? "建图速度模式" : "常规速度模式";
  addEvent(`网页速度切换至 ${DRIVE_PROFILES[nextProfile].label}`);
  markTogglePending(elements.mappingModeToggle);
}

function toggleEchoMode() {
  const action = state.echoEnabled ? "echo_off" : "echo_on";
  if (sendSerialCommand(action)) markTogglePending(elements.echoModeToggle);
}

function updateThemeButton() {
  const isDark = document.documentElement.dataset.theme === "dark";
  elements.themeToggle.title = isDark ? "切换到亮色模式" : "切换到暗色模式";
  elements.themeToggle.setAttribute("aria-label", elements.themeToggle.title);
  replaceIcon(elements.themeToggle, isDark ? "sun" : "moon");
}

function toggleTheme(event) {
  const nextTheme = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
  const apply = () => {
    document.documentElement.dataset.theme = nextTheme;
    localStorage.setItem(STORAGE.theme, nextTheme);
    updateThemeButton();
  };
  const reducedMotion = window.matchMedia?.("(prefers-reduced-motion: reduce)").matches;
  if (!document.startViewTransition || reducedMotion) {
    apply();
    return;
  }
  const rect = event.currentTarget.getBoundingClientRect();
  const x = rect.left + rect.width / 2;
  const y = rect.top + rect.height / 2;
  const radius = Math.hypot(Math.max(x, innerWidth - x), Math.max(y, innerHeight - y));
  const transition = document.startViewTransition(apply);
  transition.ready.then(() => {
    document.documentElement.animate(
      { clipPath: [`circle(0 at ${x}px ${y}px)`, `circle(${radius}px at ${x}px ${y}px)`] },
      { duration: 620, easing: "cubic-bezier(.2,.8,.2,1)", pseudoElement: "::view-transition-new(root)" },
    );
  });
}

function setPanelFocus(panel, button) {
  const opening = state.focusedPanel !== panel;
  if (state.focusedPanel) state.focusedPanel.classList.remove("is-focus");
  state.focusedPanel = opening ? panel : null;
  document.body.classList.toggle("focus-mode", opening);
  if (opening) panel.classList.add("is-focus");
  replaceIcon(button, opening ? "minimize-2" : "maximize-2");
  button.title = opening ? "退出放大" : panel === elements.videoPanel ? "放大 RGB 画面" : "放大地图";
  button.setAttribute("aria-label", button.title);
  if (!opening) {
    replaceIcon(elements.videoExpandButton, "maximize-2");
    replaceIcon(elements.mapExpandButton, "maximize-2");
    elements.videoExpandButton.setAttribute("aria-label", elements.videoExpandButton.title);
    elements.mapExpandButton.setAttribute("aria-label", elements.mapExpandButton.title);
  }
  window.setTimeout(() => mapRenderer.resize(), 280);
}

function closePanelFocus() {
  if (!state.focusedPanel) return;
  const button = state.focusedPanel === elements.videoPanel
    ? elements.videoExpandButton
    : elements.mapExpandButton;
  setPanelFocus(state.focusedPanel, button);
}

function selectTelemetryTab(tab) {
  elements.tabButtons.forEach((button) => {
    const selected = button.dataset.tab === tab;
    button.classList.toggle("is-active", selected);
    button.setAttribute("aria-selected", String(selected));
  });
  elements.tabPanes.forEach((pane) => pane.classList.toggle("is-active", pane.dataset.pane === tab));
}

const videoStream = new VideoStream({
  image: $("#videoStream"),
  frame: $("#videoFrame"),
  placeholderText: $("#videoPlaceholderText"),
  onStateChange(videoState) {
    state.video = videoState;
    const summary = {
      disconnected: ["视频离线", "is-offline"],
      connecting: ["视频连接中", "is-waiting"],
      receiving: ["视频接收中", "is-online"],
      failed: ["视频连接失败", "is-offline"],
    }[videoState];
    const panelLabels = {
      disconnected: "Disconnected",
      connecting: "Connecting",
      receiving: "Receiving stream",
      failed: "Connection failed",
    };
    setSummary(elements.videoSummary, "camera", summary[0], summary[1]);
    elements.videoPanelStatus.textContent = panelLabels[videoState];
    setStatusValue(
      elements.statusVideo,
      videoState === "receiving" ? "Receiving" : videoState === "failed" ? "Failed" : "Disconnected",
      videoState === "receiving" ? "online" : videoState === "failed" ? "danger" : "offline",
    );
  },
});

elements.robotIp.addEventListener("input", () => {
  const result = normalizeRobotIp(elements.robotIp.value);
  if (result.error) return;
  fillConnectionUrls(result.value);
  saveSettings();
  showMessage("已根据机器人 IP 生成视频与 ROSBridge 地址。", "success");
});
elements.robotIp.addEventListener("change", () => {
  const result = normalizeRobotIp(elements.robotIp.value);
  if (result.error) showMessage(result.error, "error");
});
elements.videoUrl.addEventListener("change", saveSettings);
elements.rosUrl.addEventListener("change", saveSettings);

$("#connectVideoButton").addEventListener("click", () => {
  const result = validateVideoUrl(elements.videoUrl.value);
  if (result.error) return showMessage(result.error, "error");
  elements.videoUrl.value = result.value;
  elements.activeVideoUrl.textContent = result.value;
  elements.activeVideoUrl.title = result.value;
  saveSettings();
  videoStream.connect(result.value);
  startTxFramePolling(result.value);
  addEvent("正在连接视频流");
});

$("#connectRosButton").addEventListener("click", () => {
  const result = validateRosUrl(elements.rosUrl.value);
  if (result.error) return showMessage(result.error, "error");
  let normalizedUrl;
  try {
    normalizedUrl = normalizeRosbridgeUrl(result.value);
  } catch (error) {
    return showMessage(error.message, "error");
  }
  elements.rosUrl.value = normalizedUrl;
  saveSettings();
  subscribers.clear({ sendUnsubscribe: true });
  publishers.clear();
  connectRosbridge(normalizedUrl, rosCallbacks);
  addEvent("正在连接 ROS Bridge");
});

$("#disconnectButton").addEventListener("click", () => {
  videoStream.disconnect();
  stopTxFramePolling();
  setTxFrameWaiting();
  subscribers.clear({ sendUnsubscribe: true });
  publishers.clear();
  disconnectRosbridge();
  elements.activeVideoUrl.textContent = "—";
  showMessage("视频与 ROSBridge 已断开。");
  addEvent("全部连接已断开");
});

$("#gearButton").addEventListener("click", () => gearController.cycle());
elements.estopButton.addEventListener("click", () => estopController.activate());
elements.resetEstopButton.addEventListener("click", () => estopController.reset());
elements.fitMapButton.addEventListener("click", () => mapRenderer.fit());
elements.resetMapButton.addEventListener("click", () => mapRenderer.reset());
elements.baselineCaptureButton.addEventListener("click", () => {
  if (!publishers.available()) return showMessage("ROSBridge 未连接，无法采集 Baseline。", "warning");
  state.baselineReady = false;
  updateBaselineUi();
  updateMotionAvailability();
  publishers.publishBaselineCapture();
  addEvent("开始采集深度 Baseline");
  showMessage("正在采集深度 Baseline；完成前运动与 PS2 保持锁定。", "success");
});
elements.sendGoalButton.addEventListener("click", sendSelectedGoal);
elements.clearGoalButton.addEventListener("click", clearSelectedGoal);
elements.autoMappingToggle.addEventListener("click", toggleAutoMapping);
elements.serialCommandButtons.forEach((button) => {
  button.addEventListener("click", () => sendSerialCommand(button.dataset.serialAction));
});
elements.controlModeToggle.addEventListener("click", toggleControlMode);
elements.mappingModeToggle.addEventListener("click", toggleMappingMode);
elements.echoModeToggle.addEventListener("click", toggleEchoMode);
elements.themeToggle.addEventListener("click", toggleTheme);
elements.videoExpandButton.addEventListener("click", () => setPanelFocus(elements.videoPanel, elements.videoExpandButton));
elements.mapExpandButton.addEventListener("click", () => setPanelFocus(elements.mapPanel, elements.mapExpandButton));
elements.tabButtons.forEach((button) => button.addEventListener("click", () => selectTelemetryTab(button.dataset.tab)));
window.addEventListener("keydown", (event) => { if (event.key === "Escape") closePanelFocus(); });

elements.slamLogToggle.addEventListener("click", () => {
  if (!publishers.available()) return showMessage("ROSBridge 未连接，无法控制 SLAM 日志。", "warning");
  state.slamLogEnabled = !state.slamLogEnabled;
  updateModeUi();
  publishers.publishSlamLogControl(state.slamLogEnabled, state.slamLogIntervalSec);
  addEvent(`SLAM 日志 ${state.slamLogEnabled ? `开启（${state.slamLogIntervalSec} 秒）` : "关闭"}`);
});

function commitSlamLogInterval() {
  const interval = normalizeSlamLogInterval(elements.slamLogInterval.value);
  if (interval === null) {
    elements.slamLogInterval.value = String(state.slamLogIntervalSec);
    return showMessage("SLAM 日志间隔必须是 0.5–60 秒的数值。", "warning");
  }
  state.slamLogIntervalSec = interval;
  elements.slamLogInterval.value = String(interval);
  if (!publishers.available()) {
    return showMessage("ROSBridge 未连接，日志间隔尚未发送。", "warning");
  }
  publishers.publishSlamLogConfig(interval);
  addEvent(`SLAM 日志间隔设为 ${interval} 秒${state.slamLogEnabled ? "（已生效）" : "（日志仍关闭）"}`);
  showMessage(`SLAM 日志记录间隔已设为 ${interval} 秒。`, "success");
}

elements.slamLogInterval.addEventListener("change", commitSlamLogInterval);
elements.slamLogInterval.addEventListener("keydown", (event) => {
  if (event.key === "Enter") {
    event.preventDefault();
    elements.slamLogInterval.blur();
  }
});

[
  elements.motionSerialToggle,
  elements.obstacleFillToggle,
  elements.roiBoxToggle,
  elements.rgbDebugTextToggle,
].forEach((element) => element.addEventListener("change", publishRuntimeOptions));

loadSettings();
gearController.setProfile("mapping");
gearController.setGear(1);
setEstopUi(false);
updateModeUi();
updateThemeButton();
selectTelemetryTab("tx");
videoStream.disconnect();
refreshIcons();

const demoStreamUrl = import.meta.env.VITE_DEMO_STREAM_URL;
if (demoStreamUrl) {
  elements.videoUrl.value = demoStreamUrl;
  elements.activeVideoUrl.textContent = demoStreamUrl;
  elements.activeVideoUrl.title = demoStreamUrl;
  videoStream.connect(demoStreamUrl);
  startTxFramePolling(demoStreamUrl);
}
