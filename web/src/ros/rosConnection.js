// 维护全站唯一的 roslibjs 连接实例，并隔离旧连接的异步事件。
import * as RoslibNamespace from "roslib";

// roslib 1.x 是 CommonJS 包。Vite 通常提供命名导出；某些构建配置则放在 default。
// 这里同时兼容两种形态，但全项目只从本模块取得同一个 ROSLIB 对象。
export const ROSLIB = RoslibNamespace.Ros
  ? RoslibNamespace
  : RoslibNamespace.default;

let activeRos = null;
let activeCallbacks = {};
let connectionGeneration = 0;
let connectionState = "disconnected";

function callSafely(callback, ...args) {
  if (typeof callback === "function") callback(...args);
}

export function normalizeRosbridgeUrl(value) {
  const parsed = new URL(String(value || "").trim());
  if (!["ws:", "wss:"].includes(parsed.protocol) || !parsed.hostname) {
    throw new Error("ROSBridge URL 必须是有效的 WS/WSS 地址。");
  }

  // 根路径的一个或多个尾斜杠对 rosbridge 没有意义。查询参数和非根路径保持不变。
  if (/^\/+$/.test(parsed.pathname)) parsed.pathname = "";
  const normalized = parsed.toString();
  return parsed.pathname === "/" && !parsed.search && !parsed.hash
    ? normalized.replace(/\/$/, "")
    : normalized;
}

function closeCurrentConnection({ notify = false } = {}) {
  const ros = activeRos;
  const callbacks = activeCallbacks;
  if (!ros) {
    connectionState = "disconnected";
    if (notify) callSafely(callbacks.onClose);
    return;
  }

  // 先让旧实例失效，再 close。这样它稍后到达的 close/error 不会污染新连接。
  connectionGeneration += 1;
  activeRos = null;
  activeCallbacks = {};
  connectionState = "disconnected";

  try {
    ros.close();
  } catch (error) {
    console.warn("[ROS] close ignored", error);
  }

  console.warn("[ROS] closed");
  if (notify) callSafely(callbacks.onClose);
}

export function connectRosbridge(url, callbacks = {}) {
  const normalizedUrl = normalizeRosbridgeUrl(url);
  closeCurrentConnection({ notify: false });

  const generation = ++connectionGeneration;
  activeCallbacks = callbacks;
  connectionState = "connecting";
  console.info(`[ROS] connecting to ${normalizedUrl}`);
  callSafely(callbacks.onConnecting, normalizedUrl);

  let ros;
  try {
    // 按 roslibjs 官方连接方式创建实例；连接结果只能由异步事件决定。
    ros = new ROSLIB.Ros({ url: normalizedUrl });
    activeRos = ros;
  } catch (error) {
    connectionState = "error";
    console.error("[ROS] error", error);
    callSafely(callbacks.onError, error);
    return null;
  }

  const isCurrent = () =>
    generation === connectionGeneration && ros === activeRos;

  ros.on("connection", () => {
    if (!isCurrent()) return;
    connectionState = "connected";
    console.info("[ROS] connected");
    callSafely(callbacks.onConnection, ros, normalizedUrl);
  });

  ros.on("error", (error) => {
    if (!isCurrent()) return;
    connectionState = "error";
    console.error("[ROS] error", error);
    callSafely(callbacks.onError, error);
  });

  ros.on("close", () => {
    if (!isCurrent()) return;
    activeRos = null;
    activeCallbacks = {};
    connectionState = "disconnected";
    console.warn("[ROS] closed");
    callSafely(callbacks.onClose);
  });

  return ros;
}

export function disconnectRosbridge() {
  closeCurrentConnection({ notify: true });
}

export function getRos() {
  return activeRos;
}

export function isRosConnected() {
  return connectionState === "connected" && activeRos?.isConnected === true;
}

export function getRosConnectionState() {
  return connectionState;
}
