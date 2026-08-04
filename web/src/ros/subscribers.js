// 使用 main.js 传入的唯一已连接 ROS 实例创建并管理全部订阅器。
import { ROSLIB } from "./rosConnection.js";
import { ROS_TOPICS } from "./topics.js";

export function createSubscribers(ros, handlers = {}) {
  if (!ros) throw new Error("createSubscribers 需要已连接的 ROS 实例。");
  const subscriptions = [];

  const subscribe = (key, callback) => {
    if (typeof callback !== "function") return;
    const topic = new ROSLIB.Topic({ ros, ...ROS_TOPICS[key] });
    topic.subscribe(callback);
    subscriptions.push(topic);
  };

  subscribe("map", handlers.onMap);
  subscribe("robotPose", handlers.onPose);
  subscribe("previewPath", handlers.onPreviewPath);
  subscribe("navPlan", handlers.onNavPlan);
  subscribe("serialDebug", (message) => {
    try {
      handlers.onSerialDebug?.(JSON.parse(message.data));
    } catch (error) {
      handlers.onSerialDebugError?.(error);
    }
  });
  subscribe("controlState", (message) => {
    try {
      handlers.onControlState?.(JSON.parse(message.data));
    } catch (error) {
      handlers.onControlStateError?.(error);
    }
  });
  subscribe("autoMappingStatus", (message) => {
    try {
      handlers.onAutoMappingStatus?.(JSON.parse(message.data));
    } catch (error) {
      handlers.onAutoMappingStatusError?.(error);
    }
  });
  subscribe("baselineReady", (message) => handlers.onBaselineReady?.(Boolean(message.data)));
  subscribe("robotStatus", (message) => {
    try {
      handlers.onStatus?.(JSON.parse(message.data));
    } catch (error) {
      handlers.onStatusError?.(error);
    }
  });

  return subscriptions;
}

export class RobotSubscribers {
  constructor(callbacks = {}) {
    this.ros = null;
    this.callbacks = callbacks;
    this.subscriptions = [];
  }

  initialize(ros) {
    this.clear();
    this.ros = ros;
    this.subscriptions = createSubscribers(ros, this.callbacks);
  }

  clear({ sendUnsubscribe = false } = {}) {
    if (sendUnsubscribe && this.ros?.isConnected === true) {
      this.subscriptions.forEach((topic) => topic.unsubscribe());
    }
    this.subscriptions = [];
    this.ros = null;
  }
}
