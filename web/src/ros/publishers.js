// 使用 main.js 传入的唯一已连接 ROS 实例创建并管理全部发布器。
import { ROSLIB } from "./rosConnection.js";
import { ROS_TOPICS } from "./topics.js";
import { nowTimestampMs } from "../utils/time.js";

const ZERO_VECTOR = { x: 0, y: 0, z: 0 };
const PUBLISHER_KEYS = ["gear", "webControl", "emergencyStop", "cmdVel", "goalPose", "previewGoal"];

const MOTION_TO_TWIST = Object.freeze({
  forward: { linear: 1, angular: 0 },
  backward: { linear: -1, angular: 0 },
  turn_left: { linear: 0, angular: 1 },
  turn_right: { linear: 0, angular: -1 },
  forward_left: { linear: 1, angular: 1, arc: true },
  forward_right: { linear: 1, angular: -1, arc: true },
  backward_left: { linear: -1, angular: -1, arc: true },
  backward_right: { linear: -1, angular: 1, arc: true },
  stop: { linear: 0, angular: 0 },
});

export function createPublishers(ros) {
  if (!ros) throw new Error("createPublishers 需要已连接的 ROS 实例。");
  return Object.fromEntries(
    PUBLISHER_KEYS.map((key) => [
      key,
      new ROSLIB.Topic({ ros, ...ROS_TOPICS[key] }),
    ]),
  );
}

export class RobotPublishers {
  constructor() {
    this.ros = null;
    this.publishers = {};
  }

  initialize(ros) {
    this.ros = ros;
    this.publishers = createPublishers(ros);
  }

  clear() {
    this.ros = null;
    this.publishers = {};
  }

  available() {
    return this.ros?.isConnected === true && Boolean(this.publishers.webControl);
  }

  publishGear(gear) {
    return this.publish("gear", { data: gear });
  }

  publishWebControl(command, gearConfig, gear) {
    const zeroSpeed = command === "stop" || command === "emergency_stop";
    const payload = {
      source: "web_console",
      command,
      gear,
      led_color: gearConfig.ledColorName,
      speed_cnt_per_sec: zeroSpeed ? 0 : gearConfig.speedCntPerSec,
      multiplier: gearConfig.multiplier,
      profile: gearConfig.profile,
      timestamp_ms: nowTimestampMs(),
    };
    this.publish("webControl", { data: JSON.stringify(payload) });
    return payload;
  }

  publishRuntimeOptions(options) {
    return this.publish("webControl", {
      data: JSON.stringify({
        source: "web_console",
        command: "runtime_options",
        timestamp_ms: nowTimestampMs(),
        ...options,
      }),
    });
  }

  publishClearPreviewGoal() {
    return this.publish("webControl", {
      data: JSON.stringify({
        source: "web_console",
        command: "clear_preview_goal",
        timestamp_ms: nowTimestampMs(),
      }),
    });
  }

  publishAutoMappingControl(enabled) {
    return this.publish("webControl", {
      data: JSON.stringify({
        source: "web_console",
        command: enabled ? "auto_mapping_start" : "auto_mapping_stop",
        timestamp_ms: nowTimestampMs(),
      }),
    });
  }

  publishSerialCommand(action) {
    return this.publish("webControl", {
      data: JSON.stringify({
        source: "web_console",
        command: "serial_command",
        action,
        timestamp_ms: nowTimestampMs(),
      }),
    });
  }

  publishBaselineCapture() {
    return this.publish("webControl", {
      data: JSON.stringify({
        source: "web_console",
        command: "baseline_capture",
        timestamp_ms: nowTimestampMs(),
      }),
    });
  }

  publishSlamLogControl(enabled, intervalSec) {
    return this.publish("webControl", {
      data: JSON.stringify({
        source: "web_console",
        command: enabled ? "slam_log_enable" : "slam_log_disable",
        interval_sec: intervalSec,
        timestamp_ms: nowTimestampMs(),
      }),
    });
  }

  publishSlamLogConfig(intervalSec) {
    return this.publish("webControl", {
      data: JSON.stringify({
        source: "web_console",
        command: "slam_log_config",
        interval_sec: intervalSec,
        timestamp_ms: nowTimestampMs(),
      }),
    });
  }

  publishEmergencyStop(active) {
    return this.publish("emergencyStop", { data: active });
  }

  publishCmdVel(command, gearConfig) {
    const motion = MOTION_TO_TWIST[command] || MOTION_TO_TWIST.stop;
    const linearBase = motion.arc ? gearConfig.cmdVel.arcLinear : gearConfig.cmdVel.linear;
    const linearX = motion.linear * linearBase;
    const angularBase = motion.arc ? gearConfig.cmdVel.arcAngular : gearConfig.cmdVel.angular;
    const angularZ = motion.angular * angularBase;

    return this.publish("cmdVel", {
      linear: { ...ZERO_VECTOR, x: linearX },
      angular: { ...ZERO_VECTOR, z: angularZ },
    });
  }

  publishGoalPose(goal) {
    const stampSeconds = Math.floor(Date.now() / 1000);
    const stampNano = (Date.now() % 1000) * 1000000;
    return this.publish("goalPose", {
      header: {
        frame_id: "map",
        stamp: { sec: stampSeconds, nanosec: stampNano },
      },
      pose: {
        position: { x: goal.x, y: goal.y, z: 0 },
        orientation: goal.orientation || { x: 0, y: 0, z: 0, w: 1 },
      },
    });
  }

  publishPreviewGoal(goal) {
    const stampSeconds = Math.floor(Date.now() / 1000);
    const stampNano = (Date.now() % 1000) * 1000000;
    return this.publish("previewGoal", {
      header: {
        frame_id: "map",
        stamp: { sec: stampSeconds, nanosec: stampNano },
      },
      pose: {
        position: { x: goal.x, y: goal.y, z: 0 },
        orientation: goal.orientation || { x: 0, y: 0, z: 0, w: 1 },
      },
    });
  }

  publish(key, data) {
    const publisher = this.publishers[key];
    if (!this.available() || !publisher) {
      console.warn(`[ROS] publish skipped: ${key} is unavailable`);
      return false;
    }
    publisher.publish(new ROSLIB.Message(data));
    return true;
  }
}
