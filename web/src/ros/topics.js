// Web 控制台使用的全部 ROS2 topic 名称与消息类型。
export const ROS_TOPICS = Object.freeze({
  gear: { name: "/robot/control/gear", messageType: "std_msgs/UInt8" },
  webControl: { name: "/robot/web_control", messageType: "std_msgs/String" },
  emergencyStop: { name: "/robot/emergency_stop", messageType: "std_msgs/Bool" },
  cmdVel: { name: "/cmd_vel_web", messageType: "geometry_msgs/Twist" },
  goalPose: { name: "/web/nav_goal", messageType: "geometry_msgs/PoseStamped" },
  previewGoal: { name: "/web/preview_goal", messageType: "geometry_msgs/PoseStamped" },
  previewPath: { name: "/web/preview_path", messageType: "nav_msgs/Path" },
  navPlan: { name: "/plan", messageType: "nav_msgs/Path" },
  map: { name: "/map", messageType: "nav_msgs/OccupancyGrid" },
  robotPose: { name: "/robot_pose", messageType: "geometry_msgs/PoseStamped" },
  robotStatus: { name: "/robot/status", messageType: "std_msgs/String" },
  serialDebug: { name: "/robot/serial_debug", messageType: "std_msgs/String" },
  controlState: { name: "/robot/control_state", messageType: "std_msgs/String" },
  autoMappingStatus: { name: "/auto_mapping/status", messageType: "std_msgs/String" },
  baselineReady: { name: "/depth/baseline_ready", messageType: "std_msgs/Bool" },
});
