export function quaternionToYaw(quaternion = {}) {
  const { x = 0, y = 0, z = 0, w = 1 } = quaternion;
  const siny = 2 * (w * z + x * y);
  const cosy = 1 - 2 * (y * y + z * z);
  return Math.atan2(siny, cosy);
}

export function yawToQuaternion(yaw = 0) {
  return {
    x: 0,
    y: 0,
    z: Math.sin(yaw / 2),
    w: Math.cos(yaw / 2),
  };
}

export function poseToMapPixel(pose, mapInfo) {
  const world = {
    x: pose.position.x,
    y: pose.position.y,
    yaw: quaternionToYaw(pose.orientation),
  };
  return worldToMapPixel(world, mapInfo);
}

export function worldToMapPixel(world, mapInfo) {
  return {
    x: (world.x - mapInfo.origin.position.x) / mapInfo.resolution,
    y: mapInfo.height - (world.y - mapInfo.origin.position.y) / mapInfo.resolution,
    yaw: world.yaw || 0,
  };
}

export function mapPixelToWorld(pixel, mapInfo) {
  return {
    x: mapInfo.origin.position.x + pixel.x * mapInfo.resolution,
    y: mapInfo.origin.position.y + (mapInfo.height - pixel.y) * mapInfo.resolution,
  };
}

export function drawRobotPose(context, posePixel, displayScale) {
  const size = Math.max(9, 15 / Math.max(displayScale, 0.2));
  context.save();
  context.translate(posePixel.x, posePixel.y);
  context.rotate(-posePixel.yaw);
  context.beginPath();
  context.moveTo(size, 0);
  context.lineTo(-size * 0.65, size * 0.62);
  context.lineTo(-size * 0.35, 0);
  context.lineTo(-size * 0.65, -size * 0.62);
  context.closePath();
  context.fillStyle = "#34d8ff";
  context.strokeStyle = "#071014";
  context.lineWidth = 2 / Math.max(displayScale, 0.2);
  context.fill();
  context.stroke();
  context.restore();
}

export function drawGoalMarker(context, goalPixel, displayScale) {
  const radius = Math.max(7, 12 / Math.max(displayScale, 0.2));
  const lineWidth = 2 / Math.max(displayScale, 0.2);
  context.save();
  context.translate(goalPixel.x, goalPixel.y);
  context.strokeStyle = "#ffcf4a";
  context.fillStyle = "rgba(255, 207, 74, 0.18)";
  context.lineWidth = lineWidth;
  context.beginPath();
  context.arc(0, 0, radius, 0, Math.PI * 2);
  context.fill();
  context.stroke();
  context.beginPath();
  context.moveTo(-radius * 1.35, 0);
  context.lineTo(radius * 1.35, 0);
  context.moveTo(0, -radius * 1.35);
  context.lineTo(0, radius * 1.35);
  context.stroke();
  context.restore();
}

function drawPath(context, pathPixels, displayScale, strokeStyle, haloStyle) {
  if (!Array.isArray(pathPixels) || pathPixels.length < 2) return;
  const lineWidth = Math.max(2, 4 / Math.max(displayScale, 0.2));
  context.save();
  context.lineJoin = "round";
  context.lineCap = "round";
  context.strokeStyle = haloStyle;
  context.lineWidth = lineWidth + 3 / Math.max(displayScale, 0.2);
  context.beginPath();
  context.moveTo(pathPixels[0].x, pathPixels[0].y);
  for (let i = 1; i < pathPixels.length; i += 1) {
    context.lineTo(pathPixels[i].x, pathPixels[i].y);
  }
  context.stroke();
  context.strokeStyle = strokeStyle;
  context.lineWidth = lineWidth;
  context.beginPath();
  context.moveTo(pathPixels[0].x, pathPixels[0].y);
  for (let i = 1; i < pathPixels.length; i += 1) {
    context.lineTo(pathPixels[i].x, pathPixels[i].y);
  }
  context.stroke();
  context.restore();
}

export function drawPreviewPath(context, pathPixels, displayScale) {
  drawPath(context, pathPixels, displayScale, "#42f59b", "rgba(7, 16, 20, 0.85)");
}

export function drawNavPath(context, pathPixels, displayScale) {
  drawPath(context, pathPixels, displayScale, "#34d8ff", "rgba(7, 16, 20, 0.82)");
}
