import { drawGoalMarker, drawNavPath, drawPreviewPath, drawRobotPose, mapPixelToWorld, poseToMapPixel, worldToMapPixel, yawToQuaternion } from "./poseRenderer.js";
import { formatMapUpdate } from "../utils/time.js";

export class MapRenderer {
  constructor({ canvas, viewport, placeholder, onMetaChange = () => {}, onGoalSelected = () => {} }) {
    this.canvas = canvas;
    this.viewport = viewport;
    this.placeholder = placeholder;
    this.context = canvas.getContext("2d");
    this.mapCanvas = document.createElement("canvas");
    this.mapContext = this.mapCanvas.getContext("2d");
    this.map = null;
    this.pose = null;
    this.goal = null;
    this.previewPath = null;
    this.navPath = null;
    this.scale = 1;
    this.offsetX = 0;
    this.offsetY = 0;
    this.drag = null;
    this.didInitialFit = false;
    this.onMetaChange = onMetaChange;
    this.onGoalSelected = onGoalSelected;
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(viewport);
    this.bind();
  }

  bind() {
    this.viewport.addEventListener(
      "wheel",
      (event) => {
        if (!this.map) return;
        event.preventDefault();
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;
        const factor = event.deltaY < 0 ? 1.12 : 0.89;
        const nextScale = Math.min(20, Math.max(0.05, this.scale * factor));
        this.offsetX = mouseX - ((mouseX - this.offsetX) * nextScale) / this.scale;
        this.offsetY = mouseY - ((mouseY - this.offsetY) * nextScale) / this.scale;
        this.scale = nextScale;
        this.draw();
      },
      { passive: false },
    );

    this.viewport.addEventListener("pointerdown", (event) => {
      if (!this.map || event.button !== 0) return;
      this.drag = {
        x: event.clientX,
        y: event.clientY,
        offsetX: this.offsetX,
        offsetY: this.offsetY,
        moved: false,
      };
      this.viewport.setPointerCapture?.(event.pointerId);
      this.viewport.classList.add("is-dragging");
    });

    this.viewport.addEventListener("pointermove", (event) => {
      if (!this.drag) return;
      const dx = event.clientX - this.drag.x;
      const dy = event.clientY - this.drag.y;
      if (Math.hypot(dx, dy) > 5) this.drag.moved = true;
      this.offsetX = this.drag.offsetX + dx;
      this.offsetY = this.drag.offsetY + dy;
      this.draw();
    });

    const stopDrag = (event) => {
      const wasClick = this.drag && !this.drag.moved;
      this.drag = null;
      this.viewport.classList.remove("is-dragging");
      if (wasClick) this.selectGoalFromEvent(event);
    };
    this.viewport.addEventListener("pointerup", stopDrag);
    this.viewport.addEventListener("pointercancel", () => {
      this.drag = null;
      this.viewport.classList.remove("is-dragging");
    });
  }

  setMap(map) {
    if (!map?.info || !Array.isArray(map.data)) return;
    this.map = map;
    this.mapCanvas.width = map.info.width;
    this.mapCanvas.height = map.info.height;
    const image = this.mapContext.createImageData(map.info.width, map.info.height);

    for (let sourceY = 0; sourceY < map.info.height; sourceY += 1) {
      const canvasY = map.info.height - 1 - sourceY;
      for (let x = 0; x < map.info.width; x += 1) {
        const value = map.data[sourceY * map.info.width + x];
        const gray = value < 0 ? 104 : value === 0 ? 226 : Math.round(226 * (1 - value / 100));
        const index = (canvasY * map.info.width + x) * 4;
        image.data[index] = gray;
        image.data[index + 1] = gray;
        image.data[index + 2] = gray;
        image.data[index + 3] = 255;
      }
    }
    this.mapContext.putImageData(image, 0, 0);
    this.placeholder.hidden = true;
    if (!this.didInitialFit) {
      this.fit();
      this.didInitialFit = true;
    } else {
      this.draw();
    }
    this.onMetaChange({
      resolution: `${map.info.resolution.toFixed(3)} m/px`,
      size: `${map.info.width} x ${map.info.height}`,
      updated: formatMapUpdate(),
      zoom: this.zoomText(),
    });
  }

  setPose(message) {
    this.pose = message?.pose || null;
    this.draw();
  }

  setPreviewPath(message) {
    this.previewPath = Array.isArray(message?.poses) ? message.poses : null;
    this.draw();
  }

  setNavPath(message) {
    this.navPath = Array.isArray(message?.poses) ? message.poses : null;
    this.draw();
  }

  getGoal() {
    return this.goal;
  }

  clearGoal() {
    this.goal = null;
    this.previewPath = null;
    this.navPath = null;
    this.onGoalSelected(null);
    this.draw();
  }

  selectGoalFromEvent(event) {
    if (!this.map) return;
    const rect = this.canvas.getBoundingClientRect();
    const canvasX = event.clientX - rect.left;
    const canvasY = event.clientY - rect.top;
    const mapPixel = {
      x: (canvasX - this.offsetX) / this.scale,
      y: (canvasY - this.offsetY) / this.scale,
    };
    if (
      mapPixel.x < 0 ||
      mapPixel.y < 0 ||
      mapPixel.x >= this.map.info.width ||
      mapPixel.y >= this.map.info.height
    ) {
      return;
    }
    this.goal = mapPixelToWorld(mapPixel, this.map.info);
    if (this.pose?.position) {
      const dx = this.goal.x - this.pose.position.x;
      const dy = this.goal.y - this.pose.position.y;
      if (Math.hypot(dx, dy) > 0.05) {
        this.goal.orientation = yawToQuaternion(Math.atan2(dy, dx));
      }
    }
    this.onGoalSelected(this.goal);
    this.draw();
  }

  resize() {
    const rect = this.viewport.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(rect.width * ratio));
    this.canvas.height = Math.max(1, Math.round(rect.height * ratio));
    this.canvas.style.width = `${rect.width}px`;
    this.canvas.style.height = `${rect.height}px`;
    this.context.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.cssWidth = rect.width;
    this.cssHeight = rect.height;
    if (this.map) this.fit();
    else this.clear();
  }

  fit() {
    if (!this.map || !this.cssWidth || !this.cssHeight) return;
    const padding = 28;
    this.scale = Math.min(
      (this.cssWidth - padding * 2) / this.map.info.width,
      (this.cssHeight - padding * 2) / this.map.info.height,
    );
    this.offsetX = (this.cssWidth - this.map.info.width * this.scale) / 2;
    this.offsetY = (this.cssHeight - this.map.info.height * this.scale) / 2;
    this.draw();
  }

  reset() {
    this.fit();
  }

  clear() {
    this.context.clearRect(0, 0, this.cssWidth || 0, this.cssHeight || 0);
  }

  draw() {
    this.clear();
    if (!this.map) return;
    this.context.save();
    this.context.imageSmoothingEnabled = false;
    this.context.translate(this.offsetX, this.offsetY);
    this.context.scale(this.scale, this.scale);
    this.context.drawImage(this.mapCanvas, 0, 0);
    if (this.navPath?.length) {
      const points = this.navPath
        .map((item) => item?.pose)
        .filter(Boolean)
        .map((pose) => poseToMapPixel(pose, this.map.info));
      drawNavPath(this.context, points, this.scale);
    }
    if (this.previewPath?.length) {
      const points = this.previewPath
        .map((item) => item?.pose)
        .filter(Boolean)
        .map((pose) => poseToMapPixel(pose, this.map.info));
      drawPreviewPath(this.context, points, this.scale);
    }
    if (this.goal) {
      drawGoalMarker(this.context, worldToMapPixel(this.goal, this.map.info), this.scale);
    }
    if (this.pose) {
      drawRobotPose(this.context, poseToMapPixel(this.pose, this.map.info), this.scale);
    }
    this.context.restore();
    this.onMetaChange({ zoom: this.zoomText() });
  }

  zoomText() {
    return `${Math.round(this.scale * 100)}%`;
  }
}
