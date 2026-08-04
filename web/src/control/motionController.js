// 实现按住即以 10Hz 发布运动命令，松开立即发布 stop。
const COMMAND_LABELS = {
  forward_left: "Forward Left",
  forward: "Forward",
  forward_right: "Forward Right",
  backward: "Backward",
  backward_left: "Backward Left",
  backward_right: "Backward Right",
  turn_left: "Turn Left",
  turn_right: "Turn Right",
  stop: "Stop",
};

export class MotionController {
  constructor({ buttons, publishers, getGear, onCommand = () => {} }) {
    this.buttons = buttons;
    this.publishers = publishers;
    this.getGear = getGear;
    this.onCommand = onCommand;
    this.interval = null;
    this.activeButton = null;
    this.enabled = false;
    this.bind();
  }

  bind() {
    this.buttons.forEach((button) => {
      const command = button.dataset.command;
      if (command === "stop") {
        button.addEventListener("click", () => this.stop());
        return;
      }
      button.addEventListener("pointerdown", (event) => this.start(event, button, command));
      button.addEventListener("pointerup", () => this.release());
      button.addEventListener("pointercancel", () => this.release());
      button.addEventListener("lostpointercapture", () => this.release());
      button.addEventListener("contextmenu", (event) => event.preventDefault());
    });
    window.addEventListener("blur", () => this.release());
  }

  setEnabled(enabled) {
    this.enabled = enabled;
    this.buttons.forEach((button) => {
      button.disabled = !enabled;
    });
    if (!enabled) this.cancel();
  }

  start(event, button, command) {
    if (!this.enabled || event.button !== 0) return;
    this.cancel();
    this.activeButton = button;
    button.classList.add("is-active");
    button.setPointerCapture?.(event.pointerId);
    this.send(command);
    this.interval = window.setInterval(() => this.send(command), 100);
  }

  release() {
    if (!this.activeButton) return;
    this.cancel();
    this.stop();
  }

  cancel() {
    window.clearInterval(this.interval);
    this.interval = null;
    this.activeButton?.classList.remove("is-active");
    this.activeButton = null;
  }

  stop() {
    if (!this.publishers.available()) return;
    this.cancel();
    const { gear, config } = this.getGear();
    this.publishers.publishCmdVel("stop", config);
    this.publishers.publishSerialCommand("zero_move");
    this.publishers.publishWebControl("stop", config, gear);
    this.onCommand("stop", "Zero-speed MOVE");
  }

  send(command) {
    const { gear, config } = this.getGear();
    this.publishers.publishWebControl(command, config, gear);
    this.publishers.publishCmdVel(command, config);
    this.onCommand(command, COMMAND_LABELS[command] || command);
  }
}
