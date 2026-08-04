// 管理前端软件急停锁定，并按协议发布急停、复位和零速度。
export class EstopController {
  constructor({
    publishers,
    getGear,
    onChange = () => {},
    onCommand = () => {},
    cancelMotion = () => {},
  }) {
    this.publishers = publishers;
    this.getGear = getGear;
    this.onChange = onChange;
    this.onCommand = onCommand;
    this.cancelMotion = cancelMotion;
    this.active = false;
  }

  activate() {
    const { gear, config } = this.getGear();
    this.cancelMotion();
    this.active = true;
    this.publishers.publishWebControl("emergency_stop", config, gear);
    this.publishers.publishEmergencyStop(true);
    this.publishers.publishCmdVel("stop", config);
    this.onCommand("emergency_stop", "E-Stop");
    this.onChange(true);
  }

  reset() {
    if (!this.active) return;
    const { gear, config } = this.getGear();
    this.active = false;
    this.publishers.publishWebControl("reset_estop", config, gear);
    this.publishers.publishEmergencyStop(false);
    this.onCommand("reset_estop", "Reset E-Stop");
    this.onChange(false);
  }
}
