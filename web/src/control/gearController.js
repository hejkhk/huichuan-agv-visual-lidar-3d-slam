// 负责四挡循环、界面同步及 gear_change 发布。
import { DEFAULT_GEAR, DEFAULT_PROFILE, DRIVE_PROFILES } from "../config/gearConfig.js";

export class GearController {
  constructor({ publishers, onChange = () => {}, onUnavailable = () => {} }) {
    this.publishers = publishers;
    this.onChange = onChange;
    this.onUnavailable = onUnavailable;
    this.gear = DEFAULT_GEAR;
    this.profile = DEFAULT_PROFILE;
  }

  get config() {
    return DRIVE_PROFILES[this.profile].gears[this.gear];
  }

  get sequence() {
    return DRIVE_PROFILES[this.profile].sequence;
  }

  setProfile(profile, { publish = false } = {}) {
    if (!DRIVE_PROFILES[profile] || profile === this.profile) return;
    this.profile = profile;
    if (!this.sequence.includes(this.gear)) this.gear = this.sequence[0];
    this.onChange(this.gear, this.config, { publish, profileChanged: true });
  }

  setGear(nextGear, { publish = false } = {}) {
    if (!DRIVE_PROFILES[this.profile].gears[nextGear]) return;
    this.gear = nextGear;
    this.onChange(this.gear, this.config, { publish });
    if (publish) {
      const published = this.publishers.publishGear(this.gear);
      this.publishers.publishWebControl("gear_change", this.config, this.gear);
      if (!published) this.onUnavailable();
    }
  }

  cycle() {
    const index = this.sequence.indexOf(this.gear);
    this.setGear(this.sequence[(index + 1) % this.sequence.length], { publish: true });
  }
}
