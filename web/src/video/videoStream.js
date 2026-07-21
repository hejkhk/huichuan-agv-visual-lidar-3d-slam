// 使用 img 元素连接 MJPEG，并报告连接中、接收、失败和断开状态。
export class VideoStream {
  constructor({ image, frame, placeholderText, onStateChange = () => {} }) {
    this.image = image;
    this.frame = frame;
    this.placeholderText = placeholderText;
    this.onStateChange = onStateChange;
    this.requested = false;

    image.addEventListener("load", () => {
      if (!this.requested) return;
      this.frame.classList.add("has-stream");
      this.setState("receiving");
    });
    image.addEventListener("error", () => {
      if (!this.requested) return;
      this.frame.classList.remove("has-stream");
      this.placeholderText.textContent = "Connection failed";
      this.setState("failed");
    });
  }

  connect(url) {
    this.disconnect();
    this.requested = true;
    this.placeholderText.textContent = "Connecting…";
    this.setState("connecting");
    this.image.src = url;
  }

  disconnect() {
    this.requested = false;
    this.image.removeAttribute("src");
    this.frame.classList.remove("has-stream");
    this.placeholderText.textContent = "No camera stream";
    this.setState("disconnected");
  }

  setState(state) {
    this.state = state;
    this.onStateChange(state);
  }
}
