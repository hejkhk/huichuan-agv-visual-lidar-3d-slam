// 校验局域网 IP、HTTP 视频地址和 ROSBridge WebSocket 地址。
export function normalizeRobotIp(value) {
  const trimmed = value.trim().replace(/^https?:\/\//i, "").replace(/\/.*$/, "");
  if (!trimmed) return { error: "请输入 Robot IP。" };

  const ipv4 = trimmed.split(".");
  const validIpv4 =
    ipv4.length === 4 &&
    ipv4.every((part) => /^\d{1,3}$/.test(part) && Number(part) >= 0 && Number(part) <= 255);
  const validHost = /^(localhost|[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?(?:\.[a-z0-9-]+)*)$/i.test(
    trimmed,
  );

  return validIpv4 || validHost ? { value: trimmed } : { error: "Robot IP 或主机名格式无效。" };
}

export function validateVideoUrl(value) {
  if (!value.trim()) return { error: "请输入 Video URL。" };
  try {
    const url = new URL(value.trim());
    if (!["http:", "https:"].includes(url.protocol) || !url.hostname) {
      return { error: "Video URL 必须是有效的 HTTP/HTTPS 地址。" };
    }
    return { value: url.href };
  } catch {
    return { error: "Video URL 格式无效。" };
  }
}

export function validateRosUrl(value) {
  if (!value.trim()) return { error: "请输入 ROSBridge URL。" };
  try {
    const url = new URL(value.trim());
    if (!["ws:", "wss:"].includes(url.protocol) || !url.hostname) {
      return { error: "ROSBridge URL 必须是有效的 WS/WSS 地址。" };
    }
    return { value: url.href };
  } catch {
    return { error: "ROSBridge URL 格式无效。" };
  }
}
