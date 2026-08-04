# Windows 快速开始

## 环境

- Windows 10/11；
- Node.js 18 或更高；
- Chrome 或 Edge；
- Windows 与树莓派连接同一 WiFi/LAN。

## 启动

在 PowerShell 中执行：

```powershell
cd F:\Python\robot-video-viewer
npm install
npm run dev
```

打开：

```text
http://localhost:5173
```

输入 Robot IP 后，页面自动生成默认视频和 ROSBridge 地址。点击 `Connect Video`、`Connect ROS` 分别连接。连接配置保存在当前浏览器的 `localStorage`，刷新后恢复。

## 构建

```powershell
npm run build
```

静态交付文件位于 `dist/`。如需在其他 Windows 机器运行，可交付项目源码后重新 `npm install`，或使用任意静态 HTTP 服务托管 `dist/`；不要直接双击 `index.html`。
