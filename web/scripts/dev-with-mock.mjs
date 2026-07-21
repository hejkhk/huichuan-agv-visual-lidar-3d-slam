import { spawn } from "node:child_process";
import { fileURLToPath } from "node:url";
import { startMockStreamServer } from "./mock-stream-server.mjs";

const viteBin = fileURLToPath(
  new URL("../node_modules/vite/bin/vite.js", import.meta.url),
);

let mock;
let vite;
let shuttingDown = false;

async function shutdown(exitCode = 0) {
  if (shuttingDown) return;
  shuttingDown = true;

  if (vite && !vite.killed) vite.kill();
  if (mock) await mock.close();
  process.exit(exitCode);
}

try {
  mock = await startMockStreamServer();

  console.log(`\nMock Raspberry Pi stream: ${mock.streamUrl}`);
  console.log(`Looping video: ${mock.videoPath}\n`);

  vite = spawn(process.execPath, [viteBin, "--host", "0.0.0.0"], {
    env: {
      ...process.env,
      VITE_DEMO_STREAM_URL: mock.streamUrl,
    },
    stdio: "inherit",
    windowsHide: true,
  });

  vite.once("exit", (code) => {
    if (!shuttingDown) shutdown(code ?? 0);
  });

  vite.once("error", (error) => {
    console.error(`Unable to start Vite: ${error.message}`);
    shutdown(1);
  });

  process.once("SIGINT", () => shutdown(0));
  process.once("SIGTERM", () => shutdown(0));
} catch (error) {
  console.error(error.message);
  await shutdown(1);
}
