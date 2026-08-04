import { createServer } from "node:http";
import { existsSync } from "node:fs";
import { pathToFileURL } from "node:url";
import { spawn } from "node:child_process";
import ffmpegInstaller from "@ffmpeg-installer/ffmpeg";

const BOUNDARY = "frame";
const ffmpegPath = ffmpegInstaller.path;

function findMarker(buffer, firstByte, secondByte, start = 0) {
  for (let index = start; index < buffer.length - 1; index += 1) {
    if (buffer[index] === firstByte && buffer[index + 1] === secondByte) {
      return index;
    }
  }

  return -1;
}

export function startMockStreamServer({
  videoPath = process.env.ROBOT_MOCK_VIDEO || "",
  port = Number(process.env.ROBOT_MOCK_PORT || 8080),
} = {}) {
  if (!videoPath) {
    throw new Error(
      "Mock video path is empty. Set ROBOT_MOCK_VIDEO to a local MP4 file.",
    );
  }

  if (!existsSync(videoPath)) {
    throw new Error(
      `Mock video not found: ${videoPath}\nSet ROBOT_MOCK_VIDEO to another MP4 file path.`,
    );
  }

  if (!ffmpegPath) {
    throw new Error("The ffmpeg-static binary is unavailable on this platform.");
  }

  const clients = new Set();
  let ffmpeg = null;
  let jpegBuffer = Buffer.alloc(0);

  const stopEncoder = () => {
    if (!ffmpeg) return;
    ffmpeg.kill();
    ffmpeg = null;
    jpegBuffer = Buffer.alloc(0);
  };

  const broadcastFrame = (frame) => {
    const header = Buffer.from(
      `--${BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ${frame.length}\r\n\r\n`,
    );

    for (const response of clients) {
      response.write(header);
      response.write(frame);
      response.write("\r\n");
    }
  };

  const startEncoder = () => {
    if (ffmpeg) return;

    ffmpeg = spawn(
      ffmpegPath,
      [
        "-hide_banner",
        "-loglevel",
        "warning",
        "-re",
        "-stream_loop",
        "-1",
        "-i",
        videoPath,
        "-an",
        "-vf",
        "fps=20,scale='min(1280,iw)':-2",
        "-q:v",
        "5",
        "-f",
        "image2pipe",
        "-vcodec",
        "mjpeg",
        "pipe:1",
      ],
      { windowsHide: true },
    );

    ffmpeg.stdout.on("data", (chunk) => {
      jpegBuffer = Buffer.concat([jpegBuffer, chunk]);

      while (jpegBuffer.length > 1) {
        const start = findMarker(jpegBuffer, 0xff, 0xd8);
        if (start === -1) {
          jpegBuffer = jpegBuffer.subarray(Math.max(0, jpegBuffer.length - 1));
          break;
        }

        const end = findMarker(jpegBuffer, 0xff, 0xd9, start + 2);
        if (end === -1) {
          if (start > 0) jpegBuffer = jpegBuffer.subarray(start);
          break;
        }

        broadcastFrame(jpegBuffer.subarray(start, end + 2));
        jpegBuffer = jpegBuffer.subarray(end + 2);
      }
    });

    ffmpeg.stderr.on("data", (chunk) => {
      const message = chunk.toString().trim();
      if (message) console.error(`[mock-stream] ${message}`);
    });

    ffmpeg.on("error", (error) => {
      console.error(`[mock-stream] FFmpeg failed: ${error.message}`);
    });

    ffmpeg.on("exit", (code, signal) => {
      ffmpeg = null;
      if (clients.size > 0 && code !== 0 && signal !== "SIGTERM") {
        console.error(`[mock-stream] FFmpeg exited with code ${code}.`);
      }
    });
  };

  const server = createServer((request, response) => {
    if (request.url === "/video_feed") {
      response.writeHead(200, {
        "Access-Control-Allow-Origin": "*",
        "Cache-Control": "no-store, no-cache, must-revalidate",
        Connection: "keep-alive",
        "Content-Type": `multipart/x-mixed-replace; boundary=${BOUNDARY}`,
        Pragma: "no-cache",
      });
      response.flushHeaders();

      clients.add(response);
      startEncoder();

      request.on("close", () => {
        clients.delete(response);
        if (clients.size === 0) stopEncoder();
      });
      return;
    }

    if (request.url === "/health") {
      response.writeHead(200, { "Content-Type": "application/json; charset=utf-8" });
      response.end(
        JSON.stringify({
          status: "ok",
          stream: `http://127.0.0.1:${port}/video_feed`,
          videoPath,
        }),
      );
      return;
    }

    response.writeHead(200, { "Content-Type": "text/plain; charset=utf-8" });
    response.end(
      `Mock Raspberry Pi MJPEG stream\n\nOpen: http://127.0.0.1:${port}/video_feed\n`,
    );
  });

  return new Promise((resolve, reject) => {
    server.once("error", reject);
    server.listen(port, "0.0.0.0", () => {
      server.removeListener("error", reject);
      resolve({
        port,
        videoPath,
        streamUrl: `http://127.0.0.1:${port}/video_feed`,
        close: () =>
          new Promise((closeResolve) => {
            for (const response of clients) response.destroy();
            clients.clear();
            stopEncoder();
            server.close(() => closeResolve());
          }),
      });
    });
  });
}

const isDirectRun =
  process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href;

if (isDirectRun) {
  try {
    const mock = await startMockStreamServer({
      videoPath: process.argv[2] || process.env.ROBOT_MOCK_VIDEO || "",
    });

    console.log(`Mock video: ${mock.videoPath}`);
    console.log(`MJPEG stream: ${mock.streamUrl}`);
    console.log("Press Ctrl+C to stop.");

    const shutdown = async () => {
      await mock.close();
      process.exit(0);
    };

    process.once("SIGINT", shutdown);
    process.once("SIGTERM", shutdown);
  } catch (error) {
    console.error(error.message);
    process.exit(1);
  }
}
