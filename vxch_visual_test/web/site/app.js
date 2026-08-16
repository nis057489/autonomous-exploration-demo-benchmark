// Frontend for the vxch WASM demo. This file owns UI/rendering only -- all
// codec logic (Haar transform, tiling, varint/zstd packing, reconstruction)
// runs inside vxch.wasm (see ../bindings.cpp), compiled from the same
// codec.cpp/haar_forward.hpp the real ROS nodes use. Nothing here
// reimplements that math, the same way gui/vxch_gui.py's Python frontend
// doesn't either.

const CANVAS_PX = 480;

// kbps presets matching wifi_profiles.json in the source repo, so this
// mirrors the same link conditions the real hardware experiments run under.
const WIFI_PRESETS = {
  "250000": "Good (250 mbps)",
  "1000": "Degraded (1 mbps)",
  "1": "Denied (1 kbps)",
};

let demo = null;
let sending = false;
let stopRequested = false;
let width = 0;
let height = 0;

const $ = (id) => document.getElementById(id);

function log(text) {
  const el = $("log");
  el.textContent += text + "\n";
  el.scrollTop = el.scrollHeight;
}

function setStatus(text) {
  $("status").textContent = text;
}

function fmtBytes(n) {
  return n >= 1024 ? `${(n / 1024).toFixed(1)} KB` : `${Math.round(n)} B`;
}

// Same grayscale mapping as gui/grid_io.py's grid_to_rgb: unknown (<0) mid
// gray, free (0) white, occupied (100) black, linear in between.
function cellToGray(v) {
  if (v < 0) {return 190;}
  return Math.max(0, Math.min(255, 255 - Math.round((v * 255) / 100)));
}

function drawGrid(canvas, cells, w, h) {
  const off = document.createElement("canvas");
  off.width = w;
  off.height = h;
  const octx = off.getContext("2d");
  const imgData = octx.createImageData(w, h);
  for (let i = 0; i < w * h; ++i) {
    const gray = cellToGray(cells[i]);
    imgData.data[i * 4 + 0] = gray;
    imgData.data[i * 4 + 1] = gray;
    imgData.data[i * 4 + 2] = gray;
    imgData.data[i * 4 + 3] = 255;
  }
  octx.putImageData(imgData, 0, 0);

  // Fit the longer side to CANVAS_PX in both directions (never just upscale
  // -- a map bigger than the display box must shrink to fit, the same fix
  // gui/vxch_gui.py needed for its Tk window). Nearest-neighbour when
  // enlarging keeps small maps crisp/blocky; the browser's default bilinear
  // smoothing when shrinking blends mixed regions instead of aliasing.
  const scale = CANVAS_PX / Math.max(w, h);
  const destW = Math.max(1, Math.round(w * scale));
  const destH = Math.max(1, Math.round(h * scale));

  canvas.width = CANVAS_PX;
  canvas.height = CANVAS_PX;
  const ctx = canvas.getContext("2d");
  ctx.imageSmoothingEnabled = scale < 1;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const offsetX = (CANVAS_PX - destW) / 2;
  const offsetY = (CANVAS_PX - destH) / 2;
  ctx.drawImage(off, 0, 0, w, h, offsetX, offsetY, destW, destH);
}

function drawBlankReceiver() {
  const cells = new Array(width * height).fill(-1);
  drawGrid($("canvas-receiver"), cells, width, height);
}

function jsArrayFromEmbind(val) {
  // val::array() on the C++ side already produces a plain JS Array.
  return val;
}

function setSendControlsEnabled(enabled) {
  $("btn-send-next").disabled = !enabled;
  $("btn-send-all").disabled = !enabled;
  $("btn-reset").disabled = !enabled;
}

function updateStatus() {
  const totalEntries = demo.totalEntries();
  if (totalEntries === 0) {return;}
  const sentIndex = demo.sentIndex();
  const cumulative = demo.cumulativeBytes();
  const totalCompressed = demo.totalCompressedBytes();
  const rawBytes = demo.rawBytes();
  const ratio = rawBytes ? (100 * totalCompressed) / rawBytes : 0;
  setStatus(
    `Sent ${sentIndex}/${totalEntries} bands ` +
    `(${fmtBytes(cumulative)} / ${fmtBytes(totalCompressed)} of encoded traffic, ` +
    `raw map = ${fmtBytes(rawBytes)}, full vxch stream = ${ratio.toFixed(0)}% of raw)`
  );
  if (sentIndex >= totalEntries) {
    $("btn-send-next").disabled = true;
    $("btn-send-all").disabled = true;
  }
}

function bandwidthKbps() {
  const v = parseFloat($("bandwidth").value);
  return Number.isFinite(v) && v > 0 ? v : 50;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function generateMap() {
  width = parseInt($("width").value, 10);
  height = parseInt($("height").value, 10);
  const seed = parseInt($("seed").value, 10) || 0;
  if (!Number.isFinite(width) || !Number.isFinite(height) || width < 1 || height < 1) {
    alert("Width/height must be positive integers.");
    return;
  }

  $("btn-generate").disabled = true;
  setStatus("Generating synthetic map...");

  const result = demo.genMap(width, height, seed >>> 0);
  drawGrid($("canvas-sender"), jsArrayFromEmbind(demo.getSenderGrid()), width, height);
  drawBlankReceiver();

  setSendControlsEnabled(false);
  $("btn-send-next").disabled = true;
  $("btn-send-all").disabled = true;
  $("btn-reset").disabled = true;
  $("btn-generate").disabled = false;
  $("btn-encode").disabled = false;

  log(
    `Generated ${result.width}x${result.height} map: ${result.freeCells} free, ` +
    `${result.occupiedCells} occupied, ${result.unknownCells} unknown cells.`
  );
  setStatus("Map generated. Encode it to build the band send queue.");
}

async function encodeMap() {
  const levels = parseInt($("levels").value, 10);
  const tileSize = parseInt($("tile-size").value, 10);
  const compression = $("compression").value;
  const useVarint = $("varint").checked;
  if (!Number.isFinite(levels) || !Number.isFinite(tileSize) || levels < 1 || tileSize < 1) {
    alert("Haar levels/tile size must be positive integers.");
    return;
  }

  $("btn-encode").disabled = true;
  setStatus("Encoding with vxch...");

  const result = demo.encode(levels, tileSize, compression, useVarint);
  drawBlankReceiver();

  $("btn-encode").disabled = false;
  setSendControlsEnabled(true);

  const ratio = result.rawBytes ? (100 * result.totalCompressedBytes) / result.rawBytes : 0;
  const varintLabel = result.varint ? "varint" : "fixed-width int32";
  log(
    `Encoded ${result.tiles} tiles x ${result.bandsPerTile} bands ` +
    `= ${result.totalEntries} band messages queued (${varintLabel} packing). ` +
    `Full stream = ${fmtBytes(result.totalCompressedBytes)} vs ` +
    `${fmtBytes(result.rawBytes)} raw (${ratio.toFixed(0)}%).`
  );
  updateStatus();
}

function resetReceiver() {
  demo.resetReceiver();
  drawBlankReceiver();
  setSendControlsEnabled(true);
  log("Receiver reset -- same encoded session, replaying from band 0.");
  updateStatus();
}

let lastCanvasDrawMs = 0;
// Canvas repaint (createImageData + drawImage over up to width*height
// pixels) is the expensive part of each band -- cheap for one click, but a
// large map's "Send All" can mean thousands of sendNext() calls (a 640x480
// map at the default tile size is 300 tiles x 5 bands = 1500), so redrawing
// every single one made the whole run visibly janky/slow. Redraw at most
// ~12/sec during a run, but always on the last band so the final frame is
// never stale. Log/stats stay per-band either way -- those are cheap DOM
// text updates, not canvas work.
function shouldRedrawCanvas(done) {
  const now = performance.now();
  if (done || now - lastCanvasDrawMs > 80) {
    lastCanvasDrawMs = now;
    return true;
  }
  return false;
}

// Pops the next queued band, waits out its simulated transfer time over the
// configured bandwidth, then reveals the receiver's updated reconstruction.
// Mirrors _send_one_band_blocking in gui/vxch_gui.py -- same honest (not
// fudged/animated) bytes*8/kbps delay, just via setTimeout instead of a
// background thread since the browser is single-threaded.
async function sendOneBand() {
  const kbps = bandwidthKbps();
  const result = demo.sendNext();
  if (!result.sent) {return true;}

  const label = result.bandIndex === 0 ? "LL (coarsest)" : `detail band ${result.bandIndex}`;
  const delayMs = (result.compressedSize * 8) / (kbps * 1000) * 1000;
  setStatus(
    `Sending tile (${result.tileRow},${result.tileCol}) ${label}: ` +
    `${fmtBytes(result.compressedSize)} @ ${kbps} kbps -> ~${delayMs.toFixed(0)} ms in flight...`
  );
  await sleep(delayMs);

  if (shouldRedrawCanvas(result.done)) {
    drawGrid($("canvas-receiver"), jsArrayFromEmbind(demo.getReceiverGrid()), width, height);
  }
  log(
    `tile(${result.tileRow},${result.tileCol}) ${label}: ` +
    `${fmtBytes(result.compressedSize)} compressed (${fmtBytes(result.uncompressedSize)} raw) ` +
    `delivered after ${delayMs.toFixed(0)} ms @ ${kbps} kbps`
  );
  updateStatus();
  return result.done;
}

async function sendNextClicked() {
  if (sending) {return;}
  sending = true;
  setSendControlsEnabled(false);
  await sendOneBand();
  sending = false;
  const done = demo.sentIndex() >= demo.totalEntries();
  setSendControlsEnabled(!done);
}

async function sendAllClicked() {
  if (sending) {return;}
  sending = true;
  stopRequested = false;
  setSendControlsEnabled(false);
  $("btn-stop").disabled = false;

  while (!stopRequested) {
    const done = await sendOneBand();
    if (done || stopRequested) {break;}
  }

  sending = false;
  $("btn-stop").disabled = true;
  const done = demo.sentIndex() >= demo.totalEntries();
  setSendControlsEnabled(!done);
  $("btn-reset").disabled = false;
}

function stopSending() {
  stopRequested = true;
}

function wirePresets() {
  document.querySelectorAll("button.preset").forEach((btn) => {
    btn.addEventListener("click", () => {
      $("bandwidth").value = btn.dataset.kbps;
    });
  });
}

createVxchModule().then((Module) => {
  demo = new Module.VxchDemo();
  setStatus("Ready. Generate a map to begin.");

  $("btn-generate").addEventListener("click", generateMap);
  $("btn-encode").addEventListener("click", encodeMap);
  $("btn-send-next").addEventListener("click", sendNextClicked);
  $("btn-send-all").addEventListener("click", sendAllClicked);
  $("btn-stop").addEventListener("click", stopSending);
  $("btn-reset").addEventListener("click", resetReceiver);
  wirePresets();
}).catch((err) => {
  setStatus("Failed to load WebAssembly module -- see console.");
  console.error(err);
});
