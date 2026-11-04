"use strict";
/**
 * Canopy Seedling Planner: a Foxglove panel extension.
 *
 * Plain JavaScript with no dependencies and no build step. Foxglove loads this
 * file directly (see package.json "main"). Package and install it with
 * build_foxe.py in this folder.
 *
 * What the panel does:
 *   - Shows a satellite map (Esri World Imagery) centered on Flagstaff Hill.
 *   - Click the map to add seedlings, drag to move, right-click to delete.
 *   - Paste known lat/lon coordinates (or a whole `ros2 topic pub` command).
 *   - "Start planting" / "Stop" call the canopy_launcher services, which run
 *     launch/canopy_launch.launch.py in the background on the robot.
 *   - "Send plan" publishes canopy_msgs/msg/Mode {level: 2} on
 *     /planning/requested_mode, then canopy_msgs/msg/PlantingPlan on
 *     /planning/complete_plan.
 */

const PANEL_NAME = "Seedling Planner";

const ESRI_URL =
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";
const FLAGSTAFF = { lat: 40.44104, lon: -79.94695 };
const FAR_WARNING_M = 2000; // warn about points this far from Flagstaff Hill
const MODE_NAMES = ["TELEOP", "ASSISTED", "AUTO", "STOPPED"];
const MIN_ZOOM = 3;
const MAX_ZOOM = 22;
const TILE = 256;

const DEFAULT_SETTINGS = {
  planTopic: "/planning/complete_plan",
  sendMode: true,
  modeTopic: "/planning/requested_mode",
  modeLevel: 2,
  currentModeTopic: "/planning/current_mode",
  remainingTopic: "/planning/remaining_plan",
  robotTopic: "/vis/robot_fix",
  launcherNs: "/canopy_launcher",
  speciesPrefix: "seedling_",
  tileUrl: ESRI_URL,
  maxNativeZoom: 19, // Esri has no real imagery above 19 here
};

// Message definitions, so Foxglove can CDR-encode these types even when no
// other node on the robot is publishing them yet.
const PLAN_SCHEMA = "canopy_msgs/msg/PlantingPlan";
const SEEDLING_SCHEMA = "canopy_msgs/msg/Seedling";
const MODE_SCHEMA = "canopy_msgs/msg/Mode";
const field = (name, type, extra) =>
  Object.assign({ name, type, isComplex: false, isArray: false }, extra);
const DATATYPES = new Map([
  [
    PLAN_SCHEMA,
    {
      definitions: [
        field("bounds_geojson", "string"),
        field("seedlings", SEEDLING_SCHEMA, { isComplex: true, isArray: true }),
      ],
    },
  ],
  [
    SEEDLING_SCHEMA,
    {
      definitions: [
        field("latitude", "float64"),
        field("longitude", "float64"),
        field("species_id", "string"),
      ],
    },
  ],
  // Mode.msg declares `byte level`; byte and uint8 are the same on the wire.
  [MODE_SCHEMA, { definitions: [field("level", "uint8")] }],
]);

// ---------------------------------------------------------------- geometry

function worldSize(z) {
  return TILE * Math.pow(2, z);
}

function project(lat, lon, z) {
  const s = worldSize(z);
  const sin = Math.min(Math.max(Math.sin((lat * Math.PI) / 180), -0.9999), 0.9999);
  return {
    x: ((lon + 180) / 360) * s,
    y: (0.5 - Math.log((1 + sin) / (1 - sin)) / (4 * Math.PI)) * s,
  };
}

function unproject(x, y, z) {
  const s = worldSize(z);
  const n = Math.PI - (2 * Math.PI * y) / s;
  return { lat: (180 / Math.PI) * Math.atan(Math.sinh(n)), lon: (x / s) * 360 - 180 };
}

function distanceM(a, b) {
  const R = 6371000;
  const toRad = Math.PI / 180;
  const dLat = (b.lat - a.lat) * toRad;
  const dLon = (b.lon - a.lon) * toRad;
  const h =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(a.lat * toRad) * Math.cos(b.lat * toRad) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}

function isValidLatLon(lat, lon) {
  return (
    Number.isFinite(lat) && Number.isFinite(lon) && Math.abs(lat) <= 90 && Math.abs(lon) <= 180
  );
}

// ---------------------------------------------------------------- helpers

function clamp(v, lo, hi) {
  return Math.min(Math.max(v, lo), hi);
}

function escapeHtml(s) {
  return String(s).replace(
    /[&<>"']/g,
    (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c],
  );
}

function fmt(v) {
  return Number(v).toFixed(7);
}

function fmtDuration(sec) {
  sec = Math.max(0, Math.floor(sec));
  const h = Math.floor(sec / 3600);
  const m = Math.floor((sec % 3600) / 60);
  const s = sec % 60;
  return `${h}:${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`;
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

/**
 * Parse coordinates typed or pasted by the user. Accepts either:
 *   - one "lat, lon" pair per line (comma, space or tab separated), or
 *   - anything containing `latitude: X ... longitude: Y` pairs, such as a
 *     `ros2 topic pub ... PlantingPlan "{...}"` command or JSON.
 * Returns { points: [{lat, lon, species?}], errors: [string] }.
 */
function parseCoordinates(text) {
  const points = [];
  const errors = [];
  const num = "(-?\\d+(?:\\.\\d+)?(?:[eE][-+]?\\d+)?)";
  const pairRe = new RegExp(
    `latitude["']?\\s*[:=]\\s*${num}[\\s\\S]*?longitude["']?\\s*[:=]\\s*${num}` +
      `(?:[^{}]*?species_id["']?\\s*[:=]\\s*["']?([\\w.-]+))?`,
    "g",
  );
  let m;
  while ((m = pairRe.exec(text)) !== null) {
    const lat = parseFloat(m[1]);
    const lon = parseFloat(m[2]);
    if (isValidLatLon(lat, lon)) {
      points.push(m[3] ? { lat, lon, species: m[3] } : { lat, lon });
    } else {
      errors.push(`out of range: ${m[1]}, ${m[2]}`);
    }
  }
  if (points.length > 0 || errors.length > 0) {
    return { points, errors };
  }

  text.split(/\r?\n/).forEach((raw, i) => {
    const line = raw.replace(/#.*$/, "").trim();
    if (line === "") {
      return;
    }
    const nums = line.match(/-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?/g) || [];
    if (nums.length < 2) {
      errors.push(`line ${i + 1}: expected "lat, lon"`);
      return;
    }
    const lat = parseFloat(nums[0]);
    const lon = parseFloat(nums[1]);
    if (!isValidLatLon(lat, lon)) {
      errors.push(`line ${i + 1}: out of range`);
      return;
    }
    points.push({ lat, lon });
  });
  return { points, errors };
}

// ---------------------------------------------------------------- styles

const STYLE = `
.csp-root { --bg:#15171c; --panel:#1b1e24; --fg:#e6e6e6; --muted:#9aa0a6; --border:#2e323b;
  --input:#101217; --accent:#2ea043; --accent-fg:#fff; --danger:#da3633; --warn:#d29922;
  --info:#58a6ff; --seed:#3fb950; --sel:#ffd33d; --remaining:#ff9f1c; --robot:#58a6ff;
  position:absolute; inset:0; display:flex; background:var(--bg); color:var(--fg); color-scheme:dark;
  font:12px/1.4 system-ui,-apple-system,"Segoe UI",sans-serif; }
.csp-root.light { --bg:#fff; --panel:#f6f8fa; --fg:#1f2328; --muted:#59636e; --border:#d0d7de;
  --input:#fff; --warn:#9a6700; --info:#0969da; color-scheme:light; }
.csp-root.narrow { flex-direction:column; }
.csp-map { position:relative; flex:1; min-width:0; min-height:0; overflow:hidden; background:#222;
  cursor:crosshair; outline:none; touch-action:none; user-select:none; }
.csp-root.narrow .csp-map { flex:0 0 58%; }
.csp-tiles, .csp-overlay { position:absolute; inset:0; }
.csp-tile { position:absolute; left:0; top:0; opacity:0; transition:opacity .15s; pointer-events:none;
  image-rendering:auto; }
.csp-overlay { width:100%; height:100%; pointer-events:none; overflow:visible; }
.csp-overlay [data-seedling] { pointer-events:all; cursor:grab; }
.csp-overlay .csp-seed circle { fill:var(--seed); stroke:#fff; stroke-width:2; }
.csp-overlay .csp-seed.sel circle { fill:var(--sel); stroke:#000; }
.csp-overlay .csp-seed text { fill:#fff; font:bold 11px system-ui,sans-serif; text-anchor:middle;
  paint-order:stroke; stroke:#0008; stroke-width:2px; pointer-events:none; }
.csp-overlay .csp-seed.sel text { fill:#000; stroke:none; }
.csp-overlay .csp-path { fill:none; stroke:#fff; stroke-width:1.5; stroke-dasharray:4 4; opacity:.8; }
.csp-overlay .csp-remaining { fill:none; stroke:var(--remaining); stroke-width:3; }
.csp-overlay .csp-robot .halo { fill:var(--robot); opacity:.25; }
.csp-overlay .csp-robot .dot { fill:var(--robot); stroke:#fff; stroke-width:2; }
.csp-overlay .csp-robot.stale .dot { fill:#888; }
.csp-controls { position:absolute; top:8px; left:8px; display:flex; flex-direction:column; gap:4px; }
.csp-controls button { width:30px; height:30px; padding:0; font-size:16px; }
.csp-readout, .csp-hint, .csp-warning { position:absolute; padding:3px 7px; border-radius:4px;
  background:#000a; color:#fff; font-size:11px; pointer-events:none; }
.csp-readout { right:8px; bottom:8px; font-variant-numeric:tabular-nums; }
.csp-hint { left:8px; bottom:8px; }
.csp-warning { top:8px; left:50%; transform:translateX(-50%); background:#9a6700e0; }
.csp-scale { display:inline-block; height:6px; border:1.5px solid #fff; border-top:none;
  margin-right:6px; vertical-align:middle; }
.csp-side { flex:0 0 300px; display:flex; flex-direction:column; gap:10px; padding:10px;
  overflow-y:auto; background:var(--panel); border-left:1px solid var(--border); box-sizing:border-box; }
.csp-root.narrow .csp-side { flex:1; border-left:none; border-top:1px solid var(--border); }
.csp-side h3 { margin:0 0 6px; font-size:11px; letter-spacing:.06em; text-transform:uppercase;
  color:var(--muted); display:flex; justify-content:space-between; }
.csp-status { display:grid; grid-template-columns:auto 1fr; gap:2px 8px; margin-bottom:8px; }
.csp-status .k { color:var(--muted); }
.csp-status .ok { color:var(--seed); } .csp-status .bad { color:var(--danger); }
.csp-status .warn { color:var(--warn); }
.csp-row { display:flex; gap:6px; }
.csp-row > button { flex:1; }
.csp-root button { font:inherit; color:var(--fg); background:var(--input); border:1px solid var(--border);
  border-radius:4px; padding:5px 8px; cursor:pointer; }
.csp-root button:hover:not(:disabled) { border-color:var(--muted); }
.csp-root button:disabled { opacity:.45; cursor:default; }
.csp-root button.primary { background:var(--accent); border-color:var(--accent); color:var(--accent-fg);
  font-weight:600; }
.csp-root button.danger { background:var(--danger); border-color:var(--danger); color:#fff; font-weight:600; }
.csp-root button.confirm { background:var(--warn); border-color:var(--warn); color:#000; }
.csp-root button.big { width:100%; padding:9px; font-size:13px; }
.csp-list { max-height:240px; overflow-y:auto; border:1px solid var(--border); border-radius:4px;
  margin-bottom:6px; background:var(--input); }
.csp-empty { padding:10px; color:var(--muted); text-align:center; }
.csp-item { display:flex; align-items:center; gap:4px; padding:3px 4px; border-bottom:1px solid var(--border); }
.csp-item:last-child { border-bottom:none; }
.csp-item.sel { background:#ffd33d22; }
.csp-num { width:20px; text-align:right; color:var(--muted); font-variant-numeric:tabular-nums; }
.csp-item input { flex:1; min-width:0; font:11px ui-monospace,monospace; color:var(--fg); background:transparent;
  border:1px solid transparent; border-radius:3px; padding:2px 3px; }
.csp-item input:focus { border-color:var(--info); outline:none; background:var(--bg); }
.csp-item input.far { color:var(--warn); }
.csp-item button { padding:0 6px; border:none; background:transparent; color:var(--muted); font-size:14px; }
.csp-root textarea { width:100%; box-sizing:border-box; min-height:64px; resize:vertical;
  font:11px ui-monospace,monospace; color:var(--fg); background:var(--input); border:1px solid var(--border);
  border-radius:4px; padding:5px; margin-bottom:6px; }
.csp-note { color:var(--muted); font-size:11px; margin:4px 0 0; }
.csp-root details summary { cursor:pointer; color:var(--muted); margin-top:6px; }
.csp-root pre { white-space:pre-wrap; word-break:break-all; font:10px ui-monospace,monospace;
  background:var(--input); border:1px solid var(--border); border-radius:4px; padding:6px; margin:4px 0 0;
  user-select:text; }
.csp-log { font:11px ui-monospace,monospace; max-height:140px; overflow-y:auto; }
.csp-log div { padding:1px 0; }
.csp-log .t { color:var(--muted); margin-right:6px; }
.csp-log .error { color:var(--danger); } .csp-log .warn { color:var(--warn); } .csp-log .ok { color:var(--seed); }
`;

const HTML = `
<style>${STYLE}</style>
<div class="csp-root">
  <div class="csp-map" tabindex="0">
    <div class="csp-tiles"></div>
    <svg class="csp-overlay"></svg>
    <div class="csp-controls">
      <button data-act="zoom-in" title="Zoom in">+</button>
      <button data-act="zoom-out" title="Zoom out">&minus;</button>
      <button data-act="fit" title="Fit seedlings and robot">&#x2922;</button>
      <button data-act="center-robot" title="Center on robot">&#x25CE;</button>
      <button data-act="home" title="Back to Flagstaff Hill">&#x2302;</button>
    </div>
    <div class="csp-warning" data-el="tile-warning" hidden>Satellite tiles are not loading. Is this computer online?</div>
    <div class="csp-hint">Click: add &middot; Drag: move &middot; Right-click: delete</div>
    <div class="csp-readout" data-el="readout"></div>
  </div>
  <div class="csp-side">
    <section>
      <h3>Robot</h3>
      <div class="csp-status" data-el="status"></div>
      <div class="csp-row">
        <button class="primary" data-act="start">Start planting</button>
        <button class="danger" data-act="stop">Stop</button>
      </div>
    </section>
    <section>
      <h3><span>Seedlings</span><span data-el="count"></span></h3>
      <div class="csp-list" data-el="list"></div>
      <div class="csp-row">
        <button data-act="undo">Undo</button>
        <button data-act="clear">Clear</button>
        <button data-act="load-remaining" title="Copy the plan the robot is currently following into the editor">Load robot plan</button>
      </div>
    </section>
    <section>
      <h3>Add by coordinates</h3>
      <textarea data-el="manual" spellcheck="false"
        placeholder="One &quot;lat, lon&quot; per line, e.g.&#10;40.441133, -79.947280&#10;40.441069, -79.947142&#10;or paste a ros2 topic pub PlantingPlan command"></textarea>
      <div class="csp-row">
        <button data-act="manual-add">Add to list</button>
        <button data-act="manual-replace">Replace list</button>
      </div>
    </section>
    <section>
      <button class="primary big" data-act="send">Send plan</button>
      <p class="csp-note" data-el="send-note"></p>
      <details>
        <summary>Equivalent ros2 CLI commands</summary>
        <pre data-el="cli"></pre>
      </details>
    </section>
    <section>
      <h3>Log</h3>
      <div class="csp-log" data-el="log"></div>
    </section>
  </div>
</div>`;

// ---------------------------------------------------------------- panel

function initPanel(context) {
  const saved = context.initialState || {};
  const settings = Object.assign({}, DEFAULT_SETTINGS, saved.settings);
  const st = {
    view: Object.assign({ lat: FLAGSTAFF.lat, lon: FLAGSTAFF.lon, zoom: 19 }, saved.view),
    seedlings: (Array.isArray(saved.seedlings) ? saved.seedlings : []).filter((s) =>
      isValidLatLon(s && s.lat, s && s.lon),
    ),
    selected: -1,
    history: [],
    robot: null, // {lat, lon, t}
    remaining: null, // {points, t}
    currentMode: null, // {level, t}
    launcher: null, // {status, t}
    confirm: null, // {action, until}
    advertised: {}, // topic -> schema
    log: [],
    tileLoads: 0,
    tileErrors: 0,
    size: { w: 1, h: 1 },
    drag: null,
    wheelAccum: 0,
    busy: {},
  };
  st.view.zoom = clamp(Math.round(st.view.zoom), MIN_ZOOM, MAX_ZOOM);

  const panel = context.panelElement;
  panel.innerHTML = HTML;
  const root = panel.querySelector(".csp-root");
  const mapEl = root.querySelector(".csp-map");
  const tileLayer = root.querySelector(".csp-tiles");
  const svg = root.querySelector(".csp-overlay");
  const el = (name) => root.querySelector(`[data-el="${name}"]`);
  const btn = (act) => root.querySelector(`button[data-act="${act}"]`);
  const tiles = new Map(); // key -> img

  context.setDefaultPanelTitle && context.setDefaultPanelTitle(PANEL_NAME);

  // ---------------------------------------------------------- persistence

  let saveTimer;
  function saveSoon() {
    clearTimeout(saveTimer);
    saveTimer = setTimeout(() => {
      context.saveState({
        settings,
        view: { lat: st.view.lat, lon: st.view.lon, zoom: st.view.zoom },
        seedlings: st.seedlings,
      });
    }, 300);
  }

  function pushHistory() {
    st.history.push(st.seedlings.map((s) => Object.assign({}, s)));
    if (st.history.length > 100) {
      st.history.shift();
    }
  }

  function seedlingsChanged() {
    if (st.selected >= st.seedlings.length) {
      st.selected = -1;
    }
    renderList();
    renderOverlay();
    renderSendState();
    saveSoon();
  }

  function addLog(text, level) {
    const t = new Date().toLocaleTimeString([], { hour12: false });
    st.log.unshift({ t, text, level: level || "" });
    st.log.length = Math.min(st.log.length, 40);
    el("log").innerHTML = st.log
      .map((l) => `<div class="${l.level}"><span class="t">${l.t}</span>${escapeHtml(l.text)}</div>`)
      .join("");
  }

  // ---------------------------------------------------------- ROS plumbing

  function canPublish() {
    return typeof context.publish === "function" && typeof context.advertise === "function";
  }

  function syncAdvertisements() {
    if (typeof context.advertise !== "function") {
      return;
    }
    const want = { [settings.planTopic]: PLAN_SCHEMA };
    if (settings.sendMode) {
      want[settings.modeTopic] = MODE_SCHEMA;
    }
    for (const topic of Object.keys(st.advertised)) {
      if (want[topic] !== st.advertised[topic]) {
        try {
          context.unadvertise && context.unadvertise(topic);
        } catch (e) {
          // ignore
        }
        delete st.advertised[topic];
      }
    }
    for (const [topic, schema] of Object.entries(want)) {
      if (!st.advertised[topic]) {
        try {
          context.advertise(topic, schema, { datatypes: DATATYPES });
          st.advertised[topic] = schema;
        } catch (e) {
          addLog(`Could not advertise ${topic}: ${e.message || e}`, "error");
        }
      }
    }
  }

  function syncSubscriptions() {
    const topics = [
      settings.robotTopic,
      settings.remainingTopic,
      settings.currentModeTopic,
      `${settings.launcherNs}/status`,
    ].filter((t, i, a) => t && a.indexOf(t) === i);
    context.subscribe(topics.map((topic) => ({ topic })));
  }

  function handleMessage(ev) {
    const m = ev.message || {};
    const now = Date.now();
    if (ev.topic === settings.robotTopic) {
      if (isValidLatLon(m.latitude, m.longitude) && !(m.latitude === 0 && m.longitude === 0)) {
        st.robot = { lat: m.latitude, lon: m.longitude, t: now };
        scheduleMap();
      }
    }
    if (ev.topic === settings.remainingTopic) {
      const points = (m.seedlings || [])
        .map((s) => ({ lat: s.latitude, lon: s.longitude, species: s.species_id }))
        .filter((p) => isValidLatLon(p.lat, p.lon));
      st.remaining = { points, t: now };
      scheduleMap();
    }
    if (ev.topic === settings.currentModeTopic) {
      let level = m.level;
      if (level != null && typeof level !== "number") {
        level = level[0];
      }
      st.currentMode = { level, t: now };
    }
    if (ev.topic === `${settings.launcherNs}/status`) {
      try {
        st.launcher = { status: JSON.parse(m.data), t: now };
      } catch (e) {
        st.launcher = { status: { state: "unknown", message: String(m.data) }, t: now };
      }
    }
  }

  async function callLauncher(action) {
    const service = `${settings.launcherNs}/${action}`;
    if (typeof context.callService !== "function") {
      addLog("This connection cannot call services. Connect through foxglove_bridge.", "error");
      return;
    }
    st.busy[action] = true;
    renderButtons();
    addLog(`Calling ${service} ...`);
    try {
      const res = (await context.callService(service, {})) || {};
      addLog(`${service}: ${res.message || (res.success ? "ok" : "failed")}`, res.success ? "ok" : "error");
    } catch (e) {
      const msg = String(e.message || e);
      const hint = /not|unknown|advertis/i.test(msg)
        ? " (is foxglove_operator.launch.py running on the robot?)"
        : "";
      addLog(`${service} failed: ${msg}${hint}`, "error");
    } finally {
      st.busy[action] = false;
      renderButtons();
    }
  }

  function buildPlanMessage() {
    return {
      bounds_geojson: "",
      seedlings: st.seedlings.map((s, i) => ({
        latitude: s.lat,
        longitude: s.lon,
        species_id: s.species || `${settings.speciesPrefix}${i + 1}`,
      })),
    };
  }

  async function sendPlan() {
    if (!canPublish()) {
      addLog("Publishing is not available. Connect to the robot through foxglove_bridge.", "error");
      return;
    }
    if (st.seedlings.length === 0) {
      addLog("No seedlings to send.", "warn");
      return;
    }
    st.busy.send = true;
    renderButtons();
    try {
      syncAdvertisements();
      if (settings.sendMode) {
        const level = Number(settings.modeLevel);
        context.publish(settings.modeTopic, { level });
        addLog(`Published ${settings.modeTopic} level ${level} (${MODE_NAMES[level] || "?"})`, "ok");
        await sleep(500); // the FSM must switch mode before the plan arrives
      }
      const msg = buildPlanMessage();
      context.publish(settings.planTopic, msg);
      addLog(`Published ${settings.planTopic} with ${msg.seedlings.length} seedlings`, "ok");
    } catch (e) {
      addLog(`Send failed: ${e.message || e}`, "error");
    } finally {
      st.busy.send = false;
      renderButtons();
    }
  }

  // Two-click confirmation for actions that make the robot do something.
  function withConfirm(action, fn) {
    const now = Date.now();
    if (st.confirm && st.confirm.action === action && st.confirm.until > now) {
      st.confirm = null;
      renderButtons();
      fn();
      return;
    }
    st.confirm = { action, until: now + 4000 };
    renderButtons();
    setTimeout(() => {
      if (st.confirm && st.confirm.action === action && Date.now() >= st.confirm.until) {
        st.confirm = null;
        renderButtons();
      }
    }, 4100);
  }

  // ---------------------------------------------------------- map math

  function centerPx() {
    return project(st.view.lat, st.view.lon, st.view.zoom);
  }

  function geoToScreen(lat, lon) {
    const c = centerPx();
    const p = project(lat, lon, st.view.zoom);
    return { x: p.x - c.x + st.size.w / 2, y: p.y - c.y + st.size.h / 2 };
  }

  function screenToGeo(x, y) {
    const c = centerPx();
    return unproject(c.x + x - st.size.w / 2, c.y + y - st.size.h / 2, st.view.zoom);
  }

  function zoomAt(newZoom, x, y) {
    newZoom = clamp(newZoom, MIN_ZOOM, MAX_ZOOM);
    if (newZoom === st.view.zoom) {
      return;
    }
    if (x == null) {
      x = st.size.w / 2;
      y = st.size.h / 2;
    }
    const geo = screenToGeo(x, y);
    const g = project(geo.lat, geo.lon, newZoom);
    const c = unproject(g.x - (x - st.size.w / 2), g.y - (y - st.size.h / 2), newZoom);
    st.view = { lat: c.lat, lon: c.lon, zoom: newZoom };
    scheduleMap();
    saveSoon();
  }

  function centerOn(lat, lon, zoom) {
    st.view = { lat, lon, zoom: zoom == null ? st.view.zoom : zoom };
    scheduleMap();
    saveSoon();
  }

  function fitPoints(points) {
    if (points.length === 0) {
      return;
    }
    if (points.length === 1) {
      centerOn(points[0].lat, points[0].lon, Math.max(st.view.zoom, 19));
      return;
    }
    const lats = points.map((p) => p.lat);
    const lons = points.map((p) => p.lon);
    const lat = (Math.min(...lats) + Math.max(...lats)) / 2;
    const lon = (Math.min(...lons) + Math.max(...lons)) / 2;
    let zoom = 21;
    for (; zoom > MIN_ZOOM; zoom--) {
      const a = project(Math.max(...lats), Math.min(...lons), zoom);
      const b = project(Math.min(...lats), Math.max(...lons), zoom);
      if (b.x - a.x < st.size.w - 80 && b.y - a.y < st.size.h - 80) {
        break;
      }
    }
    centerOn(lat, lon, zoom);
  }

  // ---------------------------------------------------------- rendering

  let mapFrame = 0;
  function scheduleMap() {
    if (!mapFrame) {
      mapFrame = requestAnimationFrame(() => {
        mapFrame = 0;
        renderTiles();
        renderOverlay();
        renderReadout();
      });
    }
  }

  function renderTiles() {
    const { w, h } = st.size;
    const z = st.view.zoom;
    const tz = Math.min(z, Number(settings.maxNativeZoom) || 19);
    const ts = TILE * Math.pow(2, z - tz);
    const c = centerPx();
    const left = c.x - w / 2;
    const top = c.y - h / 2;
    const n = Math.pow(2, tz);
    const x0 = Math.floor(left / ts);
    const x1 = Math.floor((left + w) / ts);
    const y0 = Math.max(0, Math.floor(top / ts));
    const y1 = Math.min(n - 1, Math.floor((top + h) / ts));
    const keep = new Set();
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const key = `${tz}/${x}/${y}`;
        keep.add(key);
        let img = tiles.get(key);
        if (!img) {
          img = document.createElement("img");
          img.className = "csp-tile";
          img.draggable = false;
          img.alt = "";
          img.onload = () => {
            img.style.opacity = "1";
            st.tileLoads++;
            renderTileWarning();
          };
          img.onerror = () => {
            st.tileErrors++;
            renderTileWarning();
          };
          const xx = ((x % n) + n) % n;
          img.src = String(settings.tileUrl)
            .replace("{z}", tz)
            .replace("{x}", xx)
            .replace("{y}", y);
          tileLayer.appendChild(img);
          tiles.set(key, img);
        }
        img.style.width = img.style.height = `${ts}px`;
        img.style.transform = `translate(${Math.round(x * ts - left)}px, ${Math.round(y * ts - top)}px)`;
      }
    }
    for (const [key, img] of tiles) {
      if (!keep.has(key)) {
        img.remove();
        tiles.delete(key);
      }
    }
  }

  function renderTileWarning() {
    el("tile-warning").hidden = !(st.tileErrors > 3 && st.tileLoads === 0);
  }

  function renderOverlay() {
    const parts = [];
    if (st.seedlings.length > 1) {
      const pts = st.seedlings
        .map((s) => geoToScreen(s.lat, s.lon))
        .map((p) => `${p.x.toFixed(1)},${p.y.toFixed(1)}`)
        .join(" ");
      parts.push(`<polyline class="csp-path" points="${pts}"/>`);
    }
    for (const r of (st.remaining && st.remaining.points) || []) {
      const p = geoToScreen(r.lat, r.lon);
      parts.push(`<circle class="csp-remaining" cx="${p.x}" cy="${p.y}" r="14"/>`);
    }
    st.seedlings.forEach((s, i) => {
      const p = geoToScreen(s.lat, s.lon);
      const sel = i === st.selected ? " sel" : "";
      const label = escapeHtml(s.species || `${settings.speciesPrefix}${i + 1}`);
      parts.push(
        `<g class="csp-seed${sel}" data-seedling="${i}" transform="translate(${p.x},${p.y})">` +
          `<title>${label}\n${fmt(s.lat)}, ${fmt(s.lon)}</title>` +
          `<circle r="10"/><text dy="0.35em">${i + 1}</text></g>`,
      );
    });
    if (st.robot) {
      const p = geoToScreen(st.robot.lat, st.robot.lon);
      const stale = Date.now() - st.robot.t > 3000 ? " stale" : "";
      parts.push(
        `<g class="csp-robot${stale}" transform="translate(${p.x},${p.y})">` +
          `<circle class="halo" r="14"/><circle class="dot" r="6"/></g>`,
      );
    }
    svg.innerHTML = parts.join("");
  }

  let cursorGeo = null;
  function renderReadout() {
    const lat = cursorGeo ? cursorGeo.lat : st.view.lat;
    // Pick a "nice" scale bar length that fits within ~100 px.
    const mpp = (156543.03392 * Math.cos((lat * Math.PI) / 180)) / Math.pow(2, st.view.zoom);
    const nice = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000];
    let meters = nice[0];
    for (const v of nice) {
      if (v / mpp <= 100) {
        meters = v;
      }
    }
    const label = meters >= 1000 ? `${meters / 1000} km` : `${meters} m`;
    const coords = cursorGeo ? `${fmt(cursorGeo.lat)}, ${fmt(cursorGeo.lon)} &middot; ` : "";
    el("readout").innerHTML =
      `${coords}<span class="csp-scale" style="width:${Math.round(meters / mpp)}px"></span>${label}` +
      ` &middot; z${st.view.zoom}`;
  }

  function renderList() {
    el("count").textContent = st.seedlings.length ? String(st.seedlings.length) : "";
    if (st.seedlings.length === 0) {
      el("list").innerHTML =
        '<div class="csp-empty">Click the map or add coordinates below.</div>';
    } else {
      el("list").innerHTML = st.seedlings
        .map((s, i) => {
          const far = distanceM(s, FLAGSTAFF) > FAR_WARNING_M ? " far" : "";
          const title = escapeHtml(s.species || `${settings.speciesPrefix}${i + 1}`);
          return (
            `<div class="csp-item${i === st.selected ? " sel" : ""}" data-idx="${i}" title="${title}">` +
            `<span class="csp-num">${i + 1}</span>` +
            `<input class="${far}" data-field="lat" data-idx="${i}" value="${fmt(s.lat)}" spellcheck="false">` +
            `<input class="${far}" data-field="lon" data-idx="${i}" value="${fmt(s.lon)}" spellcheck="false">` +
            `<button data-act="del" data-idx="${i}" title="Delete">&times;</button></div>`
          );
        })
        .join("");
    }
    renderCli();
  }

  function renderCli() {
    const msg = buildPlanMessage();
    const seeds = msg.seedlings
      .map((s) => `{latitude: ${fmt(s.latitude)}, longitude: ${fmt(s.longitude)}, species_id: '${s.species_id}'}`)
      .join(", ");
    const lines = [];
    if (settings.sendMode) {
      lines.push(
        `ros2 topic pub --once ${settings.modeTopic} ${MODE_SCHEMA} "{level: [${Number(settings.modeLevel)}]}"`,
      );
    }
    lines.push(
      `ros2 topic pub --once ${settings.planTopic} ${PLAN_SCHEMA} "{bounds_geojson: '', seedlings: [${seeds}]}"`,
    );
    el("cli").textContent = lines.join("\n\n");
  }

  function setButton(act, label, opts) {
    const b = btn(act);
    const confirming = st.confirm && st.confirm.action === act;
    b.textContent = confirming ? "Click again to confirm" : label;
    b.classList.toggle("confirm", !!confirming);
    b.disabled = !!(opts && opts.disabled);
  }

  function renderButtons() {
    const n = st.seedlings.length;
    const level = Number(settings.modeLevel);
    setButton("start", st.busy.start ? "Starting..." : "Start planting", { disabled: st.busy.start });
    setButton("stop", st.busy.stop ? "Stopping..." : "Stop", { disabled: st.busy.stop });
    setButton(
      "send",
      st.busy.send ? "Sending..." : `Send plan (${n} seedling${n === 1 ? "" : "s"})`,
      { disabled: st.busy.send || n === 0 || !canPublish() },
    );
    btn("undo").disabled = st.history.length === 0;
    btn("clear").disabled = n === 0;
    btn("load-remaining").disabled = !(st.remaining && st.remaining.points.length);
    el("send-note").textContent = !canPublish()
      ? "Publishing needs a live foxglove_bridge connection."
      : settings.sendMode
        ? `Publishes ${settings.modeTopic} = ${MODE_NAMES[level] || level}, then ${settings.planTopic}.`
        : `Publishes ${settings.planTopic}.`;
  }

  function renderSendState() {
    renderButtons();
  }

  function renderStatus() {
    const now = Date.now();
    const rows = [];
    const row = (k, v, cls) => rows.push(`<span class="k">${k}</span><span class="${cls || ""}">${v}</span>`);

    const l = st.launcher;
    if (!l || now - l.t > 5000) {
      row("Stack", "no launcher status (run foxglove_operator.launch.py)", "warn");
    } else {
      const s = l.status || {};
      if (s.state === "running") {
        row("Stack", `running &middot; ${fmtDuration(s.uptime_s || 0)}`, "ok");
      } else if (s.state === "stopping") {
        row("Stack", "stopping...", "warn");
      } else {
        const exit = s.last_exit_code != null ? ` (last exit ${s.last_exit_code})` : "";
        row("Stack", `${escapeHtml(s.state || "unknown")}${exit}`, s.last_exit_code ? "bad" : "");
      }
    }

    const m = st.currentMode;
    if (!m || now - m.t > 3000) {
      row("Mode", "unknown (FSM not running?)", "warn");
    } else {
      row("Mode", escapeHtml(MODE_NAMES[m.level] || String(m.level)), m.level === 2 ? "ok" : "");
    }

    if (!st.robot) {
      row("GPS", `no fix on ${escapeHtml(settings.robotTopic)}`, "warn");
    } else {
      const age = (now - st.robot.t) / 1000;
      row("GPS", `${fmt(st.robot.lat)}, ${fmt(st.robot.lon)}${age > 3 ? ` (${age.toFixed(0)} s old)` : ""}`,
        age > 3 ? "warn" : "");
    }

    if (st.remaining) {
      row("Robot plan", `${st.remaining.points.length} seedlings remaining`);
    } else {
      row("Robot plan", "none received");
    }
    el("status").innerHTML = rows.join("");
  }

  // ---------------------------------------------------------- seedling edits

  function addSeedling(lat, lon) {
    pushHistory();
    st.seedlings.push({ lat, lon });
    st.selected = st.seedlings.length - 1;
    seedlingsChanged();
  }

  function deleteSeedling(i) {
    if (i < 0 || i >= st.seedlings.length) {
      return;
    }
    pushHistory();
    st.seedlings.splice(i, 1);
    st.selected = -1;
    seedlingsChanged();
  }

  function addManual(replace) {
    const text = el("manual").value;
    const { points, errors } = parseCoordinates(text);
    errors.forEach((e) => addLog(`Coordinates: ${e}`, "warn"));
    if (points.length === 0) {
      addLog("No valid coordinates found.", "warn");
      return;
    }
    const far = points.filter((p) => distanceM(p, FLAGSTAFF) > FAR_WARNING_M).length;
    if (far > 0) {
      addLog(`${far} point(s) are more than ${FAR_WARNING_M / 1000} km from Flagstaff Hill. Check lat/lon order.`, "warn");
    }
    pushHistory();
    st.seedlings = replace ? points : st.seedlings.concat(points);
    st.selected = -1;
    seedlingsChanged();
    addLog(`${replace ? "Replaced list with" : "Added"} ${points.length} seedling(s)`, "ok");
    el("manual").value = "";
    fitPoints(st.seedlings);
  }

  // ---------------------------------------------------------- events

  function localXY(e) {
    const r = mapEl.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  }

  function onPointerDown(e) {
    if (e.button !== 0 || e.target.closest(".csp-controls")) {
      return;
    }
    mapEl.focus();
    const g = e.target.closest && e.target.closest("[data-seedling]");
    const p = localXY(e);
    st.drag = {
      id: e.pointerId,
      start: p,
      moved: false,
      idx: g ? Number(g.getAttribute("data-seedling")) : -1,
      center: centerPx(),
    };
    try {
      mapEl.setPointerCapture(e.pointerId);
    } catch (err) {
      // pointer already released; dragging still works while over the map
    }
    e.preventDefault();
  }

  function onPointerMove(e) {
    const p = localXY(e);
    cursorGeo = screenToGeo(p.x, p.y);
    const d = st.drag;
    if (d && d.id === e.pointerId) {
      const dx = p.x - d.start.x;
      const dy = p.y - d.start.y;
      if (!d.moved && Math.hypot(dx, dy) > 4) {
        d.moved = true;
        if (d.idx >= 0) {
          pushHistory();
          st.selected = d.idx;
        }
      }
      if (d.moved) {
        if (d.idx >= 0) {
          st.seedlings[d.idx] = Object.assign({}, st.seedlings[d.idx], cursorGeo);
        } else {
          const c = unproject(d.center.x - dx, d.center.y - dy, st.view.zoom);
          st.view.lat = c.lat;
          st.view.lon = c.lon;
        }
        scheduleMap();
        return;
      }
    }
    renderReadout();
  }

  function onPointerUp(e) {
    const d = st.drag;
    if (!d || d.id !== e.pointerId) {
      return;
    }
    st.drag = null;
    if (!d.moved) {
      if (d.idx >= 0) {
        st.selected = st.selected === d.idx ? -1 : d.idx;
        renderList();
        renderOverlay();
      } else {
        const p = localXY(e);
        const g = screenToGeo(p.x, p.y);
        addSeedling(g.lat, g.lon);
      }
    } else if (d.idx >= 0) {
      seedlingsChanged();
    } else {
      saveSoon();
    }
  }

  function onContextMenu(e) {
    const g = e.target.closest && e.target.closest("[data-seedling]");
    e.preventDefault();
    if (g) {
      deleteSeedling(Number(g.getAttribute("data-seedling")));
    }
  }

  function onWheel(e) {
    e.preventDefault();
    st.wheelAccum += e.deltaY;
    if (Math.abs(st.wheelAccum) < 60) {
      return;
    }
    const p = localXY(e);
    zoomAt(st.view.zoom + (st.wheelAccum < 0 ? 1 : -1), p.x, p.y);
    st.wheelAccum = 0;
  }

  function onKeyDown(e) {
    if ((e.key === "Delete" || e.key === "Backspace") && st.selected >= 0) {
      deleteSeedling(st.selected);
      e.preventDefault();
    } else if (e.key === "Escape") {
      st.selected = -1;
      renderList();
      renderOverlay();
    } else if ((e.ctrlKey || e.metaKey) && e.key === "z") {
      undo();
      e.preventDefault();
    }
  }

  function undo() {
    if (st.history.length === 0) {
      return;
    }
    st.seedlings = st.history.pop();
    st.selected = -1;
    seedlingsChanged();
  }

  function onClick(e) {
    const b = e.target.closest("button[data-act]");
    if (!b || b.disabled) {
      return;
    }
    switch (b.getAttribute("data-act")) {
      case "zoom-in":
        zoomAt(st.view.zoom + 1);
        break;
      case "zoom-out":
        zoomAt(st.view.zoom - 1);
        break;
      case "fit":
        fitPoints(st.seedlings.concat(st.robot ? [st.robot] : []));
        break;
      case "center-robot":
        if (st.robot) {
          centerOn(st.robot.lat, st.robot.lon);
        } else {
          addLog(`No robot position on ${settings.robotTopic} yet.`, "warn");
        }
        break;
      case "home":
        centerOn(FLAGSTAFF.lat, FLAGSTAFF.lon, 19);
        break;
      case "start":
        withConfirm("start", () => callLauncher("start"));
        break;
      case "stop":
        callLauncher("stop");
        break;
      case "send":
        withConfirm("send", sendPlan);
        break;
      case "undo":
        undo();
        break;
      case "clear":
        pushHistory();
        st.seedlings = [];
        seedlingsChanged();
        break;
      case "load-remaining":
        if (st.remaining) {
          pushHistory();
          st.seedlings = st.remaining.points.map((p) => Object.assign({}, p));
          seedlingsChanged();
          fitPoints(st.seedlings);
        }
        break;
      case "manual-add":
        addManual(false);
        break;
      case "manual-replace":
        addManual(true);
        break;
      case "del":
        deleteSeedling(Number(b.getAttribute("data-idx")));
        break;
    }
  }

  function onListChange(e) {
    const input = e.target.closest("input[data-field]");
    if (!input) {
      return;
    }
    const i = Number(input.getAttribute("data-idx"));
    const v = parseFloat(input.value);
    const s = st.seedlings[i];
    const next = Object.assign({}, s, { [input.getAttribute("data-field")]: v });
    if (!s || !isValidLatLon(next.lat, next.lon)) {
      addLog(`Invalid value "${input.value}"`, "warn");
      renderList();
      return;
    }
    pushHistory();
    st.seedlings[i] = next;
    seedlingsChanged();
  }

  function onListClick(e) {
    const item = e.target.closest(".csp-item");
    if (!item || e.target.closest("button")) {
      return;
    }
    const i = Number(item.getAttribute("data-idx"));
    if (st.selected !== i) {
      st.selected = i;
      root.querySelectorAll(".csp-item").forEach((n) => n.classList.toggle("sel", n === item));
      renderOverlay();
    }
  }

  function onListDblClick(e) {
    const item = e.target.closest(".csp-item");
    if (item && !e.target.closest("input")) {
      const s = st.seedlings[Number(item.getAttribute("data-idx"))];
      s && centerOn(s.lat, s.lon);
    }
  }

  mapEl.addEventListener("pointerdown", onPointerDown);
  mapEl.addEventListener("pointermove", onPointerMove);
  mapEl.addEventListener("pointerup", onPointerUp);
  mapEl.addEventListener("pointercancel", () => (st.drag = null));
  mapEl.addEventListener("pointerleave", () => {
    if (!st.drag) {
      cursorGeo = null;
      renderReadout();
    }
  });
  mapEl.addEventListener("contextmenu", onContextMenu);
  mapEl.addEventListener("wheel", onWheel, { passive: false });
  mapEl.addEventListener("keydown", onKeyDown);
  root.addEventListener("click", onClick);
  el("list").addEventListener("change", onListChange);
  el("list").addEventListener("click", onListClick);
  el("list").addEventListener("dblclick", onListDblClick);
  el("list").addEventListener("keydown", (e) => e.key === "Enter" && e.target.blur());

  const resizeObserver = new ResizeObserver(() => {
    root.classList.toggle("narrow", panel.clientWidth < 700);
    st.size = { w: Math.max(1, mapEl.clientWidth), h: Math.max(1, mapEl.clientHeight) };
    scheduleMap();
  });
  resizeObserver.observe(panel);
  resizeObserver.observe(mapEl);

  // ---------------------------------------------------------- settings

  function updateSettingsEditor() {
    if (typeof context.updatePanelSettingsEditor !== "function") {
      return;
    }
    const s = settings;
    const str = (label, key, help) => ({ label, input: "string", value: s[key], help });
    context.updatePanelSettingsEditor({
      actionHandler: (action) => {
        if (action.action !== "update") {
          return;
        }
        const { path, value } = action.payload;
        const key = path[path.length - 1];
        if (!(key in DEFAULT_SETTINGS)) {
          return;
        }
        settings[key] = value == null ? DEFAULT_SETTINGS[key] : value;
        if (key === "tileUrl" || key === "maxNativeZoom") {
          tiles.forEach((img) => img.remove());
          tiles.clear();
          st.tileErrors = st.tileLoads = 0;
        }
        syncAdvertisements();
        syncSubscriptions();
        updateSettingsEditor();
        renderList();
        renderButtons();
        scheduleMap();
        saveSoon();
      },
      nodes: {
        publish: {
          label: "Publishing",
          fields: {
            planTopic: str("Plan topic", "planTopic"),
            sendMode: { label: "Send mode first", input: "boolean", value: s.sendMode },
            modeTopic: str("Mode topic", "modeTopic"),
            modeLevel: {
              label: "Mode level",
              input: "select",
              value: Number(s.modeLevel),
              options: MODE_NAMES.map((name, i) => ({ label: `${name} (${i})`, value: i })),
            },
            speciesPrefix: str("Species ID prefix", "speciesPrefix"),
          },
        },
        subscribe: {
          label: "Status topics",
          fields: {
            robotTopic: str("Robot GPS (NavSatFix)", "robotTopic"),
            remainingTopic: str("Robot plan", "remainingTopic"),
            currentModeTopic: str("Current mode", "currentModeTopic"),
            launcherNs: str("Launcher namespace", "launcherNs", "Services <ns>/start, <ns>/stop; topic <ns>/status"),
          },
        },
        map: {
          label: "Map",
          fields: {
            tileUrl: str("Tile URL", "tileUrl", "{z}/{x}/{y} template"),
            maxNativeZoom: { label: "Max tile zoom", input: "number", value: Number(s.maxNativeZoom), min: 1, max: 22, step: 1 },
          },
        },
      },
    });
  }

  // ---------------------------------------------------------- render loop

  function applyTheme(scheme) {
    root.classList.toggle("light", scheme === "light");
  }

  context.onRender = (renderState, done) => {
    applyTheme(renderState.colorScheme);
    for (const ev of renderState.currentFrame || []) {
      handleMessage(ev);
    }
    done();
  };
  context.watch("currentFrame");
  context.watch("colorScheme");

  const statusTimer = setInterval(() => {
    renderStatus();
    renderButtons();
    if (st.robot) {
      renderOverlay(); // refresh robot staleness
    }
  }, 1000);

  syncAdvertisements();
  syncSubscriptions();
  updateSettingsEditor();
  renderList();
  renderButtons();
  renderStatus();
  scheduleMap();

  return () => {
    clearInterval(statusTimer);
    clearTimeout(saveTimer);
    cancelAnimationFrame(mapFrame);
    resizeObserver.disconnect();
    for (const topic of Object.keys(st.advertised)) {
      try {
        context.unadvertise && context.unadvertise(topic);
      } catch (e) {
        // ignore
      }
    }
    panel.innerHTML = "";
  };
}

module.exports.activate = function activate(extensionContext) {
  extensionContext.registerPanel({ name: PANEL_NAME, initPanel });
};

// Exposed for offline tests (not used by Foxglove).
module.exports._test = { parseCoordinates, project, unproject, distanceM };
