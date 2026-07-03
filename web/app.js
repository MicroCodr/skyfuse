// SKYFUSE dashboard — renders the fused tactical picture on a canvas.
'use strict';

const AREA_HALF = 50000;                       // must match config.AREA_HALF
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');

const COLORS = {
  radar: '#4db8ff',
  eo: '#ffb84d',
  adsb: '#b07fff',
  track: '#35e0a1',
  coasting: '#e0c035',
  tentative: '#3a4a60',
  truth: '#ff5d7a',
  grid: '#141c2a',
  ring: '#1a2536',
};

let world = null;

// --- websocket -------------------------------------------------------------

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = e => { world = JSON.parse(e.data); updateTrails(); };
  ws.onclose = () => setTimeout(connect, 1000);

  for (const name of ['radar', 'eo', 'adsb']) {
    document.getElementById(`sen-${name}`).onchange = ev =>
      ws.send(JSON.stringify({ cmd: 'toggle_sensor', sensor: name, enabled: ev.target.checked }));
  }
}
connect();

// --- coordinate transform ----------------------------------------------------

let scale = 1, ox = 0, oy = 0;
function resize() {
  canvas.width = canvas.clientWidth * devicePixelRatio;
  canvas.height = canvas.clientHeight * devicePixelRatio;
  const s = Math.min(canvas.width, canvas.height) / (2 * AREA_HALF) * 0.94;
  scale = s;
  ox = canvas.width / 2;
  oy = canvas.height / 2;
}
addEventListener('resize', resize);
resize();

const sx = x => ox + x * scale;
const sy = y => oy - y * scale;                 // north up

// --- track trails ------------------------------------------------------------
// keep a breadcrumb history per track id so you can see where each track
// has been (also makes the CV filter lagging in turns really obvious)

const TRAIL_LEN = 60;
const trails = new Map();

function updateTrails() {
  if (!world) return;
  const alive = new Set();
  for (const t of world.tracks) {
    if (t.status === 'tentative') continue;
    alive.add(t.id);
    let pts = trails.get(t.id);
    if (!pts) { pts = []; trails.set(t.id, pts); }
    const last = pts[pts.length - 1];
    // only add a point if the track actually moved a bit
    if (!last || Math.hypot(t.x - last[0], t.y - last[1]) > 40) {
      pts.push([t.x, t.y]);
      if (pts.length > TRAIL_LEN) pts.shift();
    }
  }
  // forget trails of tracks that got dropped
  for (const id of trails.keys()) {
    if (!alive.has(id)) trails.delete(id);
  }
}

function drawTrails() {
  ctx.lineWidth = 1 * devicePixelRatio;
  for (const pts of trails.values()) {
    for (let i = 1; i < pts.length; i++) {
      ctx.globalAlpha = (i / pts.length) * 0.45;   // fade out the old end
      ctx.strokeStyle = COLORS.track;
      ctx.beginPath();
      ctx.moveTo(sx(pts[i - 1][0]), sy(pts[i - 1][1]));
      ctx.lineTo(sx(pts[i][0]), sy(pts[i][1]));
      ctx.stroke();
    }
  }
  ctx.globalAlpha = 1;
}

// --- rendering ---------------------------------------------------------------

function draw() {
  requestAnimationFrame(draw);
  ctx.fillStyle = '#0a0e14';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  if (!world) return;

  drawSensorSites();
  if (document.getElementById('lay-dets').checked) drawDetections();
  if (document.getElementById('lay-trails').checked) drawTrails();
  if (document.getElementById('lay-truth').checked) drawTruth();
  drawTracks();
  updatePanel();
}
requestAnimationFrame(draw);

function drawGrid() {
  ctx.strokeStyle = COLORS.grid;
  ctx.lineWidth = 1;
  const step = 10000 * scale;
  for (let x = ox % step; x < canvas.width; x += step) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
  }
  for (let y = oy % step; y < canvas.height; y += step) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
  }
  // surveillance boundary
  ctx.strokeStyle = COLORS.ring;
  ctx.lineWidth = 2;
  ctx.strokeRect(sx(-AREA_HALF), sy(AREA_HALF), 2 * AREA_HALF * scale, 2 * AREA_HALF * scale);
}

function drawSensorSites() {
  if (!world.sensors) return;
  // radar range rings
  const radar = world.sensors.radar;
  if (radar && radar.pos[0] !== null) {
    const [rx, ry] = radar.pos;
    ctx.strokeStyle = COLORS.ring;
    ctx.lineWidth = 1;
    for (const r of [25000, 50000, 75000]) {
      ctx.beginPath();
      ctx.arc(sx(rx), sy(ry), r * scale, 0, 7);
      ctx.stroke();
    }
    site(rx, ry, COLORS.radar, radar.enabled, 'RADAR');
  }
  const eo = world.sensors.eo;
  if (eo && eo.pos[0] !== null) site(eo.pos[0], eo.pos[1], COLORS.eo, eo.enabled, 'EO/IR');
}

function site(x, y, color, enabled, label) {
  ctx.fillStyle = enabled ? color : '#333c4c';
  ctx.beginPath();
  ctx.moveTo(sx(x), sy(y) - 7);
  ctx.lineTo(sx(x) + 6, sy(y) + 5);
  ctx.lineTo(sx(x) - 6, sy(y) + 5);
  ctx.fill();
  ctx.font = `${10 * devicePixelRatio}px monospace`;
  ctx.fillText(label, sx(x) + 9, sy(y) + 4);
}

function drawDetections() {
  const now = world.time;
  for (const d of world.detections) {
    const age = now - d.t;
    const alpha = Math.max(0, 1 - age / 4);
    ctx.globalAlpha = alpha * 0.8;
    ctx.fillStyle = COLORS[d.sensor] || '#888';
    ctx.beginPath();
    ctx.arc(sx(d.x), sy(d.y), 2.2 * devicePixelRatio, 0, 7);
    ctx.fill();
  }
  ctx.globalAlpha = 1;
}

function drawTruth() {
  ctx.strokeStyle = COLORS.truth;
  ctx.lineWidth = 1.5;
  for (const a of world.truth) {
    const x = sx(a.x), y = sy(a.y);
    ctx.beginPath();
    ctx.arc(x, y, 5 * devicePixelRatio, 0, 7);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(sx(a.x + a.vx * 15), sy(a.y + a.vy * 15));
    ctx.stroke();
  }
}

function drawTracks() {
  const showCov = document.getElementById('lay-cov').checked;
  ctx.font = `${10 * devicePixelRatio}px monospace`;

  for (const t of world.tracks) {
    if (t.status === 'tentative') {
      ctx.fillStyle = COLORS.tentative;
      ctx.fillRect(sx(t.x) - 2, sy(t.y) - 2, 4, 4);
      continue;
    }
    const color = t.status === 'coasting' ? COLORS.coasting : COLORS.track;
    const x = sx(t.x), y = sy(t.y);

    if (showCov) drawEllipse(t, color);

    // velocity leader
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5 * devicePixelRatio;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(sx(t.x + t.vx * 20), sy(t.y + t.vy * 20));
    ctx.stroke();

    // diamond symbol
    const r = 6 * devicePixelRatio;
    ctx.beginPath();
    ctx.moveTo(x, y - r); ctx.lineTo(x + r, y); ctx.lineTo(x, y + r); ctx.lineTo(x - r, y);
    ctx.closePath();
    ctx.stroke();

    ctx.fillStyle = color;
    ctx.fillText(`TK${String(t.id).padStart(3, '0')}`, x + r + 3, y - 3);
  }
}

function drawEllipse(t, color) {
  // 2-sigma ellipse from covariance [pxx, pxy, pyy]
  const [pxx, pxy, pyy] = t.cov;
  const tr2 = (pxx + pyy) / 2;
  const det = Math.sqrt(Math.max(0, ((pxx - pyy) / 2) ** 2 + pxy * pxy));
  const l1 = Math.max(1, tr2 + det), l2 = Math.max(1, tr2 - det);
  const ang = 0.5 * Math.atan2(2 * pxy, pxx - pyy);

  ctx.strokeStyle = color;
  ctx.globalAlpha = 0.35;
  ctx.lineWidth = 1 * devicePixelRatio;
  ctx.beginPath();
  // canvas y is flipped, so the rotation angle flips too
  ctx.ellipse(sx(t.x), sy(t.y), 2 * Math.sqrt(l1) * scale, 2 * Math.sqrt(l2) * scale, -ang, 0, 7);
  ctx.stroke();
  ctx.globalAlpha = 1;
}

// --- side panel ----------------------------------------------------------------

function updatePanel() {
  document.getElementById('hud-time').textContent = `T+${world.time.toFixed(1)}s`;
  const m = world.metrics;
  document.getElementById('m-rmse').textContent = `${m.rmse} m`;
  document.getElementById('m-matched').textContent = `${m.matched}/${world.truth.length}`;
  document.getElementById('m-missed').textContent = m.missed_targets;
  document.getElementById('m-false').textContent = m.false_tracks;

  const tbody = document.querySelector('#tracks tbody');
  const rows = world.tracks
    .filter(t => t.status !== 'tentative')
    .sort((a, b) => a.id - b.id)
    .map(t => {
      const kts = Math.round(Math.hypot(t.vx, t.vy) * 1.944);
      const hdg = Math.round((90 - Math.atan2(t.vy, t.vx) * 180 / Math.PI + 360) % 360);
      const src = ['radar', 'eo', 'adsb']
        .filter(s => t.sensors[s]).map(s => s[0].toUpperCase()).join('');
      return `<tr class="${t.status}"><td>TK${String(t.id).padStart(3, '0')}</td>` +
             `<td>${t.status === 'coasting' ? 'CST' : 'CFM'}</td>` +
             `<td>${kts}</td><td>${String(hdg).padStart(3, '0')}</td><td>${src}</td></tr>`;
    });
  tbody.innerHTML = rows.join('');
}
