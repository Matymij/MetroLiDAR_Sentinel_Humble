import * as THREE from 'three';
import { CSS2DRenderer, CSS2DObject }
  from 'three/addons/renderers/CSS2DRenderer.js';

const CFG = {
  bag: '/bags/cloud_with_fake_obj',
  maxPts: 10000,
  envelope: { halfWidth: 1.35, zBottom: 0.20, zTop: 3.40, length: 220 },
};

const zoneColor = {
  INSIDE: 0xff2a3e, RAIL: 0xff8a4d, NEAR: 0xffb84d, ABOVE: 0xd96bff,
  OUTSIDE: 0x4d7dff, BELOW: 0x666666,
};
const CRITICAL_ZONES = new Set(['INSIDE', 'RAIL', 'NEAR', 'ABOVE']);

// ---------- Renderers ----------
const canvas = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(devicePixelRatio);
renderer.setSize(innerWidth, innerHeight);
renderer.setClearColor(0x050810);

const labelRenderer = new CSS2DRenderer();
labelRenderer.setSize(innerWidth, innerHeight);
labelRenderer.domElement.style.position = 'absolute';
labelRenderer.domElement.style.top = '0';
labelRenderer.domElement.style.pointerEvents = 'none';
document.body.appendChild(labelRenderer.domElement);

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x050810, 80, 280);

const camera = new THREE.PerspectiveCamera(70, innerWidth / innerHeight, 0.1, 500);
camera.position.set(0, 1.075, 0);
camera.lookAt(80, 1.075, 0);

// ---------- Static silver rails ----------
const railMat = new THREE.MeshStandardMaterial({
  color: 0xb8c0c8, metalness: 0.85, roughness: 0.35,
});
const railGeo = new THREE.BoxGeometry(220, 0.06, 0.15);
for (const z of [-0.7175, 0.7175]) {
  const r = new THREE.Mesh(railGeo, railMat);
  r.position.set(110, 0.03, z);
  scene.add(r);
}

// ---------- Ground grid ----------
const grid = new THREE.GridHelper(240, 60, 0x0a5060, 0x072a35);
grid.position.set(120, -0.05, 0);
grid.rotateZ(Math.PI / 2);
grid.rotateY(Math.PI / 2);
scene.add(grid);

// ---------- Train envelope ----------
const env = CFG.envelope;
const envW = env.halfWidth * 2;
const envH = env.zTop - env.zBottom;
const envGeo = new THREE.BoxGeometry(env.length, envH, envW);
const envEdges = new THREE.EdgesGeometry(envGeo);
const envMat = new THREE.LineBasicMaterial({
  color: 0x00e5a0, transparent: true, opacity: 0.25,
});
const envBox = new THREE.LineSegments(envEdges, envMat);
envBox.position.set(env.length / 2, env.zBottom + envH / 2, 0);
scene.add(envBox);

const envFloorGeo = new THREE.PlaneGeometry(env.length, envW);
const envFloorMat = new THREE.MeshBasicMaterial({
  color: 0x00e5a0, transparent: true, opacity: 0.05, side: THREE.DoubleSide,
});
const envFloorMesh = new THREE.Mesh(envFloorGeo, envFloorMat);
envFloorMesh.rotation.x = -Math.PI / 2;
envFloorMesh.position.set(env.length / 2, env.zBottom, 0);
scene.add(envFloorMesh);

// ---------- Lights ----------
scene.add(new THREE.AmbientLight(0xa0fff0, 0.7));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.75);
dirLight.position.set(20, 20, 0);
scene.add(dirLight);
const fillLight = new THREE.DirectionalLight(0x00e5a0, 0.3);
fillLight.position.set(-20, 10, 0);
scene.add(fillLight);

// ---------- Point cloud ----------
const pointsGeom = new THREE.BufferGeometry();
pointsGeom.setAttribute('position',
  new THREE.BufferAttribute(new Float32Array(50000 * 3), 3));
const pointsMat = new THREE.PointsMaterial({
  color: 0x00ffb0, size: 0.09, sizeAttenuation: true,
  transparent: true, opacity: 0.9,
});
const points = new THREE.Points(pointsGeom, pointsMat);
points.frustumCulled = false;
scene.add(points);

// ---------- Bbox pool (wireframe) ----------
const bboxGroup = new THREE.Group();
scene.add(bboxGroup);
const bboxPool = [];

function getBox(i) {
  while (bboxPool.length <= i) {
    const g = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
    const m = new THREE.LineBasicMaterial({
      color: 0xff2a3e, transparent: true, opacity: 0.95,
    });
    const box = new THREE.LineSegments(g, m);
    box.visible = false;
    bboxGroup.add(box);
    bboxPool.push(box);
  }
  return bboxPool[i];
}

function resetBoxes(n) {
  for (let i = 0; i < bboxPool.length; i++) {
    bboxPool[i].visible = i < n;
  }
}

// ---------- Solid boxes pool ----------
const solidGroup = new THREE.Group();
scene.add(solidGroup);
const solidPool = [];

function getSolid(i) {
  while (solidPool.length <= i) {
    const m = new THREE.MeshBasicMaterial({
      color: 0xff2a3e, transparent: true, opacity: 0.12, side: THREE.DoubleSide,
    });
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), m);
    mesh.visible = false;
    solidGroup.add(mesh);
    solidPool.push(mesh);
  }
  return solidPool[i];
}

function resetSolids(n) {
  for (let i = 0; i < solidPool.length; i++) solidPool[i].visible = i < n;
}

// ---------- Vertical pillars ----------
const pillarGroup = new THREE.Group();
scene.add(pillarGroup);
const pillarPool = [];

function getPillar(i) {
  while (pillarPool.length <= i) {
    const m = new THREE.LineBasicMaterial({
      color: 0xffffff, transparent: true, opacity: 0.35,
    });
    const g = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 1, 0),
    ]);
    const line = new THREE.Line(g, m);
    line.visible = false;
    pillarGroup.add(line);
    pillarPool.push(line);
  }
  return pillarPool[i];
}

function resetPillars(n) {
  for (let i = 0; i < pillarPool.length; i++) pillarPool[i].visible = i < n;
}

// ---------- Halo around closest critical ----------
const haloGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
const haloMat = new THREE.LineBasicMaterial({
  color: 0xffffff, transparent: true, opacity: 0.85,
});
const halo = new THREE.LineSegments(haloGeo, haloMat);
halo.visible = false;
scene.add(halo);

// ---------- Single subtle label ----------
const closestDiv = document.createElement('div');
closestDiv.className = 'closest-label';
const closestLabel = new CSS2DObject(closestDiv);
closestLabel.visible = false;
scene.add(closestLabel);

// ---------- State ----------
let criticalTime = 0;
let safeDist = 100;

function updatePoints(arr) {
  const pos = pointsGeom.attributes.position.array;
  const n = Math.min(arr.length, pos.length / 3);
  for (let i = 0; i < n; i++) {
    const p = arr[i];
    pos[i * 3 + 0] = p[0];
    pos[i * 3 + 1] = p[2];
    pos[i * 3 + 2] = p[1];
  }
  pointsGeom.setDrawRange(0, n);
  pointsGeom.attributes.position.needsUpdate = true;
}

function updateBoxes(dets) {
  resetBoxes(dets.length);
  resetSolids(dets.length);
  resetPillars(dets.length);

  let closest = null;
  for (const d of dets) {
    if (CRITICAL_ZONES.has(d.zone)) {
      if (!closest || d.dist < closest.dist) closest = d;
    }
  }

  for (let i = 0; i < dets.length; i++) {
    const d = dets[i];
    const col = zoneColor[d.zone] || 0xffffff;
    const sx = Math.max(d.sx, 0.08);
    const sy = Math.max(d.sy, 0.08);
    const sz = Math.max(d.sz, 0.08);
    const isClosest = (d === closest);

    const box = getBox(i);
    box.position.set(d.cx, d.cz, d.cy);
    box.scale.set(sx, sz, sy);
    box.material.color.setHex(col);
    box.material.opacity = isClosest ? 1.0 : 0.7;
    box.visible = true;

    const solid = getSolid(i);
    solid.position.set(d.cx, d.cz, d.cy);
    solid.scale.set(sx, sz, sy);
    solid.material.color.setHex(col);
    solid.material.opacity = isClosest ? 0.25 : 0.10;
    solid.visible = true;

    const pil = getPillar(i);
    const zBottom = Math.max(0.0, d.cz - d.sz / 2);
    pil.geometry.setFromPoints([
      new THREE.Vector3(d.cx, 0.0, d.cy),
      new THREE.Vector3(d.cx, zBottom, d.cy),
    ]);
    pil.material.color.setHex(col);
    pil.visible = true;
  }

  if (closest) {
    halo.position.set(closest.cx, closest.cz, closest.cy);
    halo.scale.set(
      Math.max(closest.sx, 0.15) * 1.15,
      Math.max(closest.sz, 0.15) * 1.15,
      Math.max(closest.sy, 0.15) * 1.15,
    );
    halo.visible = true;

    closestLabel.position.set(
      closest.cx,
      closest.cz + Math.max(closest.sz, 0.3) / 2 + 0.25,
      closest.cy,
    );
    closestDiv.textContent = `${closest.dist.toFixed(1)} м`;
    const col = zoneColor[closest.zone] || 0xffffff;
    closestDiv.style.color = '#' + col.toString(16).padStart(6, '0');
    closestDiv.style.borderColor = '#' + col.toString(16).padStart(6, '0');
    closestLabel.visible = true;
  } else {
    halo.visible = false;
    closestLabel.visible = false;
  }
}

// ---------- UI ----------
const el = (id) => document.getElementById(id);
const log = el('log');

const logRows = new Map();
let frameIdx = 0;
const LOG_MAX = 25;

function renderLiveLog(dets) {
  frameIdx++;
  const seen = new Set();

  for (const d of dets) {
    if (d.id < 0) continue;
    seen.add(d.id);

    let row = logRows.get(d.id);
    if (!row) {
      const div = document.createElement('div');
      div.className = 'log-entry';
      div.innerHTML =
        `<span class="id">#${d.id}</span>` +
        `<span class="zone"></span>` +
        `<span class="dist"></span>` +
        `<span class="hits"></span>`;
      log.appendChild(div);
      row = { el: div, lastSeen: frameIdx };
      logRows.set(d.id, row);
    }
    row.lastSeen = frameIdx;

    const zoneEl = row.el.querySelector('.zone');
    const distEl = row.el.querySelector('.dist');
    const hitsEl = row.el.querySelector('.hits');

    zoneEl.className = 'zone ' + d.zone;
    zoneEl.textContent = d.zone;

    const isCrit = CRITICAL_ZONES.has(d.zone);
    const distCls = isCrit && d.dist < 100 ? 'crit' : (isCrit ? 'warn' : '');
    distEl.className = 'dist ' + distCls;
    distEl.textContent = `${d.dist.toFixed(1)} м`;

    hitsEl.textContent = `hits:${d.hits}`;
    hitsEl.className = d.hits === 0 ? 'hits zero' : 'hits';
  }

  for (const [id, row] of logRows.entries()) {
    if (!seen.has(id) && frameIdx - row.lastSeen > 30) {
      row.el.remove();
      logRows.delete(id);
    }
  }

  while (logRows.size > LOG_MAX) {
    let oldestId = null, oldestSeen = Infinity;
    for (const [id, r] of logRows.entries()) {
      if (r.lastSeen < oldestSeen) { oldestSeen = r.lastSeen; oldestId = id; }
    }
    if (oldestId !== null) {
      logRows.get(oldestId).el.remove();
      logRows.delete(oldestId);
    }
  }

  const empty = log.querySelector('.log-empty');
  if (logRows.size === 0) {
    if (!empty) {
      const div = document.createElement('div');
      div.className = 'log-empty';
      div.textContent = '— нет детекций —';
      log.appendChild(div);
    }
  } else if (empty) {
    empty.remove();
  }
}

function updateUI(payload) {
  const t = payload.telemetry;

  el('frame-counter').textContent = `frame ${payload.frame}`;
  el('n-anchors').textContent = payload.detections.length;
  el('n-critical').textContent = t.n_critical ?? 0;

  el('speed').textContent = `${t.speed_kmh.toFixed(1)} км/ч`;
  el('dist').textContent = `${t.distance_m.toFixed(1)} м`;

  el('max-dist').textContent = (t.max_dist != null)
    ? `${t.max_dist.toFixed(1)} м` : '—';

  if (t.min_dist != null) {
    el('min-dist').textContent = `${t.min_dist.toFixed(1)} м`;
    el('min-dist').className = 'val ' + (t.status === 'CRITICAL' ? 'crit' : 'warn');
    el('to-obs').textContent = `${t.min_dist.toFixed(1)} м`;
    el('to-obs').className = 'val ' + (t.status === 'CRITICAL' ? 'crit' : 'warn');
  } else {
    el('min-dist').textContent = 'ЧИСТО';
    el('min-dist').className = 'val ok';
    el('to-obs').textContent = 'ЧИСТО';
    el('to-obs').className = 'val ok';
  }

  if (t.ttc_s != null) {
    el('ttc').textContent = `${t.ttc_s.toFixed(1)} с`;
    el('ttc').className = 'val ' + (t.ttc_s < 5 ? 'crit' : t.ttc_s < 10 ? 'warn' : 'ok');
  } else {
    el('ttc').textContent = '—';
    el('ttc').className = 'val';
  }

  const isCritical = t.min_dist != null && t.min_dist < safeDist;
  const isWarning = t.min_dist != null && t.min_dist < safeDist * 1.8;

  const stEl = el('path-status');
  stEl.className = 'val ' + (isCritical ? 'crit' : isWarning ? 'warn' : 'ok');
  stEl.textContent = isCritical ? 'ОПАСНО' : isWarning ? 'ВНИМАНИЕ' : 'ЧИСТО';

  const dot = el('emergency');
  if (dot) dot.className = 'brake-dot' + (isCritical ? ' active' : '');

  const st01 = el('status-m01');
  if (st01) {
    st01.className = 'cup-status ' + (isCritical ? 'bad' : 'ok');
    st01.textContent = isCritical ? 'ОПАСНО' : 'НОРМА';
  }

  renderLiveLog(payload.detections);
}

// ---------- FPS tracker ----------
let lastFrameTime = performance.now();
let fpsAccum = 0, fpsCount = 0;
const fpsEl = document.getElementById('fps');

function trackFps() {
  const now = performance.now();
  const dt = now - lastFrameTime;
  lastFrameTime = now;
  if (dt > 0 && dt < 1000) {
    fpsAccum += 1000 / dt;
    fpsCount++;
    if (fpsCount >= 15) {
      fpsEl.textContent = `fps ${(fpsAccum / fpsCount).toFixed(0)}`;
      fpsAccum = 0;
      fpsCount = 0;
    }
  }
}

// ---------- WebSocket ----------
let ws = null;
let paused = false;

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => {
    el('link').className = 'val ok';
    el('link').textContent = 'подключено';
  };
  ws.onclose = () => {
    el('link').className = 'val warn';
    el('link').textContent = 'offline';
    setTimeout(connect, 2000);
  };
  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);

    if (msg.event === 'started') {
      logRows.clear();
      log.querySelectorAll('.log-entry, .log-empty').forEach(n => n.remove());
      frameIdx = 0;
      paused = false;
      el('btn-pause').disabled = false;
      el('btn-pause').textContent = 'II ПАУЗА';
      el('btn-start').disabled = true;
      return;
    }
    if (msg.event === 'paused') {
      paused = true;
      el('btn-pause').textContent = '▶ ПРОДОЛЖИТЬ';
      return;
    }
    if (msg.event === 'resumed') {
      paused = false;
      el('btn-pause').textContent = 'II ПАУЗА';
      return;
    }
    if (msg.event === 'finished' || msg.event === 'stopped') {
      paused = false;
      el('btn-pause').disabled = true;
      el('btn-pause').textContent = 'II ПАУЗА';
      el('btn-start').disabled = false;
      return;
    }

    updatePoints(msg.points);
    updateBoxes(msg.detections);
    updateUI(msg);
  };
}
connect();

el('btn-start').onclick = () => {
  if (!ws || ws.readyState !== 1) return;
  ws.send(JSON.stringify({ cmd: 'start', bag: CFG.bag }));
};

el('btn-pause').onclick = () => {
  if (!ws || ws.readyState !== 1) return;
  if (paused) {
    ws.send(JSON.stringify({ cmd: 'resume' }));
  } else {
    ws.send(JSON.stringify({ cmd: 'pause' }));
  }
};

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
  labelRenderer.setSize(innerWidth, innerHeight);
});

// ---------- ЦУП sliders ----------
function bindSlider(id, valId, fmt, onchange) {
  const sl = document.getElementById(id);
  const lbl = document.getElementById(valId);
  if (!sl || !lbl) return;
  const update = () => {
    lbl.textContent = fmt(sl.value);
    if (onchange) onchange(sl.value);
  };
  sl.addEventListener('input', update);
  update();
}

bindSlider('sl-points', 'val-points', v => v, v => { CFG.maxPts = parseInt(v); });
bindSlider('sl-safe',   'val-safe',   v => v + 'м', v => { safeDist = parseFloat(v); });
bindSlider('sl-range',  'val-range',  v => v + 'м', v => { scene.fog.far = parseFloat(v); });
bindSlider('sl-width',  'val-width',  v => v + 'м', v => {
  const hw = parseFloat(v) / 2;
  envBox.scale.z = hw / CFG.envelope.halfWidth;
  envFloorMesh.scale.z = hw / CFG.envelope.halfWidth;
});
bindSlider('sl-height', 'val-height', v => v + 'м', v => {
  const h = parseFloat(v);
  const envHCurrent = CFG.envelope.zTop - CFG.envelope.zBottom;
  envBox.scale.y = h / envHCurrent;
});

// ---------- Main loop ----------
function loop(now) {
  requestAnimationFrame(loop);
  criticalTime = (now || 0) / 1000;
  trackFps();

  if (halo.visible) {
    haloMat.opacity = 0.55 + 0.45 * (0.5 + 0.5 * Math.sin(criticalTime * 6));
  }

  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
}
requestAnimationFrame(loop);