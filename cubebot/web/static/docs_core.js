/* Core helpers of the experiments docs page: numbers, matrices, charts, live poller, 3D scene with the real M3 Pro model. */
const $ = (s, r = document) => r.querySelector(s);
const D2R = Math.PI / 180;
const f = (v, n = 3) => (v === null || v === undefined || Number.isNaN(v)) ? '–' : Number(v).toFixed(n);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim() || '#888';
const COLORS = () => ['--c1', '--c2', '--c3', '--c4', '--c5', '--c6'].map(css);

const mat = (M, n = 3, hi = null) => `<span class="mat">${M.map((r, i) => `<div style="grid-template-columns:repeat(${r.length},auto)">${r.map((v, j) =>
  `<span class="${hi && hi(i, j) ? 'hi' : ''}">${f(v, n)}</span>`).join('')}</div>`).join('')}</span>`;
const vec = (v, n = 3) => mat(v.map(x => [x]), n);
async function api(path, body) {
  const r = await fetch(path, body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  if (!r.ok) throw new Error(r.statusText);
  return r.json();
}
function math(node) {
  if (window.renderMathInElement) renderMathInElement(node, { delimiters: [{ left: '$$', right: '$$', display: true }, { left: '$', right: '$', display: false }], throwOnError: false });
}
// Live sections rebuild their HTML; render any TeX they insert (a few blocks forgot to call math()).
// Only the changed blocks are rendered, with the observer paused so its own changes are not re-observed.
const mathObserver = new MutationObserver(records => {
  const targets = new Set(records.map(r => r.target).filter(t => t.nodeType === 1 && t.textContent.includes("$")));
  if (!targets.size || !window.renderMathInElement) return;
  mathObserver.disconnect();
  try { targets.forEach(math); } finally { mathObserver.observe(document.body, { childList: true, subtree: true }); }
});
if (document.body) mathObserver.observe(document.body, { childList: true, subtree: true });
else document.addEventListener("DOMContentLoaded", () => mathObserver.observe(document.body, { childList: true, subtree: true }));
function lazy(el, fn) {
  const io = new IntersectionObserver(es => { if (es[0].isIntersecting) { io.disconnect(); Promise.resolve().then(fn).catch(e => console.error(el.id, e)); } }, { rootMargin: '300px' });
  io.observe(el);
}
function ctl(id, label, min, max, step, value, unit = '') {
  return `<label for="${id}">${label}</label><input type="range" id="${id}" min="${min}" max="${max}" step="${step}" value="${value}"><output id="${id}-o">${value}${unit}</output>`;
}
function bindCtl(ids, fn, unit = {}) {
  ids.forEach(id => { const e = $('#' + id), o = $('#' + id + '-o'); const upd = () => { o.textContent = (+e.value).toString().replace(/(\.\d{3})\d+/, '$1') + (unit[id] || ''); }; e.addEventListener('input', () => { upd(); fn(); }); upd(); });
}

/* ------------------------------------------------------------------ line chart (series: {name,color,x,y}) */
function lineChart(canvas, series, o = {}) {
  const d = window.devicePixelRatio || 1, r = canvas.getBoundingClientRect();
  if (r.width < 10) return;
  canvas.width = r.width * d; canvas.height = r.height * d;
  const g = canvas.getContext('2d'), W = canvas.width, H = canvas.height, pad = { l: 44 * d, r: 8 * d, t: 8 * d, b: 20 * d };
  g.clearRect(0, 0, W, H);
  const xs = series.flatMap(s => s.x.filter(v => v != null)), ys = series.flatMap(s => s.y.filter(v => v != null && isFinite(v)));
  if (!xs.length && o.x0 === undefined) { g.fillStyle = css('--muted'); g.font = `${12 * d}px sans-serif`; g.fillText(o.empty || 'нет данных', pad.l + 8 * d, H / 2); return; }
  const x0 = o.x0 ?? Math.min(...xs), x1 = o.x1 ?? Math.max(...xs);
  let lo = o.y0 ?? (ys.length ? Math.min(...ys) : 0), hi = o.y1 ?? (ys.length ? Math.max(...ys) : 1);
  (o.hlines || []).forEach(h => { if (o.y0 === undefined) lo = Math.min(lo, h.y); if (o.y1 === undefined) hi = Math.max(hi, h.y); });
  if (hi - lo < 1e-9) { hi += 0.5; lo -= 0.5; }
  const span = hi - lo, X = x => pad.l + (x - x0) / ((x1 - x0) || 1) * (W - pad.l - pad.r), Y = y => H - pad.b - (y - lo) / span * (H - pad.t - pad.b);
  g.strokeStyle = 'rgba(150,170,200,.28)'; g.fillStyle = css('--muted'); g.font = `${10.5 * d}px sans-serif`; g.lineWidth = d;
  for (let i = 0; i <= 4; i++) { const y = lo + span * i / 4; g.beginPath(); g.moveTo(pad.l, Y(y)); g.lineTo(W - pad.r, Y(y)); g.stroke(); g.fillText(Math.abs(span) < 5 ? y.toFixed(2) : y.toFixed(0), 3 * d, Y(y) + 4 * d); }
  (o.bands || []).forEach(b => { g.fillStyle = b.color + '33'; g.fillRect(X(b.from), pad.t, Math.max(1, X(b.to) - X(b.from)), H - pad.t - pad.b); });
  (o.hlines || []).forEach(h => { g.strokeStyle = h.color || css('--warn'); g.setLineDash([5 * d, 4 * d]); g.beginPath(); g.moveTo(pad.l, Y(h.y)); g.lineTo(W - pad.r, Y(h.y)); g.stroke(); g.setLineDash([]); g.fillStyle = h.color || css('--warn'); g.fillText(h.label || '', pad.l + 4 * d, Y(h.y) - 3 * d); });
  series.forEach(s => {
    g.strokeStyle = s.color; g.lineWidth = 1.8 * d; g.beginPath(); let pen = false;
    s.y.forEach((y, i) => { if (y == null || !isFinite(y)) { pen = false; return; } const x = X(s.x[i]); pen ? g.lineTo(x, Y(y)) : g.moveTo(x, Y(y)); pen = true; });
    g.stroke();
  });
  g.fillStyle = css('--muted'); g.fillText(o.xLabel || '', W - 110 * d, H - 4 * d);
}
const legend = series => `<div class="legend">${series.map(s => `<span><i style="background:${s.color}"></i>${s.name}</span>`).join('')}</div>`;

/* ------------------------------------------------------------------ live data poller */
const Live = {
  last: null, ok: false, since: 0, subs: [], hist: { t: [], pose: [], cmd: [], armCmd: [], armMeas: [], raw: [], battery: [], wheel: [] },
  follow: [], gest: [], t0: null,
  on(fn) { this.subs.push(fn); },
  async tick() {
    if (document.hidden) return;
    try {
      const L = await api('/api/exp/live?since=' + this.since);
      this.ok = true; this.last = L; this.age = Date.now();
      if (this.t0 === null) this.t0 = L.t;
      const h = this.hist, t = L.t;
      h.t.push(t); h.pose.push(L.pose); h.cmd.push(L.cmd); h.armCmd.push(L.arm.cmd); h.armMeas.push(L.arm.measured || L.arm.cmd);
      h.raw.push(L.raw); h.battery.push(L.battery);
      while (h.t.length && t - h.t[0] > 60) for (const k in h) h[k].shift();
      if (L.follow) { this.follow.push(...L.follow.hist); this.since = Math.max(this.since, ...(L.follow.hist.map(x => x[0]).concat([0]))); }
      if (L.gesture) { this.gest.push(...L.gesture.hist); }
      const gmax = Math.max(0, ...((L.gesture ? L.gesture.hist : []).map(x => x[0])));
      const fmax = Math.max(0, ...((L.follow ? L.follow.hist : []).map(x => x[0])));
      this.since = Math.max(this.since, gmax, fmax);
      this.follow = this.follow.filter(x => t - x[0] < 60); this.gest = this.gest.filter(x => t - x[0] < 60);
      this.subs.forEach(fn => { try { fn(L); } catch (e) { console.error(e); } });
    } catch (e) { this.ok = false; }
    const el = $('#live-dot'); if (el) { el.className = 'dot ' + (this.ok ? 'ok' : 'bad'); $('#live-txt').textContent = this.ok ? 'живые данные с робота' : 'нет связи с роботом'; }
  },
  start() { this.tick(); setInterval(() => this.tick(), 250); },
};
/* seconds-ago x axis for ring buffers */
const ago = (ts, now) => ts.map(t => t - now);

/* ------------------------------------------------------------------ 3D scene with the real robot model */
const URDF = { data: null, geo: {}, promise: null };
const LINK_COLOR = { base_link: 0x7d8ba1, arm1: 0xe6892e, arm2: 0xe6892e, arm3: 0xe6892e, arm4: 0xe6892e, arm5: 0x39465a, DCW2: 0x20242c, Camera: 0x20242c,
  Gripping: 0x39465a, arm_base_Link: 0x39465a, lwheel1: 0x2b303a, lwheel2: 0x2b303a, rwheel1: 0x2b303a, rwheel2: 0x2b303a };
/* arm_model.REP103_OPTICAL as a 4x4 (rows [0,0,1],[-1,0,0],[0,-1,0]) */
const OPT = new THREE.Matrix4().set(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1);
const m4of = r => { const m = new THREE.Matrix4(); m.set(r[0][0], r[0][1], r[0][2], r[0][3], r[1][0], r[1][1], r[1][2], r[1][3], r[2][0], r[2][1], r[2][2], r[2][3], 0, 0, 0, 1); return m; };
function originMatrix(xyz, rpy) {
  const R = new THREE.Matrix4().makeRotationZ(rpy[2]).multiply(new THREE.Matrix4().makeRotationY(rpy[1])).multiply(new THREE.Matrix4().makeRotationX(rpy[0]));
  R.setPosition(xyz[0], xyz[1], xyz[2]); return R;
}
const m4rows = M => { const e = M.elements; return [[e[0], e[4], e[8], e[12]], [e[1], e[5], e[9], e[13]], [e[2], e[6], e[10], e[14]], [0, 0, 0, 1]]; };
function servoToQ(s) { return [(90 - s[0]) * D2R, (s[1] - 90) * D2R, (s[2] - 90) * D2R, (s[3] - 90) * D2R, (s[4] - 90) * D2R]; }

function textSprite(text, color = '#fff', size = 0.03) {
  const c = document.createElement('canvas'); c.width = 384; c.height = 96; const g = c.getContext('2d');
  g.font = 'bold 44px -apple-system,Helvetica,sans-serif'; g.textBaseline = 'middle'; g.fillStyle = 'rgba(11,15,22,.7)';
  const w = Math.min(376, g.measureText(text).width + 20); g.fillRect(4, 14, w, 68); g.fillStyle = color; g.fillText(text, 14, 48);
  const tex = new THREE.CanvasTexture(c); tex.minFilter = THREE.LinearFilter;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  sp.scale.set(size * 4, size, 1); sp.center.set(0, 0.5); sp.renderOrder = 10; return sp;
}

class Scene3D {
  constructor(canvas, o = {}) {
    this.canvas = canvas; this.scene = new THREE.Scene(); this.scene.background = new THREE.Color(0x0b0f16);
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 30); this.camera.up.set(0, 0, 1);
    const p = o.cam || [0.7, -0.8, 0.55]; this.camera.position.set(...p);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true }); this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    this.controls = new THREE.OrbitControls(this.camera, canvas); this.controls.target.set(...(o.target || [0.1, 0, 0.15]));
    this.controls.addEventListener('change', () => this.dirty = true);
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.8)); const d = new THREE.DirectionalLight(0xffffff, 0.8); d.position.set(0.5, -0.6, 1); this.scene.add(d);
    const d2 = new THREE.DirectionalLight(0x8899ff, 0.35); d2.position.set(-0.5, 0.6, 0.4); this.scene.add(d2);
    this.world = new THREE.Group(); this.scene.add(this.world);                  // robot moves inside `world` when a scene shows odom frame
    this.dyn = new THREE.Group(); this.scene.add(this.dyn);
    this.links = {}; this.named = {}; this.servo = [90, 90, 90, 90, 90, 90]; this.wheels = [0, 0, 0, 0]; this.dirty = true;
    if (o.floor !== false) {
      const size = o.floor || 1.5, grid = new THREE.GridHelper(size * 2, Math.round(size * 20), 0x3a475c, 0x222c3b); grid.rotation.x = Math.PI / 2; this.scene.add(grid);
    }
    new ResizeObserver(() => this.resize()).observe(canvas); this.resize();
    const loop = () => { if (this.dirty && document.body.contains(canvas)) { this.dirty = false; this.renderer.render(this.scene, this.camera); } requestAnimationFrame(loop); }; loop();
  }
  resize() { const r = this.canvas.getBoundingClientRect(); if (r.width < 4) return; this.renderer.setSize(r.width, r.height, false); this.camera.aspect = r.width / r.height; this.camera.updateProjectionMatrix(); this.dirty = true; }
  static async loadUrdf() {
    if (URDF.promise) return URDF.promise;
    URDF.promise = (async () => {
      URDF.data = await (await fetch('/static/urdf.json')).json();
      const loader = new THREE.STLLoader(), names = Object.values(URDF.data.links).filter(Boolean);
      await Promise.all(names.map(n => new Promise(res => loader.load('/static/meshes/' + n, g => { g.computeVertexNormals(); URDF.geo[n] = g; res(); }, undefined, () => res()))));
      return URDF.data;
    })();
    return URDF.promise;
  }
  async addRobot(o = {}) {
    const u = await Scene3D.loadUrdf(); this.robotOpacity = o.opacity ?? 1; this.urdf = u;
    for (const [name, mesh] of Object.entries(u.links)) {
      const g = new THREE.Group(); g.matrixAutoUpdate = false; this.links[name] = g; this.world.add(g);
      if (!mesh || !URDF.geo[mesh]) continue;
      if (o.hide && o.hide.includes(name)) continue;
      const mat = new THREE.MeshPhongMaterial({ color: LINK_COLOR[name] || 0x888888, transparent: this.robotOpacity < 1, opacity: this.robotOpacity, shininess: 30, side: THREE.DoubleSide });
      g.add(new THREE.Mesh(URDF.geo[mesh], mat));
    }
    this.update(); return this;
  }
  /* kinematic tree from the URDF: arm joints take servo angles, wheel joints take rolling angles */
  update() {
    if (!this.urdf) return;
    const T = { base_footprint: new THREE.Matrix4(), base_link: new THREE.Matrix4() };
    const q = servoToQ(this.servo), armIdx = { arm1_Joint: 0, arm2_Joint: 1, arm3_Joint: 2, arm4_Joiint: 3, arm5_Joint: 4 };
    const wIdx = { lwheel1_Joint: [0, 1], lwheel2_Joint: [2, 1], rwheel1_Joint: [1, -1], rwheel2_Joint: [3, -1] };
    const pending = this.urdf.joints.slice(); let guard = 0;
    while (pending.length && guard++ < 200) {
      for (let i = pending.length - 1; i >= 0; i--) {
        const j = pending[i]; if (!T[j.parent]) continue;
        let M = T[j.parent].clone().multiply(originMatrix(j.xyz, j.rpy));
        let ang = 0;
        if (j.name in armIdx) ang = q[armIdx[j.name]];
        else if (j.name in wIdx) { const [k, s] = wIdx[j.name]; ang = this.wheels[k] * s; }
        if (ang) M.multiply(new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(...j.axis).normalize(), ang));
        T[j.child] = M; pending.splice(i, 1);
      }
    }
    this.T = T;
    for (const [n, g] of Object.entries(this.links)) { if (T[n]) { g.matrix.copy(T[n]); g.matrixWorldNeedsUpdate = true; } }
    // camera optical frame from the same chain: DCW2 link frame times arm_model.REP103_OPTICAL (x right, y down, z forward)
    if (T.DCW2) this.T.camera_optical = T.DCW2.clone().multiply(OPT);
    this.dirty = true;
  }
  setServo(s) { this.servo = s.slice(); this.update(); }
  setWheels(w) { this.wheels = w.slice(); this.update(); }
  setBase(x, y, yaw) { this.world.position.set(x, y, 0); this.world.rotation.set(0, 0, yaw); this.dirty = true; }
  /* helpers: all drawn in the scene frame unless `parent` is given */
  axes(name, size = 0.06, label = '', parent = null) {
    const g = new THREE.Group(); g.matrixAutoUpdate = false;
    [[1, 0, 0, 0xef4444], [0, 1, 0, 0x22c55e], [0, 0, 1, 0x3b82f6]].forEach(([x, y, z, c]) => {
      const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(x * size, y * size, z * size)]);
      g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: c })));
    });
    if (label) { const s = textSprite(label, '#fff', Math.min(0.03, size * 0.25)); s.position.set(size * 0.1, size * 0.1, size * 0.15); g.add(s); }
    (parent || this.scene).add(g); this.named[name] = g; return g;
  }
  setAxes(name, M) { const g = this.named[name]; if (g) { g.matrix.copy(M); g.matrixWorldNeedsUpdate = true; this.dirty = true; } }
  pointCloud(name, color = 0x3b82f6, size = 0.012) {
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 800), 3)); geo.setDrawRange(0, 0);
    const p = new THREE.Points(geo, new THREE.PointsMaterial({ color, size, sizeAttenuation: true })); p.frustumCulled = false; this.scene.add(p); this.named[name] = p; return p;
  }
  setPoints(name, pts, z = 0.125) {
    const p = this.named[name]; if (!p) return; const a = p.geometry.attributes.position.array, n = Math.min(pts.length, 800);
    for (let i = 0; i < n; i++) { a[3 * i] = pts[i][0]; a[3 * i + 1] = pts[i][1]; a[3 * i + 2] = pts[i].length > 2 ? pts[i][2] : z; }
    p.geometry.setDrawRange(0, n); p.geometry.attributes.position.needsUpdate = true; this.dirty = true;
  }
  line(name, color = 0xfacc15) {
    const geo = new THREE.BufferGeometry(); geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 1200), 3)); geo.setDrawRange(0, 0);
    const l = new THREE.Line(geo, new THREE.LineBasicMaterial({ color })); l.frustumCulled = false; this.scene.add(l); this.named[name] = l; return l;
  }
  setLine(name, pts) {
    const l = this.named[name]; if (!l) return; const a = l.geometry.attributes.position.array, n = Math.min(pts.length, 1200);
    for (let i = 0; i < n; i++) { a[3 * i] = pts[i][0]; a[3 * i + 1] = pts[i][1]; a[3 * i + 2] = pts[i][2] || 0; }
    l.geometry.setDrawRange(0, n); l.geometry.attributes.position.needsUpdate = true; this.dirty = true;
  }
  sphere(name, color = 0xfacc15, r = 0.012) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), new THREE.MeshBasicMaterial({ color })); this.scene.add(m); this.named[name] = m; return m;
  }
  place(name, x, y, z) { const m = this.named[name]; if (m) { m.position.set(x, y, z); this.dirty = true; } }
  /* camera frustum as lines in the camera optical frame; K = 3x3, size = [w,h] */
  frustum(name, color = 0x38bdf8) {
    const g = new THREE.Group(); g.matrixAutoUpdate = false; this.scene.add(g); this.named[name] = g; g.userData.color = color; return g;
  }
  setFrustum(name, K, size, M, depth = 0.6) {
    const g = this.named[name]; if (!g) return; while (g.children.length) g.remove(g.children[0]);
    const [w, h] = size, c = [[0, 0], [w, 0], [w, h], [0, h]].map(([u, v]) => new THREE.Vector3((u - K[0][2]) / K[0][0] * depth, (v - K[1][2]) / K[1][1] * depth, depth));
    const o = new THREE.Vector3(0, 0, 0), pts = [];
    c.forEach((p, i) => { pts.push(o, p, p, c[(i + 1) % 4]); });
    g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: g.userData.color })));
    g.matrix.copy(M); g.matrixWorldNeedsUpdate = true; this.dirty = true;
  }
  clearDyn() { while (this.dyn.children.length) this.dyn.remove(this.dyn.children[0]); this.dirty = true; }
  dyn_dot(p, color = 0xfacc15, r = 0.008) { const m = new THREE.Mesh(new THREE.SphereGeometry(r, 12, 8), new THREE.MeshBasicMaterial({ color })); m.position.set(...p); this.dyn.add(m); return m; }
  dyn_line(pts, color = 0xfacc15) { this.dyn.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map(p => new THREE.Vector3(...p))), new THREE.LineBasicMaterial({ color }))); this.dirty = true; }
  person(name, color = 0xf472b6) {
    const g = new THREE.Group(); const body = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 1.2, 14), new THREE.MeshPhongMaterial({ color, transparent: true, opacity: 0.6 }));
    body.rotation.x = Math.PI / 2; body.position.z = 0.6; g.add(body);
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 10), new THREE.MeshPhongMaterial({ color })); head.position.z = 1.35; g.add(head);
    g.visible = false; this.scene.add(g); this.named[name] = g; return g;
  }
}

