/* Sections 1-7: live robot, frames, rotations, mecanum + odometry, forward kinematics, inverse kinematics, camera. */
const SECTIONS = [];
function section(id, title, html, init) { SECTIONS.push({ id, title, html, init }); }
const JOINT_RU = ['поворот основания', 'плечо', 'локоть', 'наклон кисти', 'вращение кисти', 'захват'];
const SERVO_RANGE = [[0, 180], [0, 180], [0, 180], [0, 180], [0, 270], [30, 180]];
let LAST_SCAN_T = 0;
const throttle = (fn, ms) => { let t = 0; return (...a) => { const n = performance.now(); if (n - t >= ms) { t = n; fn(...a); } }; };
const people3d = (sc, names, L) => names.forEach((n, i) => {
  const p = (L.people || []).filter(x => x.xyz && x.misses === 0)[i], g = sc.named[n];
  if (p) { g.visible = true; g.position.set(p.xyz[0], p.xyz[1], 0); } else g.visible = false; sc.dirty = true;
});

/* ================================================================== 1. live robot */
section('live', 'Живой робот', `
<h2>Живой робот</h2>
<p class="lead">Это настоящий M3 Pro, прямо сейчас: детали взяты из URDF производителя (STL), углы суставов приходят от серво, синие точки — слитый скан двух лидаров, голубая пирамида — поле зрения камеры на запястье, розовые фигуры — люди, найденные детектором (включите превью в разделе «Следование»). Тяните мышью, чтобы вращать, колесо — масштаб.</p>
<div class="grid2">
 <div><canvas id="c-live" class="viewer"></canvas><div class="muted" id="live-note"></div></div>
 <div><h3>Суставы руки</h3><div id="live-joints"></div><h3>База и датчики</h3><div id="live-base" class="kvs"></div></div>
</div>
<div class="grid2">
 <div><h3>Углы серво, градусы (60 с)</h3><canvas class="chart" id="ch-arm"></canvas><div id="lg-arm"></div></div>
 <div><h3>Команда скорости базы (60 с)</h3><canvas class="chart" id="ch-vel"></canvas><div id="lg-vel"></div></div>
</div>
<div class="note">Если рука стоит неподвижно, а линии на графиках ровные — это нормально. Начните двигать робота из консоли, и графики оживут. Рывки на графике — признак «ломаного» движения; профиль скорости кисти должен быть плавным (минимальный рывок).</div>`,
async () => {
  const sc = new Scene3D($('#c-live'), { cam: [0.75, -0.85, 0.6], target: [0.1, 0, 0.18], floor: 1.2 });
  await sc.addRobot({ opacity: 0.92 }); sc.pointCloud('scan', 0x3b82f6, 0.014);
  sc.frustum('fr'); sc.axes('tcp', 0.06, 'кончик'); ['p0', 'p1', 'p2'].forEach(n => sc.person(n));
  const colors = COLORS(), seen = { camera: 0 };
  const draw = throttle(L => {
    const h = Live.hist, now = L.t, t = ago(h.t, now);
    const series = [0, 1, 2, 3, 4].map(i => ({ name: JOINT_RU[i], color: colors[i], x: t, y: h.armMeas.map(a => a[i]) }));
    lineChart($('#ch-arm'), series, { x0: -60, x1: 0, xLabel: 'секунды назад' }); $('#lg-arm').innerHTML = legend(series);
    const vs = [['vx', 0], ['vy', 1], ['wz', 2]].map(([n, i], k) => ({ name: n, color: colors[k], x: t, y: h.cmd.map(c => c[i]) }));
    lineChart($('#ch-vel'), vs, { x0: -60, x1: 0, y0: -0.5, y1: 0.5, xLabel: 'секунды назад' }); $('#lg-vel').innerHTML = legend(vs);
  }, 500);
  Live.on(L => {
    sc.setServo(L.fk.servo);
    sc.setPoints('scan', L.scan);
    if (L.camera) sc.setFrustum('fr', L.camera.K, L.camera.size, m4of(L.fk.camera), 0.5);
    sc.setAxes('tcp', m4of(L.fk.tcp)); people3d(sc, ['p0', 'p1', 'p2'], L);
    const meas = L.arm.measured;
    $('#live-joints').innerHTML = `<table class="num"><tr><th>сустав</th><th>команда</th><th>измерено</th><th>диапазон</th></tr>${[0, 1, 2, 3, 4, 5].map(i => {
      const m = meas ? meas[i] : null, c = L.arm.cmd[i], r = SERVO_RANGE[i], p = ((m ?? c) - r[0]) / (r[1] - r[0]) * 100;
      return `<tr><td>${JOINT_RU[i]}</td><td>${f(c, 0)}°</td><td>${m === null ? '–' : f(m, 0) + '°'}</td><td><div class="bar"><i style="left:0;width:${clamp(p, 0, 100)}%"></i></div></td></tr>`;
    }).join('')}</table><div class="muted">Кисть (TCP): x = <b>${f(L.arm.tcp[0])}</b>, y = <b>${f(L.arm.tcp[1])}</b>, z = <b>${f(L.arm.tcp[2])}</b> м (по прямой кинематике, раздел 5).</div>`;
    $('#live-base').innerHTML = `<span>поза odom</span><span>x ${f(L.pose[0])} м · y ${f(L.pose[1])} м · ψ ${f(L.pose[2] / D2R, 1)}°</span><span>команда</span><span>${L.cmd.map(v => f(v, 2)).join(' · ')}</span>
      <span>скан лидара</span><span>${L.scan.length} точек (оба лидара, 6 Гц)</span><span>камера</span><span>${L.camera ? `${L.camera.size.join('×')}, кадр ${f(L.camera.age, 2)} с назад` : 'нет кадра'}</span>
      <span>напряжение</span><span>${f(L.battery, 2)} В</span><span>барьер лидара</span><span>${L.guard ? L.guard.join(' ') : 'свободно'}</span>
      <span>режим</span><span>${L.mode}${L.dry_run ? ' (dry-run)' : ''}${L.active ? ' · идёт ' + L.active : ''}</span>`;
    $('#live-note').textContent = L.preview.people || L.preview.hands ? 'превью восприятия включено: люди и руки показываются из реального детектора' : 'люди не показаны: включите превью восприятия в разделе «Следование»';
    draw(L);
  });
});

/* ================================================================== 2. coordinate frames */
section('frames', 'Системы координат', `
<h2>Системы координат и матрицы $4\\times4$</h2>
<p class="lead">Каждая точка в роботе записывается относительно какой-то системы: <code>odom</code> (где робот включился), <code>base_link</code> (центр корпуса: x вперёд, y влево, z вверх), камера (x вправо, y вниз, z вперёд). Переход между ними — однородная матрица $T$: левый блок $3\\times3$ — повёрнутые оси, правый столбец — положение начала. Все матрицы справа — живые.</p>
<div class="grid2">
 <div><canvas id="c-frames" class="viewer"></canvas><div class="muted">Красная ось — x, зелёная — y, синяя — z. Робот ездит по сетке odom, камера и кисть двигаются вместе с рукой.</div></div>
 <div id="fr-mats"></div>
</div>
<div class="formula">$$ {}^{odom}T_{cam} \\;=\\; {}^{odom}T_{base}\\cdot {}^{base}T_{cam},\\qquad {}^{map}T_{base} = {}^{map}T_{odom}\\cdot{}^{odom}T_{base} $$</div>
<h3>Точка перед камерой → в odom</h3>
<div class="ctl">${ctl('fr-z', 'расстояние от камеры, м', 0.3, 3, 0.05, 1)}</div>
<div id="fr-point"></div>
<p>Именно так следование за человеком превращает «пиксель + глубина» в координаты человека относительно робота, а карта — в координаты на плане. Цепочка $map \\to odom \\to base \\to cam$ собирается перемножением; <code>map→odom</code> публикует SLAM и подтягивает дрейф одометрии (раздел «Карта и навигация»).</p>`,
async () => {
  const sc = new Scene3D($('#c-frames'), { cam: [0.75, -0.9, 0.6], target: [0.15, 0, 0.15], floor: 2 });
  await sc.addRobot({ opacity: 0.55 });
  sc.axes('odom', 0.25, 'odom'); sc.axes('base', 0.15, 'base_link', sc.world); sc.axes('cam', 0.1, 'camera', sc.world); sc.axes('tcp', 0.06, 'tcp', sc.world);
  const l0 = new THREE.Matrix4().makeTranslation(-0.11617, 0.09156, 0.1253), l1 = new THREE.Matrix4().makeTranslation(0.10766, -0.09078, 0.1253);
  sc.axes('l0', 0.05, 'лидар 0', sc.world); sc.axes('l1', 0.05, 'лидар 1', sc.world); sc.setAxes('l0', l0); sc.setAxes('l1', l1);
  sc.dyn_dot([0, 0, 0]); const P = sc.sphere('pt', 0xfacc15, 0.02); const upd = L => {
    L = L || Live.last; if (!L) return;
    const [x, y, yaw] = L.pose; sc.setBase(x, y, yaw); sc.servo = L.fk.servo; sc.update();
    const Tc = m4of(L.fk.camera); sc.setAxes('cam', Tc); sc.setAxes('tcp', m4of(L.fk.tcp));
    const c = Math.cos(yaw), s = Math.sin(yaw), To = [[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]];
    const Mo = m4of(To), Mc = m4of(L.fk.camera), Moc = Mo.clone().multiply(Mc), z = +$('#fr-z').value;
    const pc = new THREE.Vector3(0, 0, z), pb = pc.clone().applyMatrix4(Mc), po = pb.clone().applyMatrix4(Mo);
    sc.place('pt', po.x, po.y, po.z); sc.controls.target.set(x + 0.15, y, 0.2);
    $('#fr-mats').innerHTML = `<div class="rowmat"><span class="l">${'{}'}<sup>odom</sup>T<sub>base</sub> (из одометрии, x=${f(x, 2)}, y=${f(y, 2)}, ψ=${f(yaw / D2R, 0)}°)</span>${mat(To, 3, (i, j) => j === 3 && i < 2)}</div>
      <div class="rowmat"><span class="l"><sup>base</sup>T<sub>cam</sub> (прямая кинематика руки, оптическая система камеры)</span>${mat(m4rows(Mc), 3, (i, j) => j === 3 && i < 3)}</div>
      <div class="rowmat"><span class="l"><sup>odom</sup>T<sub>cam</sub> = произведение</span>${mat(m4rows(Moc), 3, (i, j) => j === 3 && i < 3)}</div>`;
    $('#fr-point').innerHTML = `<div class="rowmat"><span class="l">точка в камере</span>${vec([pc.x, pc.y, pc.z])}<span class="l">→ в base</span>${vec([pb.x, pb.y, pb.z])}<span class="l">→ в odom</span>${vec([po.x, po.y, po.z])}</div>
      <div class="muted">z в камере — глубина вдоль оси взгляда; в base эта же точка оказывается впереди робота на ${f(pb.x, 2)} м и на высоте ${f(pb.z, 2)} м.</div>`;
  };
  Live.on(throttle(upd, 250)); bindCtl(['fr-z'], () => upd(), { 'fr-z': ' м' }); upd(); math($('#sec-frames'));
});

/* ================================================================== 3. rotations and lidar cloud */
section('rot', 'Повороты и точки лидара', `
<h2>Поворот, сдвиг и облако лидара</h2>
<p class="lead">Лидар видит стену «спереди-слева» — относительно корпуса. Чтобы нарисовать карту, точки надо перенести в неподвижную систему: повернуть на угол робота $\\psi$ и сдвинуть на его положение. Здесь слитый скан робота (синие точки) переносится в систему odom вашими ползунками.</p>
<div class="grid2"><div><canvas id="c-rot" class="sq"></canvas></div><div>
 <div class="ctl">${ctl('rt-psi', 'угол ψ', -180, 180, 1, 0, '°')}${ctl('rt-x', 'сдвиг x', -3, 3, 0.05, 0, ' м')}${ctl('rt-y', 'сдвиг y', -3, 3, 0.05, 0, ' м')}</div>
 <div class="row"><button class="b" id="rt-live">взять живую позу робота</button><label class="muted"><input type="checkbox" id="rt-follow"> следить за позой</label></div>
 <div class="formula">$$ \\begin{bmatrix}x'\\\\y'\\end{bmatrix}=\\underbrace{\\begin{bmatrix}\\cos\\psi&-\\sin\\psi\\\\ \\sin\\psi&\\cos\\psi\\end{bmatrix}}_{R_z(\\psi)}\\begin{bmatrix}x\\\\y\\end{bmatrix}+\\begin{bmatrix}t_x\\\\t_y\\end{bmatrix} $$</div>
 <div id="rt-mat"></div></div></div>
<div class="note ok">Ближайшая точка скана — это то, что останавливает робота у стены: лидарный барьер проверяет, не окажется ли такая точка внутри прямоугольника корпуса при движении.</div>`,
async () => {
  const cv = $('#c-rot');
  const draw = () => {
    const L = Live.last, d = window.devicePixelRatio || 1, r = cv.getBoundingClientRect(); cv.width = r.width * d; cv.height = r.height * d;
    const g = cv.getContext('2d'), W = cv.width, H = cv.height; g.clearRect(0, 0, W, H);
    const psi = +$('#rt-psi').value * D2R, tx = +$('#rt-x').value, ty = +$('#rt-y').value, c = Math.cos(psi), s = Math.sin(psi);
    const S = Math.min(W, H) / 8, X = x => W / 2 + x * S, Y = y => H / 2 - y * S;
    g.strokeStyle = 'rgba(150,170,200,.25)'; g.lineWidth = d; for (let i = -4; i <= 4; i++) { g.beginPath(); g.moveTo(X(i), 0); g.lineTo(X(i), H); g.moveTo(0, Y(i)); g.lineTo(W, Y(i)); g.stroke(); }
    const axis = (ox, oy, th, col, lab) => { g.strokeStyle = col; g.lineWidth = 2 * d; [[1, 0, '#ef4444'], [0, 1, '#22c55e']].forEach(([a, b, cc]) => { const ex = ox + 0.6 * (a * Math.cos(th) - b * Math.sin(th)), ey = oy + 0.6 * (a * Math.sin(th) + b * Math.cos(th)); g.strokeStyle = cc; g.beginPath(); g.moveTo(X(ox), Y(oy)); g.lineTo(X(ex), Y(ey)); g.stroke(); }); g.fillStyle = '#e8ecf2'; g.font = `${12 * d}px sans-serif`; g.fillText(lab, X(ox) + 6 * d, Y(oy) - 6 * d); };
    axis(0, 0, 0, '#888', 'odom'); axis(tx, ty, psi, '#888', 'base_link');
    const pts = L ? L.scan : []; let best = null;
    g.fillStyle = 'rgba(59,130,246,.35)'; pts.forEach(([x, y]) => { g.fillRect(X(x) - 1.5 * d, Y(y) - 1.5 * d, 3 * d, 3 * d); });
    g.fillStyle = '#facc15'; pts.forEach(([x, y]) => { const xo = c * x - s * y + tx, yo = s * x + c * y + ty; g.fillRect(X(xo) - 2 * d, Y(yo) - 2 * d, 4 * d, 4 * d); if (!best || Math.hypot(x, y) < Math.hypot(best[0], best[1])) best = [x, y]; });
    g.fillStyle = '#e8ecf2'; g.font = `${11 * d}px sans-serif`; g.fillText('голубые — точки в base_link, жёлтые — те же точки в odom', 8 * d, H - 8 * d);
    if (best) { const xo = c * best[0] - s * best[1] + tx, yo = s * best[0] + c * best[1] + ty; g.strokeStyle = '#f472b6'; g.lineWidth = 2 * d; g.beginPath(); g.arc(X(xo), Y(yo), 8 * d, 0, 7); g.stroke();
      $('#rt-mat').innerHTML = `<div class="rowmat"><span class="l">$R_z$</span>${mat([[c, -s], [s, c]], 3)}<span class="l">· ближайшая точка</span>${vec(best)}<span class="l">+ сдвиг</span>${vec([tx, ty])}<span class="l">=</span>${vec([xo, yo])}</div><div class="muted">Расстояние до ближайшей точки: ${f(Math.hypot(best[0], best[1]), 2)} м (одинаково в обеих системах: поворот длины не меняет).</div>`; }
  };
  const live = () => { const L = Live.last; if (!L) return; $('#rt-psi').value = Math.round(L.pose[2] / D2R); $('#rt-x').value = L.pose[0].toFixed(2); $('#rt-y').value = L.pose[1].toFixed(2); ['rt-psi', 'rt-x', 'rt-y'].forEach(i => $('#' + i).dispatchEvent(new Event('input'))); };
  bindCtl(['rt-psi', 'rt-x', 'rt-y'], draw, { 'rt-psi': '°', 'rt-x': ' м', 'rt-y': ' м' }); $('#rt-live').onclick = live;
  Live.on(throttle(() => { if ($('#rt-follow').checked) live(); else draw(); }, 300)); draw(); math($('#sec-rot'));
});

/* ================================================================== 4. mecanum and odometry */
section('mecanum', 'Меканум и одометрия', `
<h2>Меканум-колёса и одометрия</h2>
<p class="lead">Четыре колеса с роликами под 45°: комбинируя их вращения, база едет вперёд, боком и поворачивается. Геометрия ниже взята из URDF этого робота (колёса $r=40$ мм, полубаза $l_x$, полуширина $l_y$). Ползунки задают скорость базы, колёса и матрицы пересчитываются; галочка «живая» берёт реальную команду.</p>
<div class="grid2"><div><canvas id="c-mec" class="viewer"></canvas>
 <div class="ctl">${ctl('mc-vx', 'vx вперёд', -0.5, 0.5, 0.01, 0.2, ' м/с')}${ctl('mc-vy', 'vy влево', -0.4, 0.4, 0.01, 0, ' м/с')}${ctl('mc-wz', 'wz поворот', -1.5, 1.5, 0.05, 0, ' рад/с')}</div>
 <label class="muted"><input type="checkbox" id="mc-live"> показывать реальное движение (twist колёс из прошивки)</label></div>
 <div><div id="mc-mat"></div><h3>Скорости колёс, рад/с</h3><canvas class="chart" id="ch-wheels" style="height:150px"></canvas></div></div>
<div class="formula">$$ \\omega=J^{+}\\!\\begin{bmatrix}v_x\\\\v_y\\\\w_z\\end{bmatrix},\\quad J^{+}=\\frac1r\\begin{bmatrix}1&-1&-K\\\\1&1&K\\\\1&1&-K\\\\1&-1&K\\end{bmatrix},\\ K=l_x+l_y;\\qquad \\begin{bmatrix}v_x\\\\v_y\\\\w_z\\end{bmatrix}=\\frac r4\\begin{bmatrix}1&1&1&1\\\\-1&1&1&-1\\\\-\\tfrac1K&\\tfrac1K&-\\tfrac1K&\\tfrac1K\\end{bmatrix}\\omega $$</div>
<p>Порядок колёс: 1 — переднее левое, 2 — переднее правое, 3 — заднее левое, 4 — заднее правое (схема для стандартной установки роликов; прошивка делает ту же математику внутри, наш код отправляет не колёса, а $(v_x, v_y, w_z)$ в <code>/cmd_vel</code>).</p>
<h3>Одометрия: колёса, гироскоп и их смесь</h3>
<p>Поза интегрируется из скоростей: $x \\mathrel{+}= (v_x\\cos\\psi - v_y\\sin\\psi)\\,dt$, $y \\mathrel{+}= (v_x\\sin\\psi + v_y\\cos\\psi)\\,dt$, $\\psi \\mathrel{+}= w_z\\,dt$. Ролики скользят особенно при вращении, поэтому угол берётся из гироскопа (с вычитанием смещения нуля): $\\psi_{fused}=\\alpha(\\psi_{fused}+\\omega_{gyro}dt)+(1-\\alpha)\\psi_{wheels}$, здесь $\\alpha=0.97$.</p>
<div class="grid2"><div><h3>Угол ψ: колёса, гироскоп, итог (градусы от начала страницы)</h3><canvas class="chart tall" id="ch-yaw"></canvas><div id="lg-yaw"></div></div>
 <div><h3>Траектория в плоскости (м)</h3><canvas id="c-trail" class="sq"></canvas><div class="legend"><span><i style="background:#ef4444"></i>только колёса</span><span><i style="background:#22c55e"></i>итог (колёса+гироскоп)</span></div></div></div>
<div id="mc-odo" class="kvs"></div>
<div class="note">Если вы сдвинете или повернёте робота — графики разойдутся: красная (колёса) уходит первой, зелёная остаётся близко к истине. Лидарное сопоставление (ICP) здесь выключено, потому что на этом роботе живая коррекция давала скачки угла; вместо него дрейф исправляет SLAM в карте.</div>`,
async () => {
  const sc = new Scene3D($('#c-mec'), { cam: [0.55, -0.6, 0.45], target: [0, 0, 0.1], floor: 0.8 });
  await sc.addRobot({ opacity: 1 }); sc.setServo([90, 130, 5, 25, 90, 30]);
  let wheelPos = [[0.0776, 0.09775], [0.0776, -0.09775], [-0.0814, 0.09775], [-0.0814, -0.09775]];
  const urdf = sc.urdf; const wj = n => urdf.joints.find(j => j.name === n);
  wheelPos = ['lwheel1_Joint', 'rwheel1_Joint', 'lwheel2_Joint', 'rwheel2_Joint'].map(n => wj(n).xyz);
  const r = 0.04, lx = (Math.abs(wheelPos[0][0]) + Math.abs(wheelPos[2][0])) / 2, ly = Math.abs(wheelPos[0][1]), K = lx + ly;
  const Jp = [[1, -1, -K], [1, 1, K], [1, 1, -K], [1, -1, K]].map(row => row.map(v => v / r));
  const J = [[1, 1, 1, 1], [-1, 1, 1, -1], [-1 / K, 1 / K, -1 / K, 1 / K]].map(row => row.map(v => v * r / 4));
  const th = [0, 0, 0, 0]; let last = performance.now(), w = [0, 0, 0, 0], wh = [];
  const twist = () => { const L = Live.last; if ($('#mc-live').checked && L && L.raw) return L.raw.twist; return [+$('#mc-vx').value, +$('#mc-vy').value, +$('#mc-wz').value]; };
  const loop = () => {
    const now = performance.now(), dt = Math.min(0.1, (now - last) / 1000); last = now;
    const v = twist(); w = Jp.map(row => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]); w.forEach((x, i) => th[i] += x * dt); sc.setWheels(th);
    requestAnimationFrame(loop);
  }; loop();
  const show = throttle(() => {
    const v = twist(), ww = Jp.map(row => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]); wh.push([performance.now() / 1000, ...ww]); wh = wh.filter(x => x[0] - wh[wh.length - 1][0] > -20);
    $('#mc-mat').innerHTML = `<div class="rowmat"><span class="l">геометрия из URDF: $r$ = ${f(r * 1000, 0)} мм, $l_x$ = ${f(lx * 1000, 1)} мм, $l_y$ = ${f(ly * 1000, 1)} мм, $K$ = ${f(K * 1000, 1)} мм</span></div>
      <div class="rowmat"><span class="l">$J^{+}$ (4×3)</span>${mat(Jp, 1)}<span class="l">·</span>${vec(v, 2)}<span class="l">=</span>${vec(ww, 1)}</div>
      <div class="rowmat"><span class="l">$J$ (3×4)</span>${mat(J, 4)}</div><div class="muted">Проверка: $J\\cdot\\omega$ = (${(J.map(row => row.reduce((a, c, i) => a + c * ww[i], 0))).map(x => f(x, 3)).join(', ')}) — те же $(v_x, v_y, w_z)$, значит матрицы согласованы.</div>`;
    math($('#mc-mat'));
    const cols = COLORS(), t = wh.map(x => x[0] - wh[wh.length - 1][0]);
    lineChart($('#ch-wheels'), [0, 1, 2, 3].map(i => ({ name: 'ω' + (i + 1), color: cols[i], x: t, y: wh.map(x => x[i + 1]) })), { x0: -20, x1: 0, xLabel: 'секунды назад' });
  }, 250);
  const trailW = [], trailF = []; let base = null;
  Live.on(L => {
    show(); if (!L.raw) return;
    if (!base) base = { yaw: L.raw.wheel[2], gyro: L.raw.gyro_yaw, fyaw: L.pose[2], wx: L.raw.wheel[0], wy: L.raw.wheel[1], fx: L.pose[0], fy: L.pose[1] };
    trailW.push([L.raw.wheel[0] - base.wx, L.raw.wheel[1] - base.wy]); trailF.push([L.pose[0] - base.fx, L.pose[1] - base.fy]); if (trailW.length > 1500) { trailW.shift(); trailF.shift(); }
  });
  Live.on(throttle(L => {
    if (!base || !L.raw) return; const h = Live.hist, t = ago(h.t, L.t), cols = COLORS();
    const dd = v => v / D2R, ser = [
      { name: 'колёса', color: '#ef4444', x: t, y: h.raw.map(r => dd(r.wheel[2] - base.yaw)) },
      { name: 'гироскоп (∫ω−bias)', color: cols[0], x: t, y: h.raw.map(r => dd(r.gyro_yaw - base.gyro)) },
      { name: 'итог', color: '#22c55e', x: t, y: h.pose.map(p => dd(p[2] - base.fyaw)) }];
    lineChart($('#ch-yaw'), ser, { x0: -60, x1: 0, xLabel: 'секунды назад' }); $('#lg-yaw').innerHTML = legend(ser);
    const cv = $('#c-trail'), d = window.devicePixelRatio || 1, rc = cv.getBoundingClientRect(); cv.width = rc.width * d; cv.height = rc.height * d; const g = cv.getContext('2d');
    const all = trailW.concat(trailF), ext = Math.max(0.3, ...all.map(p => Math.max(Math.abs(p[0]), Math.abs(p[1])))), S = Math.min(cv.width, cv.height) / 2.4 / ext;
    const X = y => cv.width / 2 - y * S, Y = x => cv.height / 2 - x * S; g.strokeStyle = 'rgba(150,170,200,.25)'; g.beginPath(); g.moveTo(0, Y(0)); g.lineTo(cv.width, Y(0)); g.moveTo(X(0), 0); g.lineTo(X(0), cv.height); g.stroke();
    [[trailW, '#ef4444'], [trailF, '#22c55e']].forEach(([tr, c]) => { g.strokeStyle = c; g.lineWidth = 2 * d; g.beginPath(); tr.forEach((p, i) => i ? g.lineTo(X(p[1]), Y(p[0])) : g.moveTo(X(p[1]), Y(p[0]))); g.stroke(); });
    g.fillStyle = '#e8ecf2'; g.font = `${11 * d}px sans-serif`; g.fillText('вверх — вперёд (x), влево — y; масштаб ±' + f(ext, 1) + ' м', 8 * d, cv.height - 8 * d);
    $('#mc-odo').innerHTML = `<span>смещение нуля гироскопа</span><span>${f(L.raw.bias * 1000, 2)} мрад/с (изучается, пока робот стоит)</span><span>ω гироскопа сейчас</span><span>${f(L.raw.gyro_wz, 3)} рад/с</span>
      <span>twist колёс (прошивка)</span><span>${L.raw.twist.map(v => f(v, 3)).join(' · ')}</span><span>ICP использован / отвергнут</span><span>${L.odom_stats.icp_used} / ${L.odom_stats.icp_rejected} (коррекция выключена в конфиге)</span>`;
  }, 400));
  bindCtl(['mc-vx', 'mc-vy', 'mc-wz'], show, { 'mc-vx': ' м/с', 'mc-vy': ' м/с', 'mc-wz': ' рад/с' }); show(); math($('#sec-mecanum'));
});

/* ================================================================== 5. forward kinematics */
section('fk', 'Прямая кинематика руки', `
<h2>Прямая кинематика: из углов серво в положение кисти</h2>
<p class="lead">Рука — цепочка из пяти вращений (URDF: <code>arm1…arm5</code>). Для каждого сустава матрица шага — «фиксированный сдвиг и поворот из чертежа» × «поворот сустава на угол $q$». Перемножив шаги от основания, получаем положение любого звена, камеры и кончика захвата. Ползунки — серво-углы (как в прошивке: $q_1=90°-s_1$, $q_i=s_i-90°$); модель на сцене и числа пересчитываются в браузере по тому же URDF, а кнопка сверяет их с кодом робота.</p>
<div class="grid2"><div><canvas id="c-fk" class="viewer"></canvas>
 <div id="fk-sliders" class="ctl"></div><div class="row"><label class="muted"><input type="checkbox" id="fk-live"> следовать за настоящей рукой</label><button class="b" id="fk-check">Сверить с кодом робота</button></div><div id="fk-cmp" class="muted"></div></div>
 <div><div class="ctl">${ctl('fk-step', 'шаг цепочки', 1, 5, 1, 1)}</div><div id="fk-steps"></div><div id="fk-tcp"></div></div></div>
<div class="formula">$$ {}^{0}T_{k}={}^{0}T_{k-1}\\cdot\\underbrace{\\mathrm{Trans}(\\mathbf p_k)\\,R_{rpy}(\\mathbf r_k)}_{\\text{из чертежа}}\\cdot\\underbrace{R_{\\mathbf a_k}(q_k)}_{\\text{сустав}},\\qquad {}^{0}T_{tcp}={}^{0}T_5\\cdot\\mathrm{Trans}(0,0,0.085) $$</div>`,
async () => {
  const sc = new Scene3D($('#c-fk'), { cam: [0.55, -0.65, 0.5], target: [0.06, 0, 0.25], floor: 0.6 });
  await sc.addRobot({ opacity: 0.85 }); sc.axes('step', 0.07, 'шаг'); sc.axes('tcp', 0.05, 'tcp'); sc.dirty = true;
  const init = [90, 118, 52, 40, 90, 30], names = ['arm1_Joint', 'arm2_Joint', 'arm3_Joint', 'arm4_Joiint', 'arm5_Joint'];
  $('#fk-sliders').innerHTML = init.map((v, i) => ctl('fk-s' + i, JOINT_RU[i], SERVO_RANGE[i][0], SERVO_RANGE[i][1], 1, v, '°')).join('');
  const val = () => init.map((_, i) => +$('#fk-s' + i).value);
  const steps = s => {
    const q = servoToQ(s); let C = new THREE.Matrix4(); const out = [];
    names.forEach((n, i) => {
      const j = sc.urdf.joints.find(x => x.name === n), O = originMatrix(j.xyz, j.rpy), R = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(...j.axis).normalize(), q[i]);
      // arm1 hangs off base_link: its fixed part already includes the base height
      C = C.clone().multiply(O).multiply(R); out.push({ n, j, O, R, C: C.clone(), q: q[i] });
    });
    return out;
  };
  let timer = 0;
  const update = () => {
    const s = val(); sc.setServo(s); const st = steps(s), k = +$('#fk-step').value - 1, S = st[k];
    sc.setAxes('step', S.C); const tcp = st[4].C.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0, 0.085)); sc.setAxes('tcp', tcp);
    $('#fk-steps').innerHTML = `<p><b>Шаг ${k + 1}: ${S.j.parent} → ${S.j.child}.</b> Сустав «${JOINT_RU[k]}», угол $q=${f(S.q / D2R, 1)}°$ (серво ${s[k]}°), ось $(${S.j.axis.map(v => f(v, 3)).join(', ')})$.</p>
      <div class="muted">Сдвиг из чертежа: (${S.j.xyz.map(v => f(v, 4)).join(', ')}) м; поворот из чертежа rpy: (${S.j.rpy.map(v => f(v, 4)).join(', ')}) рад.</div>
      <div class="rowmat"><span class="l">фиксированная часть</span>${mat(m4rows(S.O), 3)}<span class="l">×</span><span class="l">поворот сустава</span>${mat(m4rows(S.R), 3)}</div>
      <div class="rowmat"><span class="l">итог шага (звено в системе основания)</span>${mat(m4rows(S.C), 4, (i, j) => j === 3 && i < 3)}</div>
      <div class="muted">Левый блок $3\\times3$ — повёрнутые оси звена, правый столбец (подсвечен) — положение его начала в метрах. Каждый шаг умножает прежний итог на новую матрицу.</div>`;
    const p = new THREE.Vector3().setFromMatrixPosition(tcp), az = new THREE.Vector3(0, 0, 1).transformDirection(st[4].C), pitch = Math.asin(clamp(-az.z, -1, 1)) / D2R;
    $('#fk-tcp').innerHTML = `<div class="note ok"><b>Кончик захвата:</b> x = ${f(p.x, 4)} м, y = ${f(p.y, 4)} м, z = ${f(p.z, 4)} м, наклон оси кисти вниз ${f(pitch, 1)}°.</div>`;
    math($('#fk-steps')); clearTimeout(timer); timer = setTimeout(async () => {
      try { const r = await api('/api/exp/fk', { servo: s }); const d = Math.hypot(r.tcp_xyz[0] - p.x, r.tcp_xyz[1] - p.y, r.tcp_xyz[2] - p.z);
        $('#fk-cmp').innerHTML = `код робота (<code>arm_model.fk</code>): TCP (${r.tcp_xyz.map(v => f(v, 4)).join(', ')}) · расхождение с браузерным расчётом <b>${f(d * 1000, 3)} мм</b> ${d < 1e-3 ? '✓' : '⚠'}`; } catch (e) { $('#fk-cmp').textContent = 'код робота недоступен'; }
    }, 400);
  };
  bindCtl([...init.map((_, i) => 'fk-s' + i), 'fk-step'], update, Object.fromEntries(init.map((_, i) => ['fk-s' + i, '°'])));
  $('#fk-check').onclick = update; update();
  Live.on(throttle(L => { if ($('#fk-live').checked) { L.fk.servo.forEach((v, i) => { $('#fk-s' + i).value = v; $('#fk-s' + i + '-o').textContent = Math.round(v) + '°'; }); update(); } }, 400));
  math($('#sec-fk'));
});

/* ================================================================== 6. inverse kinematics */
section('ik', 'Обратная кинематика', `
<h2>Обратная кинематика: из точки в углы серво</h2>
<p class="lead">Задача обратная: кисть должна прийти в точку $(x,y,z)$ с заданным наклоном. Здесь работает <b>настоящий решатель робота</b> (<code>arm_model.ik</code>, метод наименьших квадратов с несколькими стартами и проверкой на столкновения) — ползунок отправляет цель на робота, ответ рисуется на модели. Серое облако — где рука вообще достаёт при выбранном наклоне.</p>
<div class="grid2"><div><canvas id="c-ik" class="viewer"></canvas>
 <div class="ctl">${ctl('ik-x', 'x вперёд', 5, 45, 0.5, 22, ' см')}${ctl('ik-y', 'y влево', -20, 20, 0.5, 0, ' см')}${ctl('ik-z', 'z высота', 0, 60, 0.5, 4, ' см')}${ctl('ik-p', 'наклон вниз', 0, 90, 5, 70, '°')}</div></div>
 <div><div id="ik-res"></div><h3>Область достижимости (вид сбоку, y=0)</h3><canvas id="c-reach" class="wide"></canvas>
  <div class="ctl">${ctl('ik-hz', 'высота ладони человека', 20, 130, 1, 95, ' см')}${ctl('ik-hx', 'её расстояние x', 15, 70, 1, 40, ' см')}</div><div id="ik-hand"></div></div></div>
<div class="formula">$$ \\min_{q}\\;\\big\\|\\,\\mathrm{FK}(q)_{xyz}-\\mathbf t\\,\\big\\|^2+w_\\phi\\,(\\phi(q)-\\phi^*)^2,\\qquad \\Delta q=J^{\\top}(JJ^{\\top}+\\lambda^2 I)^{-1}\\mathbf e $$</div>
<h3>Шаг визуального сервоуправления (демпфированный метод наименьших квадратов)</h3>
<p>При захвате камера на запястье видит ошибку (кубик не там, где кисть), и рука получает поправку $\\Delta q$ через якобиан $J=\\partial\\,\\mathrm{TCP}/\\partial q$ (3×4, град). Ошибка $\\mathbf e$ = 2 см вперёд:</p>
<div class="ctl">${ctl('ik-lam', 'демпфирование λ', 0.001, 0.05, 0.001, 0.01)}</div><div id="ik-dls"></div>
<div class="note">Как избежать «странных» поз: решатель отбрасывает решения с ошибкой больше 1 см, выходом за пределы серво и столкновением с полом/корпусом — это видно по галочкам справа.</div>`,
async () => {
  const sc = new Scene3D($('#c-ik'), { cam: [0.55, -0.6, 0.45], target: [0.15, 0, 0.12], floor: 0.6 });
  await sc.addRobot({ opacity: 0.8 }); sc.sphere('tgt', 0xef4444, 0.012); sc.axes('tcp', 0.04, '');
  let busy = false, again = false, last = null, reach = {};
  const tgt = () => [+$('#ik-x').value / 100, +$('#ik-y').value / 100, +$('#ik-z').value / 100];
  const reachFor = async p => { p = Math.round(p / 10) * 10; if (reach[p] && reach[p].ready) return reach[p]; reach[p] = await api('/api/exp/reach?pitch=' + p); return reach[p]; };
  const drawReach = async () => {
    const cv = $('#c-reach'), d = window.devicePixelRatio || 1, rc = cv.getBoundingClientRect(); cv.width = rc.width * d; cv.height = rc.height * d; const g = cv.getContext('2d'), W = cv.width, H = cv.height;
    const S = Math.min(W / 0.62, H / 0.8), X = x => 30 * d + x * S, Y = z => H - 24 * d - z * S; g.clearRect(0, 0, W, H);
    g.strokeStyle = 'rgba(150,170,200,.3)'; g.fillStyle = '#9aa5b4'; g.font = `${10 * d}px sans-serif`; for (let z = 0; z <= 1.2; z += 0.2) { g.beginPath(); g.moveTo(X(0), Y(z)); g.lineTo(W, Y(z)); g.stroke(); g.fillText(z.toFixed(1) + ' м', 2 * d, Y(z) - 2 * d); }
    g.fillStyle = '#4b5563'; g.fillRect(X(-0.155), Y(0.14), 0.285 * S, 0.14 * S); g.fillStyle = '#9aa5b4'; g.fillText('корпус', X(-0.12), Y(0.05));
    const R = await reachFor(+$('#ik-p').value); g.fillStyle = 'rgba(120,150,200,.5)'; if (R.points) R.points.forEach(([x, z]) => g.fillRect(X(x) - 0.01 * S, Y(z) - 0.015 * S, 0.02 * S, 0.03 * S));
    if (!R.ready) { g.fillStyle = '#facc15'; g.fillText(`считаю область на роботе… ${R.done}/${R.total}`, X(0.05), Y(0.6)); setTimeout(drawReach, 1500); }
    const t = tgt(); g.fillStyle = '#ef4444'; g.beginPath(); g.arc(X(t[0]), Y(t[2]), 6 * d, 0, 7); g.fill();
    const hx = +$('#ik-hx').value / 100, hz = +$('#ik-hz').value / 100; g.fillStyle = '#f472b6'; g.beginPath(); g.arc(X(hx), Y(hz), 7 * d, 0, 7); g.fill(); g.fillText('ладонь', X(hx) + 9 * d, Y(hz));
    const near = (R.points || []).reduce((m, [x, z]) => Math.min(m, Math.hypot(x - hx, z - hz)), 9);
    $('#ik-hand').innerHTML = R.ready ? `<div class="note ${near < 0.04 ? 'ok' : 'bad'}">Ладонь на высоте ${f(hz * 100, 0)} см и расстоянии ${f(hx * 100, 0)} см: ближайшая достижимая точка в ${f(near * 100, 0)} см — ${near < 0.04 ? 'передача возможна' : 'вне досягаемости при этом наклоне: помощник сообщит «недостижимо»'}.</div>` : '';
  };
  const solve = async () => {
    if (busy) { again = true; return; } busy = true; const t = tgt(), p = +$('#ik-p').value;
    sc.place('tgt', t[0], t[1], t[2]);
    try {
      const r = await api('/api/exp/ik', { target: t, pitch: p, seed: Live.last ? Live.last.arm.cmd : undefined }); last = r;
      if (r.ok) { sc.setServo(r.servo); sc.setAxes('tcp', new THREE.Matrix4().makeTranslation(...r.tcp)); }
      $('#ik-res').innerHTML = r.ok ? `<div class="note ${r.collision_free && r.within_limits ? 'ok' : 'bad'}"><b>Решение найдено за ${f(r.ms, 0)} мс.</b> Остаток по позиции ${f(r.residual_m * 1000, 2)} мм, наклон ${f(r.pitch_down_deg, 1)}°; пределы серво ${r.within_limits ? '✓' : '✗'}, без столкновений ${r.collision_free ? '✓' : '✗'}.</div>
        <div class="rowmat"><span class="l">серво</span>${mat([r.servo.slice(0, 5)], 1)}<span class="l">TCP по FK</span>${mat([r.tcp], 4)}</div>
        <div class="rowmat"><span class="l">якобиан TCP по q (3×4, м/град)</span>${mat(r.jacobian, 5)}</div>` :
        `<div class="note bad"><b>Решения нет</b> (ошибка ${r.residual_m === null ? '—' : f(r.residual_m * 1000, 0) + ' мм'}): точка вне досягаемости или упирается в пол/корпус. Робот в этом случае не двигает руку, а сообщает «нет решения IK».</div>`;
      if (r.ok) dls(r.jacobian);
    } catch (e) { $('#ik-res').textContent = 'код робота недоступен: ' + e; }
    busy = false; if (again) { again = false; solve(); }
  };
  const dls = J => {
    const lam = +$('#ik-lam').value, e = [0.02, 0, 0];
    const JJt = J.map((a, i) => J.map((b, j) => a.reduce((s, v, k) => s + v * b[k], 0) + (i === j ? lam * lam : 0)));
    const det = M => M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    const dt = det(JJt); if (Math.abs(dt) < 1e-30) { $('#ik-dls').textContent = 'особая поза: якобиан вырожден'; return; }
    const inv = [0, 1, 2].map(i => [0, 1, 2].map(j => { const a = [(j + 1) % 3, (j + 2) % 3], b = [(i + 1) % 3, (i + 2) % 3]; return (JJt[a[0]][b[0]] * JJt[a[1]][b[1]] - JJt[a[0]][b[1]] * JJt[a[1]][b[0]]) / dt; }));
    const y = inv.map(row => row.reduce((s, v, k) => s + v * e[k], 0)), dq = [0, 1, 2, 3].map(k => J.reduce((s, row, i) => s + row[k] * y[i], 0));
    $('#ik-dls').innerHTML = `<div class="rowmat"><span class="l">$\\Delta q$ =</span>${vec(dq, 2)}<span class="l">градусов на суставы 1–4 для сдвига TCP на $(${e.map(v => f(v * 100, 0)).join(', ')})$ см; число обусловленности $\\det(JJ^\\top+\\lambda^2I)$ = ${dt.toExponential(2)}</span></div>`; math($('#ik-dls'));
  };
  bindCtl(['ik-x', 'ik-y', 'ik-z', 'ik-p'], () => { solve(); drawReach(); }, { 'ik-x': ' см', 'ik-y': ' см', 'ik-z': ' см', 'ik-p': '°' }); bindCtl(['ik-hz', 'ik-hx'], drawReach, { 'ik-hz': ' см', 'ik-hx': ' см' });
  bindCtl(['ik-lam'], () => last && last.ok && dls(last.jacobian)); Live.on(throttle(() => { if (!last && Live.last) solve(); }, 500));
  reachFor(0); reachFor(70); drawReach(); math($('#sec-ik')); addEventListener('resize', drawReach);
});

/* ================================================================== 7. camera to world */
section('cam', 'Камера → мир', `
<h2>Камера → мир: пиксель + глубина → точка в метрах</h2>
<p class="lead">Камера Orbbec DaBai DCW2 даёт цвет и глубину, совмещённые попиксельно. Кликните по изображению: робот возьмёт глубину из этого места и по матрице камеры $K$ вернёт точку сначала в системе камеры, затем (через положение руки) в системе робота. Именно так следование находит человека, а захват — кубик.</p>
<div class="grid2"><div><div class="imgwrap" id="cam-wrap"><img src="/video" alt="камера"><canvas id="cam-ov"></canvas></div><div class="muted">Клик по картинке = запрос к роботу (живая глубина).</div></div>
 <div><canvas id="c-cam" class="viewer"></canvas></div></div>
<div id="cam-out"></div>
<div class="formula">$$ K=\\begin{bmatrix}f_x&0&c_x\\\\0&f_y&c_y\\\\0&0&1\\end{bmatrix},\\qquad \\begin{bmatrix}X\\\\Y\\\\Z\\end{bmatrix}_{cam}=Z\\,K^{-1}\\begin{bmatrix}u\\\\v\\\\1\\end{bmatrix},\\qquad \\mathbf p_{base}={}^{base}T_{cam}\\begin{bmatrix}X\\\\Y\\\\Z\\\\1\\end{bmatrix} $$</div>
<div class="note">Меньше ~0.2 м глубина датчика недостоверна (мёртвая зона): поэтому кубик ближе 0.3 м не виден, а человек ближе 0.6 м оценивается неточно. Если глубины нет (чёрный/блестящий объект), расстояние до человека берётся грубо по высоте рамки и скорость уменьшается вдвое.</div>`,
async () => {
  const sc = new Scene3D($('#c-cam'), { cam: [0.9, -1.1, 0.8], target: [0.4, 0, 0.2], floor: 1.5 });
  await sc.addRobot({ opacity: 0.6 }); sc.frustum('fr'); sc.sphere('pt', 0xfacc15, 0.03); sc.line('ray', 0xfacc15); let pick = null;
  Live.on(throttle(L => { sc.setServo(L.fk.servo); if (L.camera) sc.setFrustum('fr', L.camera.K, L.camera.size, m4of(L.fk.camera), 1.2); }, 300));
  $('#cam-wrap').addEventListener('click', async e => {
    const rc = e.currentTarget.getBoundingClientRect(), L = Live.last; if (!L || !L.camera) return;
    const u = (e.clientX - rc.left) / rc.width * L.camera.size[0], v = (e.clientY - rc.top) / rc.height * L.camera.size[1];
    const r = await api(`/api/exp/pick?u=${u.toFixed(1)}&v=${v.toFixed(1)}`); pick = r;
    const ov = $('#cam-ov'), g = ov.getContext('2d'); ov.width = rc.width; ov.height = rc.height; g.strokeStyle = '#facc15'; g.lineWidth = 2; const px = e.clientX - rc.left, py = e.clientY - rc.top;
    g.beginPath(); g.moveTo(px - 14, py); g.lineTo(px + 14, py); g.moveTo(px, py - 14); g.lineTo(px, py + 14); g.stroke();
    if (!r.ok) { $('#cam-out').innerHTML = `<div class="note bad">${r.error}</div>`; return; }
    const o = r.origin_base, ray = r.ray_base; sc.setLine('ray', [o, o.map((v, i) => v + ray[i] * 2.5)]);
    if (r.xyz_base) sc.place('pt', ...r.xyz_base);
    $('#cam-out').innerHTML = `<div class="grid2"><div><div class="rowmat"><span class="l">$K$ (живая калибровка)</span>${mat(r.K, 2)}</div>
      <div class="kvs"><span>пиксель</span><span>u = ${f(u, 0)}, v = ${f(v, 0)}</span><span>глубина Z</span><span>${r.depth === null ? 'нет измерения' : f(r.depth, 3) + ' м'}</span>
      <span>в камере</span><span>${r.xyz_cam ? r.xyz_cam.map(x => f(x, 3)).join(', ') : '–'}</span><span>в base_link</span><span><b>${r.xyz_base ? r.xyz_base.map(x => f(x, 3)).join(', ') : '–'}</b></span>
      <span>пересечение луча с полом</span><span>${r.floor_hit ? r.floor_hit.map(x => f(x, 3)).join(', ') : 'луч не идёт вниз'}</span></div></div>
      <div><div class="rowmat"><span class="l"><sup>base</sup>T<sub>cam</sub></span>${mat(r.T_base_cam, 3, (i, j) => j === 3 && i < 3)}</div></div></div>`;
    math($('#cam-out'));
  });
  math($('#sec-cam'));
});
