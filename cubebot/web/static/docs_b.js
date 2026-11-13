/* Sections 8-13: follow, gestures, navigation, find&retrieve, assistant, trial results. */
let _ws = null;
function wsSend(o) {
  const go = () => _ws.send(JSON.stringify(o));
  if (!_ws || _ws.readyState > 1) { _ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws'); _ws.onopen = go; _ws.onmessage = () => { }; }
  else if (_ws.readyState === 0) _ws.addEventListener('open', go); else go();
}
const STATE_COLOR = { following: '#16a34a', holding: '#2563eb', locking: '#9aa5b4', lost: '#d97706', gave_up: '#dc2626', idle: '#9aa5b4' };
const STATE_RU = { following: 'едет', holding: 'держит дистанцию', locking: 'ищет цель', lost: 'потерял, ищет', gave_up: 'сдался', idle: 'нет цели' };

/* The follow control law: same numbers as FollowRunner.twist in follow.py (guard-free part) */
function followLaw(x, y, c, mono = false) {
  const r = Math.hypot(x, y); if (r < 0.15) return { r, e: 0, s: 0, g: 1, b: 0, cmd: [0, 0, 0] };
  const b = Math.atan2(y, x), e = r - c.standoff_m; let s;
  if (r < c.min_dist_m) s = -c.max_back; else if (Math.abs(e) <= c.deadband_m) s = 0; else s = clamp(c.kp_x * (e - Math.sign(e) * c.deadband_m), -c.max_back, c.max_vx);
  if (mono) s *= 0.5;
  const g = clamp(1 - (Math.abs(b) - 0.30) / 0.30, 0, 1), vx = s * Math.cos(b) * g, vy = clamp(s * Math.sin(b) * g, -c.max_vy, c.max_vy);
  let wz = Math.abs(b) > 0.04 ? c.kp_wz * b : 0; if (wz && Math.abs(wz) < 0.12) wz = Math.sign(wz) * 0.12; wz = clamp(wz, -c.max_wz, c.max_wz);
  return { r, e, s, g, b, cmd: [vx, vy, wz] };
}

/* ================================================================== 8. follow */
section('follow', 'Эксперимент 1: следование', `
<h2>Эксперимент 1. Как робот видит человека и едет за ним</h2>
<p class="lead">Конвейер: кадр → детектор людей (EfficientDet-Lite0, CPU) → прямоугольники → глубина в области торса → 3D-точка в системе робота → трекер (IoU + цвет одежды) → «замок» на цель → закон управления → скорость базы. Ниже всё это на живых данных. Включите превью — детектор и контроллер заработают на настоящей камере, <b>но робот не двинется</b>.</p>
<div class="row"><button class="b p" id="fw-prev">Превью восприятия: выкл</button><button class="b" id="fw-pose">Перевести руку в позу для людей (рука двинется)</button><span class="muted" id="fw-info"></span></div>
<div class="grid2">
 <div><div class="imgwrap"><img src="/video" alt="камера"><canvas id="fw-ov"></canvas></div></div>
 <div><canvas id="fw-radar" class="sq"></canvas></div></div>
<div class="grid2"><div><h3>Люди в кадре</h3><div id="fw-table"></div></div><div><h3>Закон управления с живыми числами</h3><div id="fw-law"></div></div></div>
<div class="grid3"><div><h3>Расстояние до цели, м</h3><canvas class="chart" id="ch-fd"></canvas><div class="muted">пунктир — заданная дистанция; фон — состояние контроллера</div></div>
 <div><h3>Азимут цели, °</h3><canvas class="chart" id="ch-fb"></canvas></div><div><h3>Команда базы</h3><canvas class="chart" id="ch-fc"></canvas><div id="lg-fc"></div></div></div>
<div class="formula">$$ e=r-r^*,\\quad s=\\begin{cases}-v_{back}& r\\lt r_{min}\\\\0&|e|\\le \\delta\\\\ \\mathrm{clip}\\big(k_x(e-\\mathrm{sgn}(e)\\delta),-v_{back},v_{max}\\big)&\\text{иначе}\\end{cases},\\quad \\begin{bmatrix}v_x\\\\v_y\\end{bmatrix}=s\\,g(\\beta)\\begin{bmatrix}\\cos\\beta\\\\\\sin\\beta\\end{bmatrix},\\quad w_z=k_w\\beta $$</div>
<p>К команде добавляется <b>упреждение по скорости человека</b>: скорость цели оценивается по её траектории в системе odom (прямая по МНК за 0.8 с) и прибавляется с коэффициентом 0.9. Без этого пропорциональный закон отстаёт от идущего человека на $v/k_x+\\delta$ (0.5 м при 0.25 м/с); с упреждением в симуляции дистанция держится около $r^*+\\delta$. Остановился человек — оценка падает до нуля за секунду и робот встаёт. <p>$g(\\beta)=\\mathrm{clip}\\big(1-(|\\beta|-0.3)/0.3,0,1\\big)$: при большом угле робот сначала разворачивается, и только потом едет. Скорость вдоль луча на человека раскладывается на $v_x$ и $v_y$ — поэтому на меканум-базе он смещается боком, а не только описывает дугу. Нет свежего измерения (&gt;0.7 с) — нулевая скорость; потеря цели — остановка и ограниченный поиск.</p>
<h3>Почему робот отстаёт: замкнутый контур (симулятор в браузере)</h3>
<p>Пропорциональный контроль не может идти быстрее потолка скорости, а задержка зрения (детектор ≈ 0.11 с + обработка) добавляет отставание. Подберите: человек идёт со скоростью $v_p$, робот — не быстрее $v_{max}$.</p>
<div class="grid2"><div><div class="ctl">${ctl('sm-vp', 'скорость человека', 0.1, 0.8, 0.05, 0.35, ' м/с')}${ctl('sm-vm', 'потолок робота', 0.1, 0.8, 0.05, 0.4, ' м/с')}${ctl('sm-lat', 'задержка зрения', 0, 0.8, 0.05, 0.25, ' с')}${ctl('sm-kp', 'коэффициент $k_x$', 0.2, 2, 0.05, 0.65)}<label class="muted"><input type="checkbox" id="sm-turn" checked> человек поворачивает на 90° на 15-й секунде</label></div></div>
 <div><canvas class="chart tall" id="ch-sim"></canvas><div id="sim-out" class="muted"></div></div></div>`,
async () => {
  const cfgOf = () => (Live.last && Live.last.cfg && Live.last.cfg.follow) || { standoff_m: 1.2, deadband_m: 0.12, min_dist_m: 0.7, max_back: 0.12, kp_x: 0.65, max_vx: 0.4, max_vy: 0.25, kp_wz: 1.5, max_wz: 0.8 };
  let prev = false;
  $('#fw-prev').onclick = () => { prev = !prev; wsSend({ action: 'exp_preview', people: prev, hands: false }); $('#fw-prev').textContent = 'Превью восприятия: ' + (prev ? 'вкл' : 'выкл'); $('#fw-prev').classList.toggle('on', prev); };
  $('#fw-pose').onclick = () => { if (confirm('Рука переместится в позу «камера смотрит вперёд». Вокруг руки свободно?')) wsSend({ action: 'preset', name: 'people' }); };
  const draw = throttle(L => {
    const ov = $('#fw-ov'), rc = ov.getBoundingClientRect(), g = ov.getContext('2d'); ov.width = rc.width; ov.height = rc.height; g.clearRect(0, 0, ov.width, ov.height);
    const sx = L.camera ? rc.width / L.camera.size[0] : 1, sy = L.camera ? rc.height / L.camera.size[1] : 1;
    (L.people || []).forEach(p => { const [x0, y0, x1, y1] = p.box; g.strokeStyle = p.target ? '#22c55e' : p.misses ? '#f59e0b' : '#38bdf8'; g.lineWidth = p.target ? 3 : 1.5; g.strokeRect(x0 * sx, y0 * sy, (x1 - x0) * sx, (y1 - y0) * sy);
      g.fillStyle = '#000a'; g.fillRect(x0 * sx, y0 * sy - 16, 150, 16); g.fillStyle = '#fff'; g.font = '12px sans-serif'; g.fillText(`#${p.id}${p.target ? ' ЦЕЛЬ' : ''} ${p.dist === null ? '' : f(p.dist, 2) + ' м'}${p.mono ? '~' : ''}`, x0 * sx + 3, y0 * sy - 4); });
    const cv = $('#fw-radar'), d = window.devicePixelRatio || 1, r2 = cv.getBoundingClientRect(); cv.width = r2.width * d; cv.height = r2.height * d; const c = cv.getContext('2d'), W = cv.width, H = cv.height;
    const R = 4.5, S = Math.min(W / 2 - 10, H - 30) / R, X = y => W / 2 - y * S, Y = x => H - 18 * d - x * S; const cf = cfgOf();
    c.clearRect(0, 0, W, H); c.strokeStyle = 'rgba(150,170,200,.25)'; c.fillStyle = '#9aa5b4'; c.font = `${10 * d}px sans-serif`;
    for (let r = 1; r <= 4; r++) { c.beginPath(); c.arc(X(0), Y(0), r * S, Math.PI, 2 * Math.PI); c.stroke(); c.fillText(r + ' м', X(0) + 3 * d, Y(r) - 2 * d); }
    const K = L.camera ? L.camera.K : null, half = K ? Math.atan(K[0][2] / K[0][0]) : 0.58; c.strokeStyle = '#38bdf8'; c.beginPath(); c.moveTo(X(0), Y(0)); c.lineTo(X(-Math.sin(half) * R * 1.1), Y(Math.cos(half) * R * 1.1)); c.moveTo(X(0), Y(0)); c.lineTo(X(Math.sin(half) * R * 1.1), Y(Math.cos(half) * R * 1.1)); c.stroke();
    c.strokeStyle = '#2563eb'; c.setLineDash([6 * d, 4 * d]); c.beginPath(); c.arc(X(0), Y(0), cf.standoff_m * S, Math.PI, 2 * Math.PI); c.stroke(); c.setLineDash([]);
    L.scan.forEach(([x, y]) => { c.fillStyle = 'rgba(100,130,190,.5)'; c.fillRect(X(y) - d, Y(x) - d, 2 * d, 2 * d); });
    c.fillStyle = '#7d8ba1'; c.fillRect(X(0.15) - 0.15 * S + 0.15 * S, Y(0.145), 0.3 * S, 0.305 * S); c.fillRect(X(0) - 0.15 * S, Y(0.145), 0.3 * S, 0.305 * S);
    (L.people || []).filter(p => p.xyz).forEach(p => { c.fillStyle = p.target ? '#22c55e' : p.misses ? '#f59e0b' : '#f472b6'; c.beginPath(); c.arc(X(p.xyz[1]), Y(p.xyz[0]), 9 * d, 0, 7); c.fill(); c.fillStyle = '#fff'; c.fillText('#' + p.id, X(p.xyz[1]) + 11 * d, Y(p.xyz[0]) + 4 * d); });
    if (L.follow) { const [vx, vy, wz] = L.follow.cmd; c.strokeStyle = '#22c55e'; c.lineWidth = 3 * d; c.beginPath(); c.moveTo(X(0), Y(0)); c.lineTo(X(vy * 3), Y(vx * 3)); c.stroke(); c.lineWidth = d; }
    c.fillStyle = '#e8ecf2'; c.fillText('вид сверху: вверх — вперёд; зелёная стрелка — команда базы (×3)', 8 * d, H - 4 * d);
  }, 120);
  Live.on(L => {
    draw(L); const fw = L.follow;
    $('#fw-info').textContent = `зрение ${L.perception.hz} кадр/с${L.perception.error ? ' · ошибка: ' + L.perception.error : ''}`;
    $('#fw-table').innerHTML = (L.people || []).length ? `<table class="num"><tr><th>id</th><th>score</th><th>кадров</th><th>дист, м</th><th>азимут</th><th>цвет/роль</th></tr>${L.people.map(p =>
      `<tr><td>#${p.id}</td><td>${p.score}</td><td>${p.hits}${p.misses ? ' (−' + p.misses + ')' : ''}</td><td>${p.dist === null ? '–' : f(p.dist, 2) + (p.mono ? '~' : '')}</td><td>${p.bearing === null ? '–' : f(p.bearing / D2R, 1) + '°'}</td><td>${p.target ? '<b style="color:var(--ok)">ЦЕЛЬ</b>' : 'посторонний'}</td></tr>`).join('')}</table>` : '<div class="muted">людей нет (включите превью и встаньте перед камерой на 1–3 м)</div>';
    const tg = (L.people || []).find(p => p.target && p.xyz) || (L.people || []).find(p => p.xyz && !p.misses), cf = cfgOf();
    if (tg) { const a = followLaw(tg.xyz[0], tg.xyz[1], cf, tg.mono);
      $('#fw-law').innerHTML = `<div class="kvs"><span>точка в base</span><span>x = ${f(tg.xyz[0], 2)}, y = ${f(tg.xyz[1], 2)} м</span><span>$r$, $\\beta$</span><span>${f(a.r, 2)} м, ${f(a.b / D2R, 1)}°</span><span>$e=r-r^*$</span><span>${f(a.r, 2)} − ${f(cf.standoff_m, 2)} = <b>${f(a.e, 2)}</b> м</span>
        <span>$s$ (скорость к цели)</span><span>${f(a.s, 3)} м/с ${Math.abs(a.e) <= cf.deadband_m && a.r >= cf.min_dist_m ? '(в мёртвой зоне ±' + cf.deadband_m + ')' : ''}</span><span>$g(\\beta)$</span><span>${f(a.g, 2)}</span>
        <span>команда по формуле</span><span><b>vx ${f(a.cmd[0], 2)} · vy ${f(a.cmd[1], 2)} · wz ${f(a.cmd[2], 2)}</b></span><span>команда контроллера</span><span>${fw ? fw.cmd.map(v => f(v, 2)).join(' · ') + ' (' + (STATE_RU[fw.state] || fw.state) + ')' : '–'}</span></div>${fw && fw.note ? `<div class="note">${fw.note}</div>` : ''}`;
      math($('#fw-law')); } else $('#fw-law').innerHTML = '<div class="muted">нет цели с известным расстоянием</div>';
  });
  Live.on(throttle(L => {
    const now = L.t, F = Live.follow.filter(x => now - x[0] < 60); if (!F.length) { ['ch-fd', 'ch-fb', 'ch-fc'].forEach(i => lineChart($('#' + i), [], { x0: -60, x1: 0, empty: 'нет данных: включите превью и встаньте перед камерой' })); return; }
    const t = F.map(x => x[0] - now), cols = COLORS(), bands = []; let s0 = 0;
    F.forEach((x, i) => { if (i === F.length - 1 || F[i + 1][6] !== x[6]) { bands.push({ from: t[s0], to: t[i], color: STATE_COLOR[x[6]] || '#888' }); s0 = i + 1; } });
    lineChart($('#ch-fd'), [{ name: 'r', color: cols[0], x: t, y: F.map(x => x[1]) }], { x0: -60, x1: 0, y0: 0, y1: 4, bands, hlines: [{ y: cfgOf().standoff_m, label: 'цель r*', color: cols[1] }, { y: cfgOf().min_dist_m, label: 'r_min', color: cols[3] }] });
    lineChart($('#ch-fb'), [{ name: 'β', color: cols[2], x: t, y: F.map(x => x[2]) }], { x0: -60, x1: 0, y0: -45, y1: 45, bands, hlines: [{ y: 0, color: '#888' }] });
    const cs = [3, 4, 5].map((k, i) => ({ name: ['vx', 'vy', 'wz'][i], color: cols[i], x: t, y: F.map(x => x[k]) })); lineChart($('#ch-fc'), cs, { x0: -60, x1: 0, y0: -0.9, y1: 0.9, bands }); $('#lg-fc').innerHTML = legend(cs);
  }, 400));
  /* closed loop simulator (JS port of the sim used in tests/test_exp.py) */
  const sim = () => {
    const vp = +$('#sm-vp').value, vm = +$('#sm-vm').value, lat = +$('#sm-lat').value, kp = +$('#sm-kp').value, cf = Object.assign({}, cfgOf(), { max_vx: vm, kp_x: kp }), turn = $('#sm-turn').checked;
    let rob = [0, 0, 0], per = [2.2, 0], head = 0, vel = [0, 0, 0]; const buf = [], dt = 0.05, T = [], Dd = []; let seen = { p: per.slice(), r: rob.slice() };
    for (let t = 0; t < 40; t += dt) {
      if (turn && Math.abs(t - 15) < dt / 2) head += Math.PI / 2;
      if (t < 32) per = [per[0] + vp * dt * Math.cos(head), per[1] + vp * dt * Math.sin(head)];
      buf.push({ t, p: per.slice(), r: rob.slice() }); while (buf.length && buf[0].t < t - lat) seen = buf.shift();
      const c = Math.cos(seen.r[2]), s = Math.sin(seen.r[2]), dx = seen.p[0] - seen.r[0], dy = seen.p[1] - seen.r[1], cmd = followLaw(c * dx + s * dy, -s * dx + c * dy, cf).cmd;
      const lim = [vm, cf.max_vy, cf.max_wz], acc = [0.5, 0.5, 2.0];
      for (let i = 0; i < 3; i++) { const target = clamp(cmd[i], -lim[i], lim[i]), st = acc[i] * dt; vel[i] += clamp(target - vel[i], -st, st); }
      const cr = Math.cos(rob[2]), sr = Math.sin(rob[2]); rob = [rob[0] + dt * (cr * vel[0] - sr * vel[1]), rob[1] + dt * (sr * vel[0] + cr * vel[1]), rob[2] + dt * vel[2]];
      T.push(t); Dd.push(Math.hypot(per[0] - rob[0], per[1] - rob[1]));
    }
    const cols = COLORS(), tail = Dd.slice(-100), mean = tail.reduce((a, b) => a + b, 0) / tail.length;
    lineChart($('#ch-sim'), [{ name: 'дистанция до человека', color: cols[0], x: T, y: Dd }], { x0: 0, x1: 40, y0: 0, y1: 5, hlines: [{ y: cf.standoff_m, label: 'цель r*', color: cols[1] }, { y: cf.min_dist_m, label: 'r_min', color: cols[3] }], xLabel: 'секунды' });
    $('#sim-out').innerHTML = `Итог: в конце дистанция ${f(mean, 2)} м (цель ${f(cf.standoff_m, 2)} м), минимум ${f(Math.min(...Dd), 2)} м, максимум ${f(Math.max(...Dd), 2)} м. ` + (vm < vp + 0.03 ? '<b style="color:var(--danger)">Робот медленнее человека — отстанет.</b>' : mean - cf.standoff_m > 0.25 ? 'Заметное отставание: увеличьте потолок скорости или $k_x$.' : '<b style="color:var(--ok)">Дистанция держится.</b>');
    math($('#sim-out'));
  };
  bindCtl(['sm-vp', 'sm-vm', 'sm-lat', 'sm-kp'], sim, { 'sm-vp': ' м/с', 'sm-vm': ' м/с', 'sm-lat': ' с' }); $('#sm-turn').onchange = sim; sim(); math($('#sec-follow'));
});

/* ================================================================== 9. gestures */
section('gesture', 'Эксперимент 2: жесты', `
<h2>Эксперимент 2. Как робот понимает жесты</h2>
<p class="lead">Кисть — 21 точка (MediaPipe, на увеличенном фрагменте вокруг верха тела человека). По точкам считаются признаки кадра, а жест — это <b>событие во времени</b>: удержание ладони, два цикла сгибания пальцев, рост ладони при движении к камере. Сработавший жест «защёлкивается» и не повторяется, пока руку не уберут. Включите превью (робот не двигается) и покажите руку.</p>
<div class="row"><button class="b p" id="gs-prev">Превью жестов: выкл</button><span class="muted" id="gs-info"></span></div>
<div class="grid2"><div><div class="imgwrap"><img src="/video" alt="камера"><canvas id="gs-ov"></canvas></div></div>
 <div><canvas id="gs-hand" class="sq"></canvas></div></div>
<div class="grid3"><div><h3>«Изгиб» пальцев curl</h3><canvas class="chart" id="ch-gc"></canvas><div class="muted">≈1.9 открытая ладонь, ≈1.0 кулак; пунктир — пороги (гистерезис)</div></div>
 <div><h3>Масштаб кисти, px</h3><canvas class="chart" id="ch-gs"></canvas><div class="muted">рост ≥35 % за 0.8 с = «от меня»</div></div>
 <div><h3>Положение запястья, px</h3><canvas class="chart" id="ch-gw"></canvas><div id="lg-gw"></div></div></div>
<div class="grid2"><div><h3>Состояние распознавателя</h3><div id="gs-state"></div></div><div><h3>Последние события</h3><div id="gs-events" class="muted">пока нет</div></div></div>
<div class="formula">$$ \\mathrm{curl}=\\frac{\\tfrac14\\sum_{f\\in\\{8,12,16,20\\}}\\|p_f-p_0\\|}{\\|p_9-p_0\\|},\\qquad \\text{ладонь: }\\forall f:\\ \\|p_{tip}-p_0\\|>1.1\\,\\|p_{pip}-p_0\\| $$</div>
<h3>Демонстрация алгоритма без руки</h3>
<p>Синтетические сигналы проходят через ту же логику окон и защёлки (порт на JS). Видно, как жест «ловится» один раз.</p>
<div class="row"><button class="b" data-demo="stop">ладонь 3 с</button><button class="b" data-demo="beckon">поманить</button><button class="b" data-demo="push">толкнуть</button><button class="b" data-demo="wave">махать рукой (не команда)</button></div>
<canvas class="chart tall" id="ch-demo"></canvas><div id="demo-out" class="muted"></div>`,
async () => {
  let prev = false; const EDGES = [[0, 1], [1, 2], [2, 3], [3, 4], [0, 5], [5, 6], [6, 7], [7, 8], [5, 9], [9, 10], [10, 11], [11, 12], [9, 13], [13, 14], [14, 15], [15, 16], [13, 17], [17, 18], [18, 19], [19, 20], [0, 17]];
  const KIND_COLOR = { palm: '#16a34a', point: '#2563eb', thumb: '#d97706', other: '#9aa5b4' }, GC = () => (Live.last && Live.last.cfg && Live.last.cfg.gesture) || { window_s: 1.6, hold_s: 0.7, curl_hi: 1.65, curl_lo: 1.4, beckon_cycles: 2, push_growth: 1.35, push_window_s: 0.8, release_s: 0.6, cooldown_s: 1.5 };
  $('#gs-prev').onclick = () => { prev = !prev; wsSend({ action: 'exp_preview', people: prev, hands: prev }); $('#gs-prev').textContent = 'Превью жестов: ' + (prev ? 'вкл' : 'выкл'); $('#gs-prev').classList.toggle('on', prev); };
  Live.on(throttle(L => {
    $('#gs-info').textContent = `зрение ${L.perception.hz} кадр/с`;
    const ov = $('#gs-ov'), rc = ov.getBoundingClientRect(), g = ov.getContext('2d'); ov.width = rc.width; ov.height = rc.height; g.clearRect(0, 0, ov.width, ov.height);
    const sx = L.camera ? rc.width / L.camera.size[0] : 1, sy = L.camera ? rc.height / L.camera.size[1] : 1; g.strokeStyle = '#facc15'; g.lineWidth = 2;
    (L.hands || []).forEach(h => { EDGES.forEach(([a, b]) => { g.beginPath(); g.moveTo(h.pts[a][0] * sx, h.pts[a][1] * sy); g.lineTo(h.pts[b][0] * sx, h.pts[b][1] * sy); g.stroke(); }); });
    const cv = $('#gs-hand'), d = window.devicePixelRatio || 1, r2 = cv.getBoundingClientRect(); cv.width = r2.width * d; cv.height = r2.height * d; const c = cv.getContext('2d'); c.clearRect(0, 0, cv.width, cv.height);
    const h = (L.hands || [])[0], gcfg = GC();
    if (h) { const xs = h.pts.map(p => p[0]), ys = h.pts.map(p => p[1]), x0 = Math.min(...xs), y0 = Math.min(...ys), sc = Math.min(cv.width, cv.height) * 0.8 / Math.max(Math.max(...xs) - x0, Math.max(...ys) - y0, 1);
      const P = p => [cv.width * 0.1 + (p[0] - x0) * sc, cv.height * 0.1 + (p[1] - y0) * sc]; c.strokeStyle = '#38bdf8'; c.lineWidth = 3 * d; EDGES.forEach(([a, b]) => { c.beginPath(); c.moveTo(...P(h.pts[a])); c.lineTo(...P(h.pts[b])); c.stroke(); });
      h.pts.forEach((p, i) => { const q = P(p); c.fillStyle = [4, 8, 12, 16, 20].includes(i) ? '#f472b6' : i === 0 ? '#22c55e' : '#facc15'; c.beginPath(); c.arc(q[0], q[1], 4 * d, 0, 7); c.fill(); });
      c.fillStyle = '#e8ecf2'; c.font = `${12 * d}px sans-serif`; c.fillText(`${h.label} · ${h.handed}${h.depth ? ' · ' + f(h.depth, 2) + ' м' : ''} (розовые — кончики пальцев, зелёная — запястье)`, 8 * d, cv.height - 8 * d); }
    else { c.fillStyle = '#9aa5b4'; c.font = `${13 * d}px sans-serif`; c.fillText('кисть не найдена: включите превью и поднимите руку перед камерой', 12 * d, cv.height / 2); }
    // charts
    const G = Live.gest.filter(x => L.t - x[0] < 20); const t = G.map(x => x[0] - L.t), cols = COLORS(), bands = []; let s0 = 0;
    G.forEach((x, i) => { if (i === G.length - 1 || G[i + 1][1] !== x[1]) { if (x[1]) bands.push({ from: t[s0], to: t[i], color: KIND_COLOR[x[1]] || '#888' }); s0 = i + 1; } });
    lineChart($('#ch-gc'), [{ name: 'curl', color: cols[0], x: t, y: G.map(x => x[2]) }], { x0: -20, x1: 0, y0: 0.6, y1: 2.2, bands, hlines: [{ y: gcfg.curl_hi, label: 'открыта', color: cols[1] }, { y: gcfg.curl_lo, label: 'согнута', color: cols[3] }] });
    lineChart($('#ch-gs'), [{ name: 'масштаб', color: cols[2], x: t, y: G.map(x => x[3]) }], { x0: -20, x1: 0, bands });
    const ws = [['x', 4], ['y', 5]].map(([n, k], i) => ({ name: 'запястье ' + n, color: cols[i], x: t, y: G.map(x => x[k]) })); lineChart($('#ch-gw'), ws, { x0: -20, x1: 0 }); $('#lg-gw').innerHTML = legend(ws);
    const gs = L.gesture; if (gs) {
      let run = 0, kind = null; for (let i = G.length - 1; i >= 0 && G[i][1] && (kind === null || G[i][1] === kind); i--) { kind = G[i][1]; run = G[G.length - 1][0] - G[i][0]; }
      const cyc = (() => { const w = G.filter(x => x[2] != null && L.t - x[0] < gcfg.window_s); let st = null, n = 0; w.forEach(x => { if (x[2] >= gcfg.curl_hi) { if (st === 'lo') n++; st = 'hi'; } else if (x[2] <= gcfg.curl_lo && st === 'hi') st = 'lo'; }); return n; })();
      const growth = (() => { const w = G.filter(x => x[1] === 'palm' && L.t - x[0] < gcfg.push_window_s && x[3]); return w.length > 2 ? w[w.length - 1][3] / w[0][3] : null; })();
      $('#gs-state').innerHTML = `<div class="kvs"><span>поза кисти</span><span><b>${gs.debug.kind || 'нет руки'}</b> (curl ${gs.debug.curl ?? '–'})</span><span>удержание «${kind || '–'}»</span><div class="bar"><i style="left:0;width:${clamp(run / gcfg.hold_s * 100, 0, 100)}%"></i></div>
        <span>порог удержания</span><span>${f(run, 2)} из ${gcfg.hold_s} с</span><span>циклы сгибания</span><span>${cyc} из ${gcfg.beckon_cycles} (за ${gcfg.window_s} с)</span><span>рост ладони</span><span>${growth === null ? '–' : '×' + f(growth, 2)} (нужно ≥ ×${gcfg.push_growth})</span>
        <span>защёлка</span><span>${gs.latched ? 'держит «' + gs.latched + '», ждёт, пока уберёте руку ' + gcfg.release_s + ' с' : 'свободна'}</span><span>пауза после команды</span><span>${gs.cooldown ? f(gs.cooldown, 1) + ' с' : '–'}</span></div>`;
      $('#gs-events').innerHTML = gs.events.length ? gs.events.slice(-8).reverse().map(([tt, n]) => `<div><b>${n}</b> <span class="muted">${f(L.t - tt, 1)} с назад</span></div>`).join('') : 'пока нет';
    } else $('#gs-state').innerHTML = '<div class="muted">распознаватель выключен</div>';
  }, 200));
  /* synthetic demo: JS port of the stop / beckon / push logic with the latch */
  const demo = kind => {
    const g = GC(), dt = 0.12, T = [], curl = [], scale = [], ev = []; let latched = null, relSince = null, cool = -1;
    const frames = []; for (let t = 0; t < 4.5; t += dt) {
      let c = 1.9, s = 60, present = true;
      if (kind === 'beckon') c = 1.55 + 0.5 * Math.cos(t * 2 * Math.PI / 0.85) - 0.0 + (Math.cos(t * 2 * Math.PI / 0.85) < 0 ? -0.1 : 0);
      if (kind === 'push') s = 50 + Math.max(0, t - 1) * 22;
      if (kind === 'wave') c = 1.9 - 0.02 * Math.sin(t * 5);
      if (kind === 'stop') c = 1.9;
      if (t < 0.5 || (kind === 'stop' && t > 3.6)) present = false;
      frames.push({ t, c: present ? c : null, s: present ? s : null, wx: kind === 'wave' ? 300 + 140 * Math.sin(t * 6) : 300, present });
    }
    frames.forEach((fr, i) => {
      T.push(fr.t); curl.push(fr.c); scale.push(fr.s); const win = frames.slice(0, i + 1).filter(x => fr.t - x.t <= g.window_s && x.present);
      if (latched) { if (!fr.present) { relSince = relSince ?? fr.t; if (fr.t - relSince >= g.release_s) { latched = null; relSince = null; } } else relSince = null; }
      if (latched || fr.t < cool || !fr.present) return;
      let name = null;
      const pw = win.filter(x => fr.t - x.t <= g.push_window_s); if (pw.length > 2 && pw[pw.length - 1].s / pw[0].s >= g.push_growth) name = 'away';
      if (!name) { let st = null, n = 0; win.forEach(x => { if (x.c >= g.curl_hi) { if (st === 'lo') n++; st = 'hi'; } else if (x.c <= g.curl_lo && st === 'hi') st = 'lo'; }); const wx = win.map(x => x.wx); if (n >= g.beckon_cycles && Math.max(...wx) - Math.min(...wx) < 130) name = 'come'; }
      if (!name) { const hw = win.filter(x => fr.t - x.t <= g.hold_s); const cs = hw.map(x => x.c), wx = hw.map(x => x.wx); if (hw.length >= 3 && hw[hw.length - 1].t - hw[0].t >= 0.85 * g.hold_s && Math.max(...cs) - Math.min(...cs) <= 0.3 && Math.max(...wx) - Math.min(...wx) < 100 && hw[hw.length - 1].s / hw[0].s < 1.18) name = 'stop'; }
      if (name) { ev.push([fr.t, name]); latched = name; cool = fr.t + g.cooldown_s; }
    });
    const cols = COLORS(); lineChart($('#ch-demo'), [{ name: 'curl', color: cols[0], x: T, y: curl }, { name: 'масштаб/40', color: cols[2], x: T, y: scale.map(s => s == null ? null : s / 40) }], { x0: 0, x1: 4.5, y0: 0.6, y1: 2.4, hlines: [{ y: g.curl_hi, color: cols[1], label: 'curl_hi' }, { y: g.curl_lo, color: cols[3], label: 'curl_lo' }], bands: ev.map(([t]) => ({ from: t, to: t + 0.06, color: '#22c55e' })), xLabel: 'секунды (зелёная полоса — команда)' });
    $('#demo-out').innerHTML = ev.length ? `Сработало: ${ev.map(([t, n]) => `<b>${n}</b> на ${f(t, 1)} с`).join(', ')} — одна команда, хотя жест держится дальше.` : 'Команда не сработала: это не жест (или он слишком короткий) — так и задумано.';
  };
  document.querySelectorAll('[data-demo]').forEach(b => b.onclick = () => demo(b.dataset.demo)); demo('stop'); math($('#sec-gesture'));
});

/* ================================================================== 10. SLAM and Nav2 */
section('nav', 'Эксперимент 3: карта и навигация', `
<h2>Эксперимент 3. Карта, исследование и навигация</h2>
<p class="lead">Карту строит готовый <b>slam_toolbox</b> по слитому скану двух лидаров; маршруты и объезд — <b>Nav2</b> (планировщик A* и локальный контроллер DWB для всенаправленной базы). Наш код — склейка: скан, одометрия, TF и мост скорости в защитный слой. Здесь — живая карта и TF-цепочка (когда эксперимент запущен), а ниже — три интерактивных модели того, как Nav2 принимает решения.</p>
<div class="grid2"><div><img id="nv-map" alt="карта" style="width:100%;border-radius:10px;background:#0b0f16;min-height:200px"><div id="nv-info" class="muted"></div></div><div><div id="nv-tf"></div><div id="nv-wp"></div></div></div>
<div class="formula">$$ {}^{map}T_{base}={}^{map}T_{odom}\\cdot{}^{odom}T_{base}\\qquad ({}^{map}T_{odom}\\text{ публикует SLAM, }{}^{odom}T_{base}\\text{ — наша одометрия}) $$</div>
<h3>Фронтиры: куда ехать за новой картой</h3>
<p>Кликайте по клеткам: неизвестно → свободно → стена. Фронтир — свободная клетка, у которой есть неизвестный сосед. Лидарные лучи оставляют одноклеточные «дырки» — перед поиском карту закрывают морфологически, иначе робот гонялся бы за шумом.</p>
<div class="grid2"><div><canvas id="nv-fr" class="wide"></canvas></div><div id="nv-frout"></div></div>
<h3>Стоимостная карта: почему коридор должен быть шире робота</h3>
<div class="grid2"><div><div class="ctl">${ctl('nv-k', 'cost_scaling_factor', 1, 12, 0.5, 6)}${ctl('nv-r', 'inflation_radius', 0.2, 0.8, 0.01, 0.3, ' м')}${ctl('nv-ins', 'вписанный радиус', 0.1, 0.25, 0.005, 0.15, ' м')}</div></div><div><canvas class="chart" id="ch-nv-cost"></canvas></div></div>
<div class="formula">$$ \\mathrm{cost}(d)=\\begin{cases}254&d\\le r_{circ}\\;(\\text{столкновение})\\\\253&d\\le r_{ins}\\;(\\text{вписанная окружность})\\\\252\\,e^{-k\\,(d-r_{ins})}&r_{ins}\\lt d\\lt R_{infl}\\\\0&\\text{иначе}\\end{cases} $$</div>
<div id="nv-cost-out" class="muted"></div>
<h3>DWB: как контроллер выбирает скорость (клик — цель, Shift+клик — препятствие)</h3>
<p>Контроллер перебирает сетку скоростей $(v_x,v_y,w_z)$, «проигрывает» каждую на 1.6 с вперёд и выбирает лучшую по сумме критериев. Для меканум-базы $v_y\\neq0$ разрешена — траектории могут скользить боком вокруг препятствий.</p>
<div class="grid2"><div><canvas id="nv-dwb" class="sq" style="cursor:crosshair"></canvas></div><div><div class="ctl">${ctl('dw-vx', 'max vx', 0.05, 0.4, 0.01, 0.18, ' м/с')}${ctl('dw-vy', 'max vy', 0, 0.3, 0.01, 0.12, ' м/с')}${ctl('dw-wz', 'max wz', 0.1, 1.2, 0.05, 0.55, ' рад/с')}</div><div id="dw-out"></div><div class="note">Реальные пределы этих экспериментов: vx ≤ 0.18, vy ≤ 0.12, wz ≤ 0.55 (настройки <code>exp.nav</code>); поверх Nav2 всегда работает лидарный барьер базы.</div></div></div>`,
async () => {
  const img = $('#nv-map'); let navData = null;
  const refresh = async () => { if (document.hidden) return; try { navData = await api('/api/exp/nav'); } catch (e) { return; }
    const L = Live.last, n = navData;
    if (n.running) { img.src = '/api/exp/map.png?' + Date.now(); const [mx, my, myaw] = n.map_odom || [0, 0, 0], [x, y, yaw] = L ? L.pose : [0, 0, 0];
      const Tmo = [[Math.cos(myaw), -Math.sin(myaw), 0, mx], [Math.sin(myaw), Math.cos(myaw), 0, my], [0, 0, 1, 0], [0, 0, 0, 1]], Tob = [[Math.cos(yaw), -Math.sin(yaw), 0, x], [Math.sin(yaw), Math.cos(yaw), 0, y], [0, 0, 1, 0], [0, 0, 0, 1]];
      const P = m4rows(m4of(Tmo).multiply(m4of(Tob)));
      $('#nv-tf').innerHTML = `<div class="rowmat"><span class="l"><sup>map</sup>T<sub>odom</sub> (SLAM)</span>${mat(Tmo, 3, (i, j) => j === 3 && i < 2)}</div><div class="rowmat"><span class="l"><sup>odom</sup>T<sub>base</sub></span>${mat(Tob, 3, (i, j) => j === 3 && i < 2)}</div>
        <div class="rowmat"><span class="l"><sup>map</sup>T<sub>base</sub> = произведение</span>${mat(P, 3, (i, j) => j === 3 && i < 2)}</div><div class="muted">Поза на карте из TF: ${n.pose_map ? n.pose_map.map(v => f(v, 3)).join(', ') : '–'}</div>`;
      $('#nv-info').textContent = n.map ? `карта ${n.map.size[1]}×${n.map.size[0]} клеток по ${n.map.res} м, известно ${n.map.known} клеток (${f(n.map.known * n.map.res * n.map.res, 1)} м²), границ неизвестного: ${n.frontiers.length}, целей отправлено: ${n.goals.length}` : 'карты ещё нет'; }
    else { img.src = '/api/exp/map.png?' + Date.now(); $('#nv-tf').innerHTML = '<div class="note">SLAM и Nav2 сейчас не запущены. Запустите «Построить карту» во вкладке «Эксперименты»: здесь появятся живая карта, траектория и матрицы TF.</div>'; $('#nv-info').textContent = ''; }
    const w = n.waypoints || {}; $('#nv-wp').innerHTML = Object.keys(w).length ? `<table class="num"><tr><th>точка</th><th>x</th><th>y</th><th>yaw</th></tr>${Object.entries(w).map(([k, v]) => `<tr><td>${k}</td><td>${v.x}</td><td>${v.y}</td><td>${f(v.yaw / D2R, 0)}°</td></tr>`).join('')}</table>` : '';
    math($('#nv-tf'));
  };
  refresh(); setInterval(refresh, 2500);
  /* frontier painter */
  const GW = 24, GH = 14; const grid = Array.from({ length: GH }, (_, y) => Array.from({ length: GW }, (_, x) => (x >= 3 && x <= 13 && y >= 3 && y <= 10) ? 1 : 0));
  const drawFr = () => { const cv = $('#nv-fr'), d = window.devicePixelRatio || 1, r = cv.getBoundingClientRect(); cv.width = r.width * d; cv.height = r.height * d; const g = cv.getContext('2d'), s = Math.min(cv.width / GW, cv.height / GH); let n = 0;
    for (let y = 0; y < GH; y++) for (let x = 0; x < GW; x++) { g.fillStyle = grid[y][x] === 2 ? '#111' : grid[y][x] === 1 ? '#e8eaf0' : '#6b7280'; g.fillRect(x * s, y * s, s - 1, s - 1); }
    for (let y = 0; y < GH; y++) for (let x = 0; x < GW; x++) if (grid[y][x] === 1 && [[1, 0], [-1, 0], [0, 1], [0, -1]].some(([dx, dy]) => grid[y + dy] && grid[y + dy][x + dx] === 0)) { n++; g.fillStyle = '#f59e0b'; g.beginPath(); g.arc(x * s + s / 2, y * s + s / 2, s * 0.22, 0, 7); g.fill(); }
    $('#nv-frout').innerHTML = `<div class="note ${n ? '' : 'ok'}">Фронтиров: <b>${n}</b>. ${n ? 'Робот выберет достижимую по свободным клеткам группу (ближе и крупнее лучше), отправит цель в Nav2; не доехал — группа уходит в чёрный список на 0.5 м.' : 'Исследовать больше нечего: карта сохраняется в <code>~/CubeBot/maps/</code>.'}</div>`; };
  $('#nv-fr').onclick = e => { const r = e.target.getBoundingClientRect(), s = Math.min(r.width / GW, r.height / GH), x = Math.floor((e.clientX - r.left) / s), y = Math.floor((e.clientY - r.top) / s); if (grid[y] && grid[y][x] !== undefined) { grid[y][x] = (grid[y][x] + 1) % 3; drawFr(); } }; drawFr();
  /* inflation profile */
  const cost = () => { const k = +$('#nv-k').value, R = +$('#nv-r').value, ins = +$('#nv-ins').value, ds = [], cs = []; for (let d = 0; d <= 0.8; d += 0.005) { ds.push(d); cs.push(d <= 0.0 ? 254 : d <= ins ? 253 : d < R ? 252 * Math.exp(-k * (d - ins)) : 0); }
    lineChart($('#ch-nv-cost'), [{ name: 'cost', color: COLORS()[0], x: ds, y: cs }], { x0: 0, x1: 0.8, y0: 0, y1: 260, hlines: [{ y: 253, label: 'непроходимо', color: COLORS()[3] }], xLabel: 'расстояние до препятствия, м' });
    $('#nv-cost-out').innerHTML = `Ширина робота ≈ 0.30 м (вписанная окружность ${f(ins * 100, 0)} см). Центр не может быть ближе ${f(ins * 100, 0)} см к стене, поэтому минимальный проход ≈ <b>${f(2 * ins * 100, 0)} см + запас</b>. Чем больше $k$, тем быстрее «страх» перед стеной убывает.`; math($('#nv-cost-out')); };
  bindCtl(['nv-k', 'nv-r', 'nv-ins'], cost, { 'nv-r': ' м', 'nv-ins': ' м' }); cost();
  /* DWB sampler */
  let goal = [0.95, 0.3], obs = [[0.5, 0.0], [0.75, 0.45]], yaw0 = 0;
  const dwb = () => { const cv = $('#nv-dwb'), d = window.devicePixelRatio || 1, r = cv.getBoundingClientRect(); cv.width = r.width * d; cv.height = r.height * d; const g = cv.getContext('2d'), W = cv.width, H = cv.height, S = Math.min(W, H) / 1.7, X = x => 20 * d + x * S, Y = y => H / 2 - y * S;
    g.clearRect(0, 0, W, H); g.strokeStyle = 'rgba(150,170,200,.2)'; for (let i = 0; i < 4; i += 0.5) { g.beginPath(); g.moveTo(X(i), 0); g.lineTo(X(i), H); g.stroke(); }
    const mx = +$('#dw-vx').value, my = +$('#dw-vy').value, mw = +$('#dw-wz').value, lin = (a, b, n) => Array.from({ length: n }, (_, i) => a + (b - a) * i / (n - 1)); let best = null, cnt = 0;
    obs.forEach(o => { g.fillStyle = '#ef4444'; g.beginPath(); g.arc(X(o[0]), Y(o[1]), 0.15 * S, 0, 7); g.fill(); g.strokeStyle = '#ef444488'; g.beginPath(); g.arc(X(o[0]), Y(o[1]), 0.3 * S, 0, 7); g.stroke(); });
    const trajs = []; for (const vx of lin(-0.08, mx, 9)) for (const vy of lin(-my, my, 9)) for (const wz of lin(-mw, mw, 9)) {
      let x = 0, y = 0, th = yaw0, ok = true, minObs = 9; const pts = [[x, y]]; for (let t = 0; t < 1.6; t += 0.1) { x += (vx * Math.cos(th) - vy * Math.sin(th)) * 0.1; y += (vx * Math.sin(th) + vy * Math.cos(th)) * 0.1; th += wz * 0.1; pts.push([x, y]); obs.forEach(o => { const dd = Math.hypot(x - o[0], y - o[1]); minObs = Math.min(minObs, dd); if (dd < 0.31) ok = false; }); }
      cnt++; const dg = Math.hypot(goal[0] - x, goal[1] - y), head = Math.abs(Math.atan2(goal[1] - y, goal[0] - x) - th), score = dg + 0.15 * Math.min(head, 3.14) + (minObs < 0.6 ? 0.1 * (0.6 - minObs) : 0);
      trajs.push({ pts, ok, score, v: [vx, vy, wz] }); if (ok && (!best || score < best.score)) best = trajs[trajs.length - 1];
    }
    trajs.forEach(tr => { g.strokeStyle = tr.ok ? 'rgba(100,150,255,.12)' : 'rgba(239,68,68,.07)'; g.beginPath(); tr.pts.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.stroke(); });
    if (best) { g.strokeStyle = '#22c55e'; g.lineWidth = 3 * d; g.beginPath(); best.pts.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.stroke(); g.lineWidth = d; }
    g.fillStyle = '#facc15'; g.beginPath(); g.arc(X(goal[0]), Y(goal[1]), 7 * d, 0, 7); g.fill(); g.fillStyle = '#7d8ba1'; g.fillRect(X(-0.15), Y(0.15), 0.3 * S, 0.3 * S);
    $('#dw-out').innerHTML = best ? `<div class="kvs"><span>проиграно траекторий</span><span>${cnt} (без столкновений: ${trajs.filter(t => t.ok).length})</span><span>лучшая скорость</span><span><b>vx ${f(best.v[0], 2)} · vy ${f(best.v[1], 2)} · wz ${f(best.v[2], 2)}</b></span><span>оценка</span><span>${f(best.score, 3)} (расстояние до цели + поворот + близость к препятствиям)</span></div>` : '<div class="note bad">Безопасной траектории нет: Nav2 включит поведения восстановления (подождать, отъехать назад).</div>'; };
  $('#nv-dwb').addEventListener('click', e => { const cv = e.currentTarget, r = cv.getBoundingClientRect(), S = Math.min(r.width, r.height) / 1.7, x = (e.clientX - r.left - 20) / S, y = (r.height / 2 - (e.clientY - r.top)) / S; if (e.shiftKey) { obs.push([x, y]); if (obs.length > 6) obs.shift(); } else goal = [x, y]; dwb(); });
  bindCtl(['dw-vx', 'dw-vy', 'dw-wz'], dwb, { 'dw-vx': ' м/с', 'dw-vy': ' м/с', 'dw-wz': ' рад/с' }); dwb(); math($('#sec-nav')); addEventListener('resize', () => { drawFr(); cost(); dwb(); });
});

/* ================================================================== 11. find and retrieve */
section('retrieve', 'Эксперимент 4: найти и принести', `
<h2>Эксперимент 4. Найти предмет, схватить, принести</h2>
<p class="lead">Конвейер кубика проверен на роботе, поэтому он используется как есть: обзор → подъезд по принципу «посмотрел–сдвинулся» → камера в контуре обратной связи (рука подходит ступеньками) → IK → захват с передавливанием → проверка удержания. Добавлены поиск по слову, проверка «влезает ли в губки», доставка и проверка камерой.</p>
<div id="rt-flow"></div>
<h3>Захват: сколько нужно сжать губки</h3>
<div class="grid2"><div><div class="ctl">${ctl('gr-w', 'ширина предмета', 15, 100, 1, 35, ' мм')}${ctl('gr-o', 'передавливание', 0, 70, 1, 55, '°')}${ctl('gr-e', 'после неудач +', 0, 20, 5, 0, '°')}</div><div id="gr-out"></div></div><div><canvas class="chart" id="ch-grip"></canvas></div></div>
<div class="formula">$$ \\theta_{contact}=\\theta_{close}-\\frac{w}{w_{open}}\\,(\\theta_{close}-\\theta_{open}),\\qquad \\theta_{cmd}=\\min(\\theta_{hold},\\ \\theta_{contact}+\\theta_{over}+\\theta_{extra}) $$</div>
<p>Серво — позиционный: сила зажима растёт с разницей между командой и реальным углом, поэтому команда заметно дальше угла касания. Обратная связь серво настоящая: на кубике оно останавливается на ≈13° раньше команды, пустое — доходит. Это и есть проверка удержания.</p>
<h3>Ступени подхода к кубику (реальная IK)</h3>
<div class="grid2"><div><canvas id="c-hover" class="viewer"></canvas></div><div><div class="ctl">${ctl('hv-s', 'ступень (расстояние до центра)', 0, 3, 1, 0)}${ctl('hv-x', 'кубик x', 15, 30, 0.5, 22, ' см')}</div><div id="hv-out"></div></div></div>`,
async () => {
  const gcfg = () => (Live.last && Live.last.cfg) || {};
  const states = [['PREFLIGHT', 'проверка'], ['SEARCH_CUBE', 'поиск'], ['APPROACH_CUBE', 'подъезд'], ['GRASP', 'захват'], ['DELIVER', 'доставка'], ['VERIFY', 'проверка камерой'], ['DONE', 'готово']];
  Live.on(throttle(L => { const e = L.exps.e4_retrieve, m = e.metrics || {}, cur = m.state, st = m.stages || {};
    $('#rt-flow').innerHTML = `<div class="chips">${states.map(([k, ru], i) => `<span class="chip ${cur === k ? 'now' : ''}">${i + 1}. ${ru}</span>`).join('<span class="muted">→</span>')}</div>
      <div class="chips">${[['detected', 'найден'], ['reached', 'подъехал'], ['grasped', 'схвачен'], ['delivered', 'доставлен (камера)']].map(([k, ru]) => `<span class="chip ${st[k] ? 'y' : ''}">${ru}</span>`).join('')}</div>
      <div class="muted">эксперимент: <b>${e.state}</b>${e.message ? ' · ' + e.message : ''}${e.result ? ' · итог: ' + e.result : ''} (живое состояние)</div>`; }, 500));
  const grip = () => { const w = +$('#gr-w').value / 1000, over = +$('#gr-o').value, ex = +$('#gr-e').value, open = 30, close = 125, wo = 0.11, hold = 165, contact = close - Math.min(w, wo) / wo * (close - open), cmd = Math.round(Math.min(hold, contact + over + ex));
    const xs = [], ys = []; for (let a = open; a <= close; a += 1) { xs.push(a); ys.push(wo * (close - a) / (close - open) * 1000); }
    lineChart($('#ch-grip'), [{ name: 'раскрытие губок', color: COLORS()[0], x: xs, y: ys }], { x0: 30, x1: 125, y0: 0, y1: 120, hlines: [{ y: w * 1000, label: 'предмет', color: COLORS()[2] }, { y: 95, label: 'предел: влезает в губки', color: COLORS()[3] }], xLabel: 'угол серво захвата, °' });
    $('#gr-out').innerHTML = `<div class="kvs"><span>угол касания</span><span>${f(contact, 1)}°</span><span>команда</span><span><b>${cmd}°</b> (${cmd >= hold ? 'упёрлись в предел 165°' : 'в пределах'})</span><span>ожидаемая остановка</span><span>≈${f(contact, 0)}°: пустое серво дойдёт до ${cmd}°, на предмете остановится раньше — разница > 6° означает «держу»</span></div>${w > 0.095 ? '<div class="note bad">Шире раскрытия губок: эксперимент сообщит «недоступен для захвата».</div>' : ''}`; };
  bindCtl(['gr-w', 'gr-o', 'gr-e'], grip, { 'gr-w': ' мм', 'gr-o': '°', 'gr-e': '°' }); grip();
  const sc = new Scene3D($('#c-hover'), { cam: [0.55, -0.6, 0.4], target: [0.15, 0, 0.08], floor: 0.5 }); await sc.addRobot({ opacity: 0.8 }); sc.sphere('cube', 0xfacc15, 0.0175); sc.axes('tcp', 0.04, '');
  const H = [0.16, 0.10, 0.05, 0.0]; let busy = false, again = false;
  const hover = async () => { if (busy) { again = true; return; } busy = true; const k = +$('#hv-s').value, x = +$('#hv-x').value / 100, p = 70 * D2R, cube = [x, 0, 0.0175], axis = [Math.cos(p), 0, -Math.sin(p)];
    const t = cube.map((v, i) => v - H[k] * axis[i]); sc.place('cube', ...cube);
    try { const r = await api('/api/exp/ik', { target: t, pitch: 70, seed: [90, 130, 0, 25, 90, 30] });
      if (r.ok) { sc.setServo(r.servo); sc.setAxes('tcp', new THREE.Matrix4().makeTranslation(...r.tcp)); }
      $('#hv-out').innerHTML = r.ok ? `<div class="kvs"><span>цель кисти</span><span>${t.map(v => f(v, 3)).join(', ')} м (куб минус ${f(H[k] * 100, 0)} см вдоль оси захвата 70°)</span><span>серво</span><span>${r.servo.slice(0, 5).map(v => f(v, 0)).join(', ')}°</span><span>остаток</span><span>${f(r.residual_m * 1000, 2)} мм · столкновений нет: ${r.collision_free ? '✓' : '✗'}</span></div><div class="muted">На каждой ступени камера на запястье заново измеряет ошибку (допуск 7–8 мм) и рука подправляется — абсолютные координаты не используются вслепую.</div>` : `<div class="note bad">IK не нашла решения для этой ступени (кубик вне зоны досягаемости): подъезд остановится и сдвинет базу.</div>`;
    } catch (e) { $('#hv-out').textContent = String(e); }
    busy = false; if (again) { again = false; hover(); } };
  bindCtl(['hv-s', 'hv-x'], hover, { 'hv-x': ' см' }); hover(); math($('#sec-retrieve')); addEventListener('resize', grip);
});

/* ================================================================== 12. assistant */
section('assistant', 'Эксперимент 5: помощник', `
<h2>Эксперимент 5. Помощник: принести и отдать в руку</h2>
<p class="lead">Сценарий склеивает предыдущие четыре: запомнить, кто просит → найти и схватить предмет → найти человека по цвету одежды → подъехать → увидеть протянутую руку → проверить, достаёт ли рука → предъявить предмет → <b>разжать захват только после физического подтверждения</b> → проверить, что предмет в руке. Живой статус сценария:</p>
<div id="as-steps"></div>
<h3>Подтверждение передачи: сигнал рывка серво (живой)</h3>
<p>Пока человек не потянул предмет, измеренные углы серво совпадают с командой. Рывок — это отклонение измеренного угла от команды больше порога на суставах 2–4, либо исчезновение «упора» захвата (серво, остановившееся на предмете, вдруг доходит до команды). Один рывок без руки в кадре у захвата не считается.</p>
<div class="grid2"><div><canvas class="chart tall" id="ch-tug"></canvas><div id="lg-tug"></div></div><div><div id="as-tug"></div></div></div>
<div class="formula">$$ \\mathrm{tug}\\iff \\Big(\\max_{i\\in\\{2,3,4\\}}\\big(|\\hat q_i-q_i^{cmd}|-|\\hat q_i^{0}-q_i^{cmd}|\\big)\\ge 4^{\\circ}\\ \\lor\\ \\text{gap}_{grip}:\\ >6^{\\circ}\\to<3^{\\circ}\\Big)\\ \\land\\ \\text{рука у захвата}\\ (<1.5\\text{ с назад}) $$</div>
<h3>Достижима ли рука человека? (настоящая IK)</h3>
<div class="grid2"><div><div class="ctl">${ctl('as-z', 'высота ладони', 10, 130, 1, 95, ' см')}${ctl('as-x', 'расстояние от центра базы', 15, 70, 1, 35, ' см')}${ctl('as-p', 'наклон кисти вниз', 0, 60, 15, 15, '°')}</div><div id="as-out"></div></div><div><canvas id="c-hand" class="viewer"></canvas></div></div>
<div class="note">Цель берётся на 6 см перед ладонью (в сторону робота), чтобы предмет оказался между пальцами. Максимальная высота кончика захвата у этой руки ≈ 0.54 м (рука вертикально вверх), поэтому ладонь стоящего человека на высоте груди недостижима — ассистент честно запишет «передача в руку недостижима, функция не завершена». Работать будет для низко протянутой руки (сидящий человек, рука на колене).</div>`,
async () => {
  const names = { command: 'команда', requester: 'запомнил человека', found_object: 'нашёл предмет', grasped: 'схватил', found_person: 'нашёл человека', approached: 'подъехал', hand_seen: 'увидел руку', reachable: 'рука достижима (IK)', pulled: 'человек взял', handed_over: 'передача подтверждена' };
  Live.on(throttle(L => { const e = L.exps.e5_assistant, m = e.metrics || {}, st = m.steps || {};
    $('#as-steps').innerHTML = `<div class="chips">${Object.entries(names).map(([k, ru]) => `<span class="chip ${k in st ? (st[k] ? 'y' : 'n') : ''}">${ru}</span>`).join('')}</div>
      <div class="muted">состояние: <b>${e.state}</b>${m.step ? ' · шаг: ' + m.step : ''}${e.result ? ' · ' + e.result : ''}</div>${m.evidence && m.evidence.reachable ? `<div class="muted">IK: ${m.evidence.reachable.why || ''}</div>` : ''}`;
    const h = Live.hist, t = ago(h.t, L.t), cols = COLORS(), dev = i => h.armMeas.map((m, k) => m[i] - h.armCmd[k][i]);
    const ser = [1, 2, 3].map((i, k) => ({ name: JOINT_RU[i] + ' (измер − команда)', color: cols[k], x: t, y: dev(i) })).concat([{ name: 'захват (измер − команда)', color: cols[3], x: t, y: dev(5) }]);
    const tug = (L.cfg && L.cfg.assistant && L.cfg.assistant.tug_deg) || 4; lineChart($('#ch-tug'), ser, { x0: -60, x1: 0, y0: -20, y1: 20, hlines: [{ y: tug, label: 'порог рывка', color: cols[2] }, { y: -tug, color: cols[2] }], xLabel: 'секунды назад' }); $('#lg-tug').innerHTML = legend(ser);
    const meas = L.arm.measured; $('#as-tug').innerHTML = meas ? `<div class="kvs">${[1, 2, 3, 5].map(i => `<span>${JOINT_RU[i]}</span><span>команда ${f(L.arm.cmd[i], 0)}° · измерено ${f(meas[i], 0)}° · разница <b>${f(meas[i] - L.arm.cmd[i], 1)}°</b></span>`).join('')}</div><div class="muted">Серво захвата, держащее кубик, останавливается заметно раньше команды (≈13°): это нормальное состояние «держу», а не ошибка.</div>` : '<div class="muted">обратная связь серво недоступна</div>'; }, 400));
  const sc = new Scene3D($('#c-hand'), { cam: [1.0, -0.9, 0.9], target: [0.3, 0, 0.45], floor: 1.2 }); await sc.addRobot({ opacity: 0.7 }); sc.sphere('palm', 0xf472b6, 0.04); sc.sphere('goal', 0xfacc15, 0.015); sc.person('pp', 0x5b8def);
  let busy = false, again = false; const go = async () => { if (busy) { again = true; return; } busy = true; const z = +$('#as-z').value / 100, x = +$('#as-x').value / 100, p = +$('#as-p').value, tgt = [x - 0.06, 0, z]; sc.place('palm', x, 0, z); sc.place('goal', ...tgt);
    const pp = sc.named.pp; pp.visible = true; pp.position.set(x + 0.2, 0, 0);
    if (z > 0.56) { $('#as-out').innerHTML = `<div class="note bad"><b>Недостижимо:</b> ладонь на ${f(z * 100, 0)} см выше предела ≈ 54 см. Ассистент запишет «передача в руку недостижима» и не будет притворяться, что передал.</div>`; sc.setServo([90, 90, 90, 90, 90, 30]); busy = false; return; }
    try { const r = await api('/api/exp/ik', { target: tgt, pitch: p, seed: [90, 90, 90, 10, 90, 30] });
      if (r.ok) sc.setServo(r.servo);
      $('#as-out').innerHTML = r.ok ? `<div class="note ${r.collision_free && r.within_limits ? 'ok' : 'bad'}"><b>Достижимо</b> (IK за ${f(r.ms, 0)} мс, остаток ${f(r.residual_m * 1000, 1)} мм). Серво: ${r.servo.slice(0, 5).map(v => f(v, 0)).join(', ')}°. ${r.collision_free ? '' : 'Но поза задевает корпус — ассистент её отвергнет.'}</div>` : `<div class="note bad"><b>Нет решения IK</b> для x = ${f(tgt[0] * 100, 0)} см, z = ${f(z * 100, 0)} см при наклоне ${p}°: ассистент попробует сдвинуться базой, потом сообщит «недостижимо».</div>`;
    } catch (e) { $('#as-out').textContent = String(e); }
    busy = false; if (again) { again = false; go(); } };
  bindCtl(['as-z', 'as-x', 'as-p'], go, { 'as-z': ' см', 'as-x': ' см', 'as-p': '°' }); go(); math($('#sec-assistant'));
});

/* ================================================================== 13. results */
section('results', 'Результаты испытаний', `
<h2>Результаты физических испытаний</h2>
<p class="lead">Здесь только то, что записано на настоящем роботе (<code>~/CubeBot/results/…/trials.jsonl</code>). Пока испытаний нет — графики пустые, это честное состояние: ни один эксперимент не считается пройденным без своих испытаний.</p>
<div id="rs-e1"></div><div id="rs-e2"></div><div id="rs-e3"></div><div id="rs-e4"></div><div id="rs-e5"></div>`,
async () => {
  const load = k => api('/api/exp/results/' + k).catch(() => []);
  const bars = (title, rows, fmt = v => v) => `<div class="kvs" style="grid-template-columns:200px 1fr 70px;align-items:center">${rows.map(([k, v, max]) => `<span>${k}</span><div class="bar"><i style="left:0;width:${clamp(v / (max || 1) * 100, 0, 100)}%"></i></div><span>${fmt(v)}</span>`).join('')}</div>`;
  const refresh = async () => {
    const [e1, e2, e3, e4, e5] = await Promise.all(['e1_follow', 'e2_gesture', 'e3_nav', 'e4_retrieve', 'e5_assistant'].map(load));
    const T = { straight: 'идёт прямо', turn90: 'поворот 90°', stop: 'остановка', occlusion: 'за препятствием', two_people: 'два человека' };
    $('#rs-e1').innerHTML = `<h3>Эксперимент 1: следование (${e1.length})</h3>` + (e1.length ? bars('', Object.keys(T).map(k => { const a = e1.filter(r => r.type === k); return [T[k] + ` (${a.length} исп., успех ${a.filter(r => r.operator_verdict === 'success').length})`, a.length ? a.reduce((s, r) => s + (r.dist_rms_error || 0), 0) / a.length : 0, 0.6]; }), v => f(v, 2) + ' м rms') +
      `<div class="muted">Потери цели всего: ${e1.reduce((s, r) => s + (r.target_losses || 0), 0)}; неверная цель по отметке оператора: ${e1.filter(r => r.correct_target === false).length}. Полоса — ошибка дистанции (rms) относительно заданной.</div>` : '<div class="muted">испытаний нет</div>');
    const G = ['stop', 'come', 'away', 'left', 'right', 'resume'], tr = e2.filter(r => r.kind === 'trial'), cols = [...G, null];
    const cell = (a, b) => tr.filter(r => r.expect === a && (r.got || null) === b).length;
    $('#rs-e2').innerHTML = `<h3>Эксперимент 2: жесты (${tr.length} попыток)</h3>` + (tr.length ? `<table class="num"><tr><th>ожидали ↓ / получили →</th>${cols.map(c => `<th>${c || 'нет реакции'}</th>`).join('')}<th>верно</th></tr>${G.map(a => { const n = tr.filter(r => r.expect === a).length, ok = tr.filter(r => r.expect === a && r.correct).length;
      return `<tr><td>${a}</td>${cols.map(b => { const v = cell(a, b), pc = n ? v / n : 0; return `<td style="background:${a === b ? `rgba(34,197,94,${pc * 0.6})` : `rgba(239,68,68,${pc * 0.6})`}">${v || ''}</td>`; }).join('')}<td>${n ? Math.round(100 * ok / n) + ' %' : '–'}</td></tr>`; }).join('')}</table><div class="muted">Критерий — не менее 90 % верных реакций (по движению, а не только по метке), без опасных ложных срабатываний. Сейчас: ${Math.round(100 * tr.filter(r => r.correct).length / tr.length)} %.</div>` : '<div class="muted">попыток нет</div>');
    const tours = e3.filter(r => r.kind === 'tour');
    $('#rs-e3').innerHTML = `<h3>Эксперимент 3: маршруты (${tours.length})</h3>` + (tours.length ? tours.slice(-6).map(t => `<div><b>${t.time}</b> ${t.label}${t.new_obstacle ? ' + препятствие' : ''}: ${t.ok ? '<span style="color:var(--ok)">успех</span>' : '<span style="color:var(--danger)">не завершён</span>'}, путь ${t.path_m} м</div>` + bars('', t.results.map(r => [`${r.label} (${r.status}, перепл. ${r.replans})`, r.err_xy_m ?? 0, 0.5]), v => f(v * 100, 0) + ' см')).join('') : '<div class="muted">маршрутов нет</div>');
    const t4 = e4.filter(r => r.kind === 'trial'), tg = [...new Set(t4.map(r => r.target))];
    $('#rs-e4').innerHTML = `<h3>Эксперимент 4: найти и принести (${t4.length})</h3>` + (t4.length ? tg.map(k => { const a = t4.filter(r => r.target === k), n = a.length; return `<div><b>${k}</b> — ${n} попыток</div>` + bars('', [['найден', a.filter(r => r.stages.detected).length, n], ['подъехал', a.filter(r => r.stages.reached).length, n], ['схвачен', a.filter(r => r.stages.grasped).length, n], ['доставлен (камерой)', a.filter(r => r.stages.delivered).length, n]], v => v + ' / ' + n); }).join('') : '<div class="muted">попыток нет</div>');
    const t5 = e5.filter(r => r.kind === 'scenario'), S = ['command', 'requester', 'found_object', 'grasped', 'found_person', 'approached', 'hand_seen', 'reachable', 'pulled', 'handed_over'];
    $('#rs-e5').innerHTML = `<h3>Эксперимент 5: помощник (${t5.length} сценариев, подтверждённых передач ${t5.filter(r => r.ok).length})</h3>` + (t5.length ? bars('', S.map(k => [k, t5.filter(r => r.steps && r.steps[k]).length, t5.length]), v => v + ' / ' + t5.length) : '<div class="muted">сценариев нет</div>');
  };
  refresh(); setInterval(refresh, 8000);
});
