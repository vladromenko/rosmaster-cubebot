INDEX_HTML = r"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rosmaster</title>
<style>
:root{--bg:#0e1116;--panel:#171c24;--line:#262d38;--text:#e6e9ee;--dim:#8a94a3;--ok:#3ecf8e;--warn:#f5b942;--bad:#ff5d5d;--acc:#5aa2ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 -apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;gap:14px;align-items:center;padding:10px 16px;border-bottom:1px solid var(--line);background:var(--panel)}
h1{font-size:16px;margin:0;font-weight:600;letter-spacing:.3px}.pill{padding:3px 10px;border-radius:999px;background:#222a36;font-size:12px}
.grow{flex:1}.banner{background:var(--warn);color:#111;text-align:center;padding:4px;font-weight:600;display:none}
main{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:12px;padding:12px;max-width:1500px;margin:0 auto}
@media(max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{margin:0 0 8px;font-size:12px;color:var(--dim);text-transform:uppercase;letter-spacing:.8px;font-weight:600}
#video{width:100%;aspect-ratio:4/3;background:#000;border-radius:8px;display:block}
.row{display:flex;gap:8px;flex-wrap:wrap}button{background:#222a36;color:var(--text);border:1px solid var(--line);padding:8px 12px;border-radius:8px;cursor:pointer;font-size:13px}
button:hover{border-color:var(--acc)}button.stop{background:var(--bad);border-color:var(--bad);color:#fff;font-weight:700;flex:1}
button.go{background:var(--ok);border-color:var(--ok);color:#06210f;font-weight:700}
.bar{height:10px;background:#222a36;border-radius:6px;overflow:hidden}.bar>i{display:block;height:100%;background:var(--ok);width:0}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-variant-numeric:tabular-nums}.kv span:nth-child(odd){color:var(--dim)}
kbd{background:#222a36;border:1px solid var(--line);border-bottom-width:2px;border-radius:4px;padding:0 5px;font-size:12px}
#log{height:150px;overflow:auto;font:12px/1.35 ui-monospace,Menlo,monospace;color:#b8c0cc;white-space:pre-wrap}
.help{font-size:12.5px;color:var(--dim)}.help b{color:var(--text)}
</style></head><body>
<div class="banner" id="dry">DRY-RUN: движение не отправляется</div>
<header><h1>Rosmaster</h1><span class="pill" id="mode">manual</span><span class="pill" id="link">нет связи</span>
<span class="grow"></span><span id="bat" class="pill">батарея …</span></header>
<nav id="tabs" style="display:flex;gap:8px;padding:6px 16px"><button id="tabc" class="go" onclick="tab(0)">Управление</button><button id="tabt" onclick="tab(1)">Учебник: рука и одометрия (живой)</button><button id="tabe" onclick="tab(2)">Эксперименты</button><button id="tabd" onclick="tab(3)">Как работают эксперименты</button></nav>
<iframe id="tut" src="about:blank" style="display:none;width:100%;height:calc(100vh - 110px);border:0;background:#fff"></iframe>
<main>
<section>
  <div class="card"><img id="video" src="/video" alt="камера"></div>
  <div class="card" style="margin-top:12px"><h2>Управление</h2>
   <div class="help">
    <p><b>Езда:</b> <kbd>W</kbd><kbd>S</kbd> вперёд/назад · <kbd>A</kbd><kbd>D</kbd> влево/вправо · <kbd>Q</kbd><kbd>E</kbd> поворот. <b>Shift</b> — медленно. Максимум вперёд 0,30 м/с; вбок 0,25 м/с; поворот 0,80 рад/с.</p>
    <div class="row"><button onclick="armMode('cartesian')">1 · Захват XYZ</button><button onclick="armMode('joint')">2 · Суставы</button><b id="armmode">Захват XYZ</b></div><p id="armhelp"></p><p><b>Рука:</b> <kbd>I</kbd><kbd>K</kbd> вперёд/назад · <kbd>J</kbd><kbd>L</kbd> влево/вправо (также стрелки) · <kbd>R</kbd><kbd>F</kbd> вверх/вниз · <kbd>,</kbd><kbd>.</kbd> наклон · <kbd>[</kbd><kbd>]</kbd> поворот кисти · <kbd>N</kbd>/<kbd>M</kbd> открыть/закрыть захват.</p>
    <p><b>Стоп:</b> <kbd>Пробел</kbd> · отмена миссии <kbd>Esc</kbd>.</p>
    <p><b>Джойстик:</b> левый стик — езда, LB/RB — поворот, правый стик — рука X/Y, крестовина ↑↓ — высота, ←→ — наклон, LT/RT — захват, A — поза обзора, X — переноска, Y — запуск миссии, B — стоп.</p>
   </div>
   <div class="row" style="margin:10px 0 0;gap:6px;align-items:center">
    <div style="display:grid;grid-template-columns:repeat(3,56px);gap:6px">
     <button class="hold" data-k="q">⟲</button><button class="hold" data-k="w">▲</button><button class="hold" data-k="e">⟳</button>
     <button class="hold" data-k="a">◀</button><button class="hold" data-k="s">▼</button><button class="hold" data-k="d">▶</button></div>
    <span style="color:var(--mut);font-size:12px">Колёса: удерживайте кнопку (мышь или палец). Отпустили — стоп. Те же клавиши W A S D Q E.</span>
    <label style="font-size:13px"><input type="checkbox" id="slowc"> медленно</label> <label style="font-size:13px"><input type="checkbox" id="freec"> без защиты от препятствий</label> <b id="gmsg" style="color:var(--warn);font-size:13px"></b></div>
  </div>
</section>
<aside style="display:grid;gap:12px;align-content:start">
  <div class="card"><h2>Заряд</h2><div class="bar"><i id="socbar"></i></div>
   <div class="kv" style="margin-top:8px"><span>Заряд</span><span id="soc">—</span><span>Напряжение</span><span id="volts">—</span><span>Осталось</span><span id="left">—</span></div>
   <div class="help" id="batnote" style="margin-top:6px"></div></div>
  <div class="card"><h2>Рука</h2><div class="kv"><span>Углы</span><span id="arm">—</span><span>Кисть x,y,z</span><span id="tcp">—</span><span>Ограничение</span><span id="blocked">—</span></div>
   <div class="row" style="margin-top:8px"><button onclick="act('preset','look')">Обзор</button><button onclick="act('preset','near')">Ближний</button><button onclick="act('preset','carry')">Переноска</button><button onclick="act('preset','home')">Домой</button></div></div>
  <div class="card"><h2>Зеркало: рука SO101 → рука Rosmaster</h2>
   <div class="help">«Включить» сам подключит Leader с Homebrain (LeRobot) — держите Leader неподвижно: рука сначала встанет в его позу (35°/с), затем повторяет движения. Текущая поза руки и захвата принимается за начало; после потери связи включите зеркало заново. Начинайте с малых движений.</div>
   <div class="kv" style="margin-top:6px"><span>Состояние</span><span id="mir">выкл</span><span>Кадров</span><span id="mirn">0</span><span>Цель</span><span id="mirt">—</span><span>Измерено</span><span id="mirm">—</span></div>
   <div class="row" style="margin-top:8px"><button class="go" onclick="fetch('/api/mirror_on',{method:'POST'})">Включить</button><button onclick="fetch('/api/mirror_off',{method:'POST'})">Выключить</button></div></div>
  <div class="card"><h2>Миссия: кубик → коробка → обратно</h2>
   <div class="kv"><span>Состояние</span><span id="mstate">idle</span><span>Итог</span><span id="mres">—</span></div>
   <div class="row" style="margin-top:8px"><button class="go" onclick="send({action:'mission_start'})">Запустить</button><button onclick="send({action:'mission_resume'})">Продолжить с кубом в клешне</button><button onclick="send({action:'mission_stop'})">Остановить</button></div></div>
  <div class="card"><div class="row"><button class="stop" onclick="send({action:'stop'})">STOP</button></div></div>
  <div class="card"><h2>Журнал</h2><div id="log"></div></div>
</aside></main>
<script>
const $=id=>document.getElementById(id);let ws,keys={},lockedKeys=new Set(),padBlocked=true,viewActive=true;
function clearControls(){for(const k in keys)lockedKeys.add(k);keys={};padBlocked=true;}
function send(o){if(o.action==='stop')clearControls();if(ws&&ws.readyState===1)ws.send(JSON.stringify(o));}
function armMode(mode){clearControls();send({action:'arm_mode',mode});}
function act(a,n){send({action:a,name:n});}
function connect(){ws=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws');
 ws.onopen=()=>{$('link').textContent='связь есть';$('link').style.background='#1f3a2c'};
 ws.onclose=()=>{clearControls();$('link').textContent='нет связи';$('link').style.background='#3a1f1f';setTimeout(connect,1000)};
 ws.onmessage=e=>render(JSON.parse(e.data));}
function render(s){
 $('armmode').textContent=s.arm_mode==='joint'?'Суставы':'Захват XYZ';$('armhelp').textContent=s.arm_mode==='joint'?'←/→ (J/L) — основание; ↑/↓ (I/K) — плечо; R/F — локоть; ,/. — кисть.':'↑/↓ (I/K) — вперёд/назад; ←/→ (J/L) — влево/вправо; R/F — вверх/вниз.';
 $('dry').style.display=s.dry_run?'block':'none';$('mode').textContent=s.mode==='mission'?'миссия':'ручной';
 const b=s.battery;if(b.soc!=null){$('socbar').style.width=b.soc+'%';$('socbar').style.background=b.soc<20?'var(--bad)':b.soc<40?'var(--warn)':'var(--ok)';
  $('soc').textContent=b.soc+' % (±'+b.error_band_percent+')'+(b.charging?' · заряжается':'');$('bat').textContent='🔋 '+Math.round(b.soc)+' %  '+(b.volts?b.volts.toFixed(2)+' В':'')}
 $('volts').textContent=b.volts?b.volts.toFixed(2)+' В':'—';$('left').textContent=b.minutes_left!=null?'≈ '+b.minutes_left+' мин':'—';
 $('batnote').textContent=s.battery_stale?'Показание напряжения не обновляется (особенность прошивки при движении)':'';
 $('arm').textContent=s.arm.join(', ');$('tcp').textContent=s.tcp.join(', ');$('blocked').textContent=s.jog_blocked||'—';
 const mi=s.mirror;$('mir').textContent=(mi.on?'включено':'выкл')+(mi.blocked?' · '+mi.blocked:'')+(mi.age!=null?' · данные '+mi.age+' с назад':'');$('mirn').textContent=mi.frames;$('mirt').textContent=mi.target.join(', ');$('mirm').textContent=s.arm_measured?s.arm_measured.join(', '):'нет обратной связи';
 $('gmsg').textContent=s.guard?(s.guard[0]==='translate'?'⚠ препятствие впереди/сбоку — езда ограничена':'⚠ препятствие мешает повороту'):'';
 $('mstate').textContent=s.mission;$('mres').textContent=s.mission_result||'—';
 const L=$('log');const atEnd=L.scrollTop+L.clientHeight>=L.scrollHeight-8;L.textContent=s.log.join('\n');if(atEnd)L.scrollTop=L.scrollHeight;}
const map={w:['d',0,1],s:['d',0,-1],a:['d',1,1],d:['d',1,-1],q:['d',2,1],e:['d',2,-1],i:['a',0,1],k:['a',0,-1],j:['a',1,1],l:['a',1,-1],r:['a',2,1],f:['a',2,-1]};
// Only text entry swallows keys; a focused checkbox/button must not (Safari keeps focus on the
// last clicked checkbox, which silently disabled all driving/arm keys).
function typing(t){return t&&(t.tagName==='TEXTAREA'||t.isContentEditable||(t.tagName==='INPUT'&&!['checkbox','radio','button','submit','range'].includes((t.type||'').toLowerCase())))}
document.addEventListener('change',e=>{if(e.target&&e.target.type==='checkbox')e.target.blur()});
document.addEventListener('click',e=>{const t=e.target.closest&&e.target.closest('button');if(t)t.blur()});
function physical(e){const c=e.code;return {ArrowUp:'i',ArrowDown:'k',ArrowLeft:'j',ArrowRight:'l',Comma:',',Period:'.',BracketLeft:'[',BracketRight:']',ShiftLeft:'shift',ShiftRight:'shift'}[c]||(c&&c.startsWith('Key')?c.slice(3).toLowerCase():c);}
addEventListener('keydown',e=>{const k=physical(e);
 if(k==='Space'||k==='Escape'){e.preventDefault();send({action:'stop'});return}
 if(typing(e.target))return;
 if(k==='Digit1'||k==='Digit2'){if(!e.repeat)armMode(k==='Digit1'?'cartesian':'joint');return}
 if(!document.hidden&&viewActive&&!lockedKeys.has(k)&&ws&&ws.readyState===1)keys[k]=true;
 if(k in map||[',','.','[',']','n','m'].includes(k))e.preventDefault();});
addEventListener('keyup',e=>{const k=physical(e);delete keys[k];lockedKeys.delete(k)});
addEventListener('blur',()=>send({action:'stop'}));
document.addEventListener('visibilitychange',()=>{if(document.hidden)send({action:'stop'})});
function pad(){const g=(navigator.getGamepads?navigator.getGamepads():[]);for(const p of g){if(p&&p.mapping==='standard')return p}return null}
let prevBtn={};
function collect(){if(document.hidden||!viewActive||!ws||ws.readyState!==1)return {drive:[0,0,0],arm:[0,0,0],pitch:0,roll:0,grip:0,slow:false};const d=[0,0,0],a=[0,0,0];let pitch=0,roll=0,grip=0;
 for(const k in keys){const m=map[k];if(m){(m[0]==='d'?d:a)[m[1]]+=m[2]}}
 if(keys[','])pitch-=1;if(keys['.'])pitch+=1;if(keys['['])roll-=1;if(keys[']'])roll+=1;if(keys['n'])grip-=1;if(keys['m'])grip+=1;
 const p=pad();if(p&&padBlocked){if(p.axes.every(v=>Math.abs(v)<.12)&&p.buttons.every(b=>!b.pressed&&b.value<.12))padBlocked=false;}if(p&&!padBlocked){const z=v=>Math.abs(v)<0.12?0:v;d[0]+=-z(p.axes[1]);d[1]+=-z(p.axes[0]);if(p.buttons[4].pressed)d[2]+=1;if(p.buttons[5].pressed)d[2]-=1;
  a[1]+=-z(p.axes[2]);a[0]+=-z(p.axes[3]);if(p.buttons[12].pressed)a[2]+=1;if(p.buttons[13].pressed)a[2]-=1;if(p.buttons[14].pressed)pitch-=1;if(p.buttons[15].pressed)pitch+=1;
  grip+=(p.buttons[7].value||0)-(p.buttons[6].value||0);
  const edge=(i,fn)=>{const on=p.buttons[i].pressed;if(on&&!prevBtn[i])fn();prevBtn[i]=on};
  edge(0,()=>act('preset','look'));edge(2,()=>act('preset','carry'));edge(3,()=>send({action:'mission_start'}));edge(1,()=>send({action:'stop'}));if(padBlocked)return {drive:[0,0,0],arm:[0,0,0],pitch:0,roll:0,grip:0,slow:false};}
 const c=v=>Math.max(-1,Math.min(1,v));return{drive:d.map(c),arm:a.map(c),pitch:c(pitch),roll:c(roll),grip:c(grip),free:!!($('freec')&&$('freec').checked),slow:!!(keys['shift'])||($('slowc')&&$('slowc').checked)}}
document.querySelectorAll('button.hold').forEach(b=>{const k=b.dataset.k,on=e=>{e.preventDefault();if(!lockedKeys.has(k))keys[k]=true},off=()=>{delete keys[k];lockedKeys.delete(k)};
 b.addEventListener('pointerdown',on);['pointerup','pointerleave','pointercancel'].forEach(t=>b.addEventListener(t,off))});
const TABS={1:['tabt','/tutorial'],2:['tabe','/experiments'],3:['tabd','/exp-docs']};
function tab(n){viewActive=!n;send({action:'stop'});document.querySelector('main').style.display=n?'none':'';const f=$('tut');f.style.display=n?'block':'none';
 if(n){const u=TABS[n][1];if(f.getAttribute('data-u')!==u){f.setAttribute('data-u',u);f.src=u}}
 $('tabc').className=n?'':'go';for(const k in TABS)$(TABS[k][0]).className=(+k===n)?'go':'';if(n)keys={}}
setInterval(()=>send(collect()),33);connect();
</script></body></html>
"""
