"""The operator dashboard page (HTML + canvas + JS) as one stdlib-served string.

Served by ``operator_ui`` at ``GET /``. Self-contained: no framework, no build
step, no CDN — it renders the shared world on an HTML5 canvas (a direct port of
the former Tkinter ``ui_map`` transforms/drawing), drives the side panel
(former ``ui_panels``), receives state over Server-Sent Events (with a
short-poll fallback) and issues commands via ``fetch`` POSTs that map 1:1 to the
ROS publishers. Works offline (lab Wi-Fi is robot-only).
"""

INDEX_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Algae Twin - Operator</title>
<style>
  :root{
    --bg:#10141a; --panel:#161b22; --fg:#d7dde4; --dim:#7d8590;
    --real:#4da3ff; --sim:#37e0c8; --algae:#3ddc84; --warn:#ffb454;
    --err:#ff5d5d; --plan:#e3c567; --edit:#ff5d5d; --mirrored:#ffb454;
  }
  *{box-sizing:border-box}
  html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);
    font:13px/1.4 system-ui,Segoe UI,Roboto,sans-serif;overflow:hidden}
  #app{display:flex;height:100%}
  #mapwrap{flex:1;position:relative;margin:8px 4px 8px 8px}
  #map{width:100%;height:100%;display:block;background:#10141a;
    border-radius:6px;cursor:crosshair}
  #statusbar{position:absolute;left:0;right:0;bottom:0;padding:4px 8px;
    background:rgba(22,27,34,.85);color:var(--dim);border-radius:0 0 6px 6px;
    font-size:12px;white-space:nowrap;overflow:hidden}
  #panel{width:310px;background:var(--panel);margin:8px 8px 8px 4px;
    border-radius:6px;padding:10px;overflow-y:auto}
  h2{font-size:11px;letter-spacing:.5px;color:var(--dim);margin:14px 0 4px;
    text-transform:uppercase}
  h2:first-child{margin-top:0}
  .row{display:flex;align-items:center;gap:6px;margin:4px 0}
  .dot{width:10px;height:10px;border-radius:50%;background:var(--dim);flex:none}
  .name{font-weight:700}
  .pose{margin-left:auto;color:var(--dim);font-variant-numeric:tabular-nums}
  .bar{height:8px;background:var(--bg);border-radius:4px;overflow:hidden;margin:2px 0}
  .bar>span{display:block;height:100%;background:var(--algae);width:0}
  .batt{color:var(--fg);font-variant-numeric:tabular-nums}
  ul{list-style:none;margin:2px 0;padding:0;max-height:120px;overflow-y:auto;
    background:var(--bg);border-radius:4px}
  li{padding:2px 8px;font-variant-numeric:tabular-nums;white-space:nowrap}
  label.tool{display:block;padding:3px 4px;cursor:pointer}
  label.tool input{margin-right:6px}
  button{display:block;width:100%;margin:3px 0;padding:7px;border:0;
    border-radius:4px;background:var(--bg);color:var(--fg);cursor:pointer;
    font:inherit}
  button:hover{background:#222b36}
  #estop{background:var(--err);color:#fff;font-weight:700;font-size:15px;padding:9px}
  #note{color:var(--fg)}
  #divergence{margin-top:4px}
</style>
</head>
<body>
<div id="app">
  <div id="mapwrap">
    <canvas id="map"></canvas>
    <div id="statusbar">connecting...</div>
  </div>
  <div id="panel">
    <h2>Twin status</h2>
    <div id="mode">mode: ...</div>
    <div class="row"><span class="dot" id="dot-real"></span>
      <span class="name" style="color:var(--real)">REAL ROBOT</span>
      <span class="pose" id="pose-real">offline</span></div>
    <div class="bar"><span id="bar-real"></span></div>
    <div class="batt" id="batt-real">battery: -</div>
    <div class="row"><span class="dot" id="dot-sim"></span>
      <span class="name" style="color:var(--sim)">DIGITAL TWIN</span>
      <span class="pose" id="pose-sim">offline</span></div>
    <div class="bar"><span id="bar-sim"></span></div>
    <div class="batt" id="batt-sim">battery: -</div>
    <div id="divergence">twin divergence: -</div>

    <h2>Mission</h2>
    <div id="note">idle</div>
    <ul id="algae-list"></ul>

    <h2>World edits</h2>
    <ul id="edits-list"></ul>

    <h2>Tools</h2>
    <div id="tools">
      <label class="tool"><input type="radio" name="tool" value="algae" checked>Place algae</label>
      <label class="tool"><input type="radio" name="tool" value="block">Block path</label>
      <label class="tool"><input type="radio" name="tool" value="erase">Erase</label>
      <label class="tool"><input type="radio" name="tool" value="goal">Nav goal</label>
      <label class="tool"><input type="radio" name="tool" value="setpose">Set robot pose</label>
    </div>

    <h2>Controls</h2>
    <button id="estop">E-STOP</button>
    <button data-cmd="home">Return home</button>
    <button data-cmd="recharge">Recharge twin battery</button>
    <button data-cmd="clear-mirrored">Clear mirrored obstacles</button>
    <button data-cmd="clear-all">Clear all edits</button>
  </div>
</div>

<script>
"use strict";
const C = getComputedStyle(document.documentElement);
const col = n => C.getPropertyValue('--'+n).trim();
const V_FULL=12.6, V_EMPTY=11.0, V_LOW=11.3;
const TOOL_HINTS = {
  algae:'click: drop an algae patch',
  block:'drag: draw a blocking box (applies to BOTH robots)',
  erase:'click: remove an edit box or algae patch',
  goal:'drag: position + heading for a nav goal',
  setpose:'drag: tell AMCL where the real robot actually is'
};

const cv = document.getElementById('map');
const ctx = cv.getContext('2d');
const statusbar = document.getElementById('statusbar');

let mapMeta = null;          // {w,h,res,ox,oy}
let mapVersion = -1;
let offscreen = null;        // native-resolution map bitmap
let mapCells = null;         // Uint8Array of occupancy (0 free, 100 wall, 255 unknown)
let zoom = 6;
let pan = [40, 40];
let state = {};              // latest dynamic snapshot
let trailReal = [], trailSim = [];
let sprayPhase = 0;
let drag = null;             // {x0,y0,sx,sy} during a left drag
let panFrom = null;          // right-drag pan anchor

function tool(){ return document.querySelector('input[name=tool]:checked').value; }

// ---- command helper --------------------------------------------------------
function cmd(name, body){
  fetch('/cmd/'+name, {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify(body||{})}).catch(()=>{});
}

// ---- coordinate transforms (ported from ui_map) ----------------------------
function w2s(x, y){
  const m = mapMeta;
  return [pan[0] + (x-m.ox)/m.res*zoom,
          pan[1] + (m.h - (y-m.oy)/m.res)*zoom];
}
function s2w(sx, sy){
  const m = mapMeta;
  return [(sx-pan[0])/zoom*m.res + m.ox,
          (m.h - (sy-pan[1])/zoom)*m.res + m.oy];
}

// ---- map bitmap ------------------------------------------------------------
function buildBitmap(meta, cells){
  const w=meta.w, h=meta.h;
  const off=document.createElement('canvas'); off.width=w; off.height=h;
  const octx=off.getContext('2d');
  const img=octx.createImageData(w,h);
  const free=col('free'), wall=col('wall');
  function rgb(hex){return [parseInt(hex.slice(1,3),16),parseInt(hex.slice(3,5),16),parseInt(hex.slice(5,7),16)];}
  const cFree=rgb(free), cWall=rgb(wall), cUnk=[16,20,26];
  for(let r=0;r<h;r++){
    const yimg=h-1-r;                      // grid row 0 = bottom -> image bottom
    for(let cx=0;cx<w;cx++){
      const b=cells[r*w+cx];
      let p; if(b===0)p=cFree; else if(b===100||(b<128&&b>=50))p=cWall; else p=cUnk;
      const o=(yimg*w+cx)*4;
      img.data[o]=p[0]; img.data[o+1]=p[1]; img.data[o+2]=p[2]; img.data[o+3]=255;
    }
  }
  octx.putImageData(img,0,0);
  return off;
}
function refreshMap(){
  fetch('/map.json').then(r=> r.status===200 ? r.json() : null).then(j=>{
    if(!j) return;
    mapMeta={w:j.w,h:j.h,res:j.res,ox:j.ox,oy:j.oy};
    const bin=atob(j.cells_b64); const cells=new Uint8Array(bin.length);
    for(let i=0;i<bin.length;i++) cells[i]=bin.charCodeAt(i);
    mapCells=cells;
    offscreen=buildBitmap(mapMeta, cells);
  }).catch(()=>{});
}
function cellFree(wx,wy){      // is (wx,wy) free navigable map space?
  if(!mapMeta||!mapCells) return true;     // no map yet -> let the server decide
  const c=Math.floor((wx-mapMeta.ox)/mapMeta.res);
  const r=Math.floor((wy-mapMeta.oy)/mapMeta.res);
  if(c<0||r<0||c>=mapMeta.w||r>=mapMeta.h) return false;   // off the map
  return mapCells[r*mapMeta.w+c] < 50;     // 0=free; 100=wall, 255=unknown -> no
}

// ---- rendering (ported from ui_map._draw_*) --------------------------------
function resize(){ cv.width=cv.clientWidth; cv.height=cv.clientHeight; }
window.addEventListener('resize', resize);

function poly(pts, stroke, fill, alpha){
  ctx.beginPath();
  pts.forEach((p,i)=> i? ctx.lineTo(p[0],p[1]) : ctx.moveTo(p[0],p[1]));
  ctx.closePath();
  if(fill){ ctx.globalAlpha=alpha||1; ctx.fillStyle=fill; ctx.fill(); ctx.globalAlpha=1; }
  if(stroke){ ctx.strokeStyle=stroke; ctx.lineWidth=2; ctx.stroke(); }
}
function drawPlan(plan){
  if(!plan||plan.length<2) return;
  ctx.save(); ctx.setLineDash([6,4]); ctx.strokeStyle=col('plan'); ctx.lineWidth=2;
  ctx.beginPath();
  plan.forEach((p,i)=>{const s=w2s(p[0],p[1]); i?ctx.lineTo(s[0],s[1]):ctx.moveTo(s[0],s[1]);});
  ctx.stroke(); ctx.restore();
}
function drawEdits(edits){
  edits.forEach(e=>{
    const c=(e.source==='mirrored')?col('mirrored'):col('edit');
    const cy=Math.cos(e.yaw), sy=Math.sin(e.yaw), pts=[];
    [[-1,-1],[1,-1],[1,1],[-1,1]].forEach(([dx,dy])=>{
      const bx=dx*e.size_x/2, by=dy*e.size_y/2;
      pts.push(w2s(e.cx+bx*cy-by*sy, e.cy+bx*sy+by*cy));
    });
    poly(pts, c, c, 0.35);
  });
}
function drawAlgae(algae){
  algae.forEach(a=>{
    const s=w2s(a.x,a.y); const r=0.15/mapMeta.res*zoom;
    let c=col('algae'); if(a.status==='cleared') c=col('dim')||'#7d8590';
    ctx.globalAlpha=0.55; ctx.fillStyle=c; ctx.beginPath();
    ctx.arc(s[0],s[1],r,0,6.283); ctx.fill(); ctx.globalAlpha=1;
    if(a.status==='active'){ ctx.strokeStyle=col('warn'); ctx.lineWidth=2;
      ctx.beginPath(); ctx.arc(s[0],s[1],r+3,0,6.283); ctx.stroke(); }
    else if(a.status==='cleaning'){ const pulse=4+3*Math.sin(sprayPhase/3);
      ctx.strokeStyle=col('algae'); ctx.lineWidth=2;
      ctx.beginPath(); ctx.arc(s[0],s[1],r+pulse,0,6.283); ctx.stroke(); }
    ctx.fillStyle='#06240f'; ctx.font='bold 9px sans-serif';
    ctx.textAlign='center'; ctx.textBaseline='middle'; ctx.fillText(a.id,s[0],s[1]);
  });
}
function drawScan(scan){
  if(!scan) return; ctx.fillStyle='#5a6470';
  scan.forEach(p=>{const s=w2s(p[0],p[1]); ctx.fillRect(s[0],s[1],2,2);});
}
function robotMarker(x,y,yaw,c,solid){
  const size=0.105, pts=[];
  [[0,1.6],[2.5,1.0],[-2.5,1.0]].forEach(([ang,sc])=>{
    pts.push(w2s(x+size*sc*Math.cos(yaw+ang), y+size*sc*Math.sin(yaw+ang)));});
  if(solid) poly(pts,'#ffffff',c,1); else poly(pts,c,null,1);
}
function drawSpray(x,y){
  ctx.fillStyle=col('algae');
  for(let i=0;i<10;i++){
    const ang=(sprayPhase*0.4+i*0.63)%6.283;
    const dist=0.12+0.10*((sprayPhase*7+i*37)%10)/10;
    const s=w2s(x+dist*Math.cos(ang), y+dist*Math.sin(ang));
    ctx.beginPath(); ctx.arc(s[0],s[1],2,0,6.283); ctx.fill();
  }
}
function drawRobots(status){
  sprayPhase++;
  const cleaning=status.clean_active;
  [['real',trailReal,col('real')],['sim',trailSim,col('sim')]].forEach(([k,trail,c])=>{
    const info=status[k]||{}, pose=info.pose; if(!pose) return;
    const [x,y,yaw]=pose;
    if(!trail.length || Math.hypot(trail[trail.length-1][0]-x,trail[trail.length-1][1]-y)>0.01){
      trail.push([x,y]); if(trail.length>600) trail.splice(0,trail.length-600);
    }
    if(trail.length>1){ ctx.strokeStyle=c; ctx.lineWidth=1; ctx.beginPath();
      trail.forEach((p,i)=>{const s=w2s(p[0],p[1]); i?ctx.lineTo(s[0],s[1]):ctx.moveTo(s[0],s[1]);});
      ctx.stroke(); }
    robotMarker(x,y,yaw,c,k==='real');
    if(cleaning) drawSpray(x,y);
  });
}
function render(){
  if(!mapMeta||!offscreen){          // no map yet: say so instead of a blank canvas
    if(cv.width!==cv.clientWidth||cv.height!==cv.clientHeight) resize();
    ctx.clearRect(0,0,cv.width,cv.height);
    ctx.fillStyle=col('dim'); ctx.font='14px sans-serif'; ctx.textAlign='center';
    ctx.fillText('waiting for /map ...', cv.width/2, cv.height/2);
    requestAnimationFrame(render); return;
  }
  try{
    if(cv.width!==cv.clientWidth||cv.height!==cv.clientHeight) resize();
    ctx.clearRect(0,0,cv.width,cv.height);
    ctx.imageSmoothingEnabled=false;
    ctx.drawImage(offscreen, pan[0], pan[1], mapMeta.w*zoom, mapMeta.h*zoom);
    const status=state.status||{};
    drawPlan(state.plan);
    drawEdits((state.edits&&state.edits.edits)||[]);
    drawAlgae((state.algae&&state.algae.algae)||[]);
    drawScan(state.scan);
    drawRobots(status);
    if(drag&&drag.rubber) drag.rubber();
  }catch(err){ /* one bad frame must never stop the loop */ console.error(err); }
  requestAnimationFrame(render);
}

// ---- side panel (ported from ui_panels.refresh) ----------------------------
let algaeVer=-1, editsVer=-1;
function setText(id,t){ document.getElementById(id).textContent=t; }
function fmt(n){ return (n>=0?'+':'')+n.toFixed(2); }
function updatePanel(){
  const status=state.status||{};
  setText('mode','mode: '+(status.mode||'...'));
  ['real','sim'].forEach(k=>{
    const info=status[k]||{}, online=!!info.ok, pose=info.pose, volt=info.voltage;
    document.getElementById('dot-'+k).style.background= online?col('algae'):col('err');
    setText('pose-'+k, pose? '('+fmt(pose[0])+', '+fmt(pose[1])+')' : 'offline');
    const frac = (volt==null)?0:Math.max(0,Math.min(1,(volt-V_EMPTY)/(V_FULL-V_EMPTY)));
    document.getElementById('bar-'+k).style.width=(frac*100)+'%';
    const b=document.getElementById('batt-'+k);
    b.textContent = (volt==null)?'battery: -':('battery: '+volt.toFixed(1)+' V');
    b.style.color = (volt!=null&&volt<V_LOW)?col('err'):col('fg');
  });
  const div=status.divergence, de=document.getElementById('divergence');
  if(div==null) { de.textContent='twin divergence: -'; de.style.color=col('fg'); }
  else { de.textContent='twin divergence: '+div.toFixed(2)+' m';
    de.style.color = div<0.10?col('algae'):(div<0.30?col('warn'):col('err')); }
  const eng=!!status.estop, eb=document.getElementById('estop');
  eb.textContent = eng?'RESUME':'E-STOP';
  eb.style.background = eng?col('warn'):col('err');
  setText('note', (state.algae&&state.algae.note)||'idle');
  const flags=[];
  if(status.estop) flags.push('E-STOP ENGAGED');
  if(status.safety) flags.push('SAFETY STOP - obstacle ahead');
  if(flags.length){ statusbar.textContent=flags.join('    '); statusbar.style.color=col('err'); }
  else { statusbar.textContent='link: '+(state.conn||'...')+'    tool: '+tool(); statusbar.style.color=col('dim'); }
  if(state._versions){
    if(state._versions.algae!==algaeVer){ algaeVer=state._versions.algae;
      list('algae-list', (state.algae&&state.algae.algae)||[],
        a=>' '+a.id+'  ('+fmt(a.x)+', '+fmt(a.y)+')  '+a.status); }
    if(state._versions.edits!==editsVer){ editsVer=state._versions.edits;
      list('edits-list', (state.edits&&state.edits.edits)||[],
        e=>' '+e.id+'  '+e.size_x.toFixed(2)+'x'+e.size_y.toFixed(2)+' m  '+e.source); }
  }
}
function list(id, items, fmtFn){
  const ul=document.getElementById(id); ul.innerHTML='';
  items.forEach(it=>{ const li=document.createElement('li'); li.textContent=fmtFn(it); ul.appendChild(li); });
}

// ---- input (ported from ui_map mouse handlers) -----------------------------
function rel(ev){ const r=cv.getBoundingClientRect(); return [ev.clientX-r.left, ev.clientY-r.top]; }
cv.addEventListener('contextmenu', e=>e.preventDefault());
cv.addEventListener('mousedown', ev=>{
  if(!mapMeta) return; const [x,y]=rel(ev);
  if(ev.button===2){ panFrom=[x-pan[0], y-pan[1]]; return; }
  if(ev.button!==0) return;
  drag={x0:x,y0:y,cx:x,cy:y};
});
cv.addEventListener('mousemove', ev=>{
  if(!mapMeta) return; const [x,y]=rel(ev);
  if(panFrom){ pan=[x-panFrom[0], y-panFrom[1]]; return; }
  if(drag){
    drag.cx=x; drag.cy=y; const t=tool();
    drag.rubber = ()=>{
      ctx.save();
      if(t==='block'){ ctx.strokeStyle=col('edit'); ctx.lineWidth=2;
        ctx.strokeRect(drag.x0,drag.y0,drag.cx-drag.x0,drag.cy-drag.y0); }
      else if(t==='goal'||t==='setpose'){ ctx.strokeStyle=col('plan'); ctx.lineWidth=2;
        ctx.beginPath(); ctx.moveTo(drag.x0,drag.y0); ctx.lineTo(drag.cx,drag.cy); ctx.stroke();
        const a=Math.atan2(drag.cy-drag.y0,drag.cx-drag.x0);
        ctx.beginPath(); ctx.moveTo(drag.cx,drag.cy);
        ctx.lineTo(drag.cx-10*Math.cos(a-0.4),drag.cy-10*Math.sin(a-0.4));
        ctx.moveTo(drag.cx,drag.cy);
        ctx.lineTo(drag.cx-10*Math.cos(a+0.4),drag.cy-10*Math.sin(a+0.4)); ctx.stroke(); }
      ctx.restore();
    };
  }
  const [wx,wy]=s2w(x,y);
  statusbar.textContent='('+fmt(wx)+', '+fmt(wy)+') m    '+(TOOL_HINTS[tool()]||'')+
    '    '+((state.status&&state.status.estop)?'E-STOP ENGAGED':'');
});
window.addEventListener('mouseup', ev=>{
  if(panFrom){ panFrom=null; return; }
  if(!drag||!mapMeta){ drag=null; return; }
  const t=tool(); const [w0x,w0y]=s2w(drag.x0,drag.y0); const [w1x,w1y]=s2w(drag.cx,drag.cy);
  if(t==='algae'){
    if(cellFree(w1x,w1y)){ cmd('algae_add',{x:w1x,y:w1y}); }
    else { statusbar.textContent='cannot place algae there - not free map space';
           statusbar.style.color=col('err'); }
  }
  else if(t==='block'){ const sx=Math.max(Math.abs(w1x-w0x),0.10), sy=Math.max(Math.abs(w1y-w0y),0.10);
    cmd('edit_add',{cx:(w0x+w1x)/2, cy:(w0y+w1y)/2, size_x:sx, size_y:sy}); }
  else if(t==='erase') eraseAt(w1x,w1y);
  else if(t==='goal'||t==='setpose'){
    const yaw=(Math.hypot(w1x-w0x,w1y-w0y)>0.05)? Math.atan2(w1y-w0y,w1x-w0x) : 0.0;
    cmd(t==='goal'?'goto':'setpose',{x:w0x,y:w0y,yaw:yaw}); }
  drag=null;
});
function eraseAt(x,y){
  const edits=(state.edits&&state.edits.edits)||[];
  for(const e of edits){
    const dx=x-e.cx, dy=y-e.cy, cy=Math.cos(-e.yaw), sy=Math.sin(-e.yaw);
    const bx=dx*cy-dy*sy, by=dx*sy+dy*cy;
    if(Math.abs(bx)<=e.size_x/2 && Math.abs(by)<=e.size_y/2){ cmd('edit_remove',{id:e.id}); return; }
  }
  const algae=(state.algae&&state.algae.algae)||[];
  for(const a of algae){ if(Math.hypot(x-a.x,y-a.y)<0.18){ cmd('algae_remove',{id:a.id}); return; } }
}
cv.addEventListener('wheel', ev=>{
  ev.preventDefault(); if(!mapMeta) return; const [x,y]=rel(ev);
  const step = ev.deltaY<0?1:-1; const nz=Math.max(3,Math.min(14,zoom+step));
  if(nz===zoom) return; const [wx,wy]=s2w(x,y); zoom=nz;
  pan[0]=x-(wx-mapMeta.ox)/mapMeta.res*zoom;
  pan[1]=y-(mapMeta.h-(wy-mapMeta.oy)/mapMeta.res)*zoom;
},{passive:false});

// ---- controls --------------------------------------------------------------
document.getElementById('estop').addEventListener('click',()=>{
  cmd('estop',{engaged: !(state.status&&state.status.estop)});
});
document.querySelectorAll('button[data-cmd]').forEach(b=> b.addEventListener('click',()=>{
  const k=b.dataset.cmd;
  if(k==='home') cmd('home');
  else if(k==='recharge') cmd('recharge',{percent:100.0});
  else if(k==='clear-mirrored') cmd('clear_edits',{source:'mirrored'});
  else if(k==='clear-all') cmd('clear_edits',{source:'all'});
}));
window.addEventListener('keydown', ev=>{        // Space toggles, Esc = hard stop
  if(ev.code==='Space'){ ev.preventDefault();
    cmd('estop',{engaged: !(state.status&&state.status.estop)}); }
  else if(ev.code==='Escape'){ ev.preventDefault(); cmd('estop',{engaged:true}); }
});

// ---- state transport: SSE primary, short-poll fallback ---------------------
function applyState(obj){
  state=obj;
  if(state.map_version!=null && state.map_version!==mapVersion){ mapVersion=state.map_version; refreshMap(); }
  try{ updatePanel(); }catch(err){ console.error(err); }
}
let pollTimer=null;
function startPolling(){
  if(pollTimer) return;
  pollTimer=setInterval(()=>{ fetch('/state.json').then(r=>r.json()).then(applyState).catch(()=>{}); }, 100);
}
function startSSE(){
  let fails=0;
  const es=new EventSource('/events');
  es.onmessage=e=>{ fails=0; try{ applyState(JSON.parse(e.data)); }catch(err){} };
  es.onerror=()=>{ if(++fails>=3){ es.close(); startPolling(); } };
}

resize(); refreshMap(); render();
if(window.EventSource) startSSE(); else startPolling();
</script>
</body>
</html>
"""
