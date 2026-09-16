'use strict';
const $ = id => document.getElementById(id);
const canvas = $('scene'), ctx = canvas.getContext('2d');
let run = null, time = 0, playing = true, last = 0, yaw = -0.65, pitch = 0.72, zoom = 1;
let width = 800, height = 500, request = 0;
const explanations = {
  inspection: 'Climb over a low wall, clear the structures, and settle at the inspection endpoint.',
  blocked: 'A wall spans the flight volume. The planner must return no route and hold at the start.',
  stale: 'Evidence updates freeze after 3 seconds. The vehicle brakes when age exceeds 600 ms.',
  link: 'The command link fails after 3 seconds. The vehicle brakes on its validated segment.',
  unknown: 'An unobserved volume spans the corridor. Unknown space is excluded from the route.'
};
const outcomes = {
  READY: 'Route validated. Replay uses Rust positions and velocities.',
  FLYING: 'Following the validated route. Stops at corners preserve bounded acceleration.',
  BRAKING: 'Safety intervention: braking within the validated stopping reserve.',
  ARRIVED: 'Inspection complete. Vehicle settled at the endpoint.',
  HOLD_NO_ROUTE: 'No admissible route. Vehicle holds at the start; no direct route fallback.',
  HOLD_STALE_SENSOR: 'Safe hold. Sensor evidence expired and the vehicle completed braking.',
  HOLD_LINK_LOSS: 'Safe hold. Command link lost and the vehicle completed braking.'
};
function describe() { $('description').textContent = explanations[$('scenario').value]; }
function metric(id, value, unit) {
  const node=$(id);node.replaceChildren(document.createTextNode(value+' '));
  const small=document.createElement('small');small.textContent=unit;node.append(small);
}
async function get(url) {
  const response=await fetch(url,{signal:AbortSignal.timeout(15000)});
  if(!response.ok) throw new Error('Rust engine returned '+response.status);
  return response.json();
}
async function generate() {
  const seed=Number($('seed').value);
  if(!Number.isInteger(seed)||seed<0||seed>4294967295){$('banner').textContent='Use a whole seed from 0 to 4294967295.';return;}
  const id=++request;playing=false;$('run').disabled=true;$('connection').textContent='PLANNING';
  $('banner').textContent='Computing a bounded 3D route in Rust…';
  try {
    const next=await get('/api/run?scenario='+$('scenario').value+'&seed='+seed);
    if(id!==request)return;run=next;time=0;playing=run.frames.length>1;describe();
    $('scene-title').textContent=$('scenario').selectedOptions[0].textContent;
    $('connection').textContent='CONNECTED';$('latency').textContent=(run.planning_us/1000).toFixed(2)+' ms';
    $('expanded').textContent=run.expanded.toLocaleString();$('samples').textContent=run.frames.length.toLocaleString();
    $('endpoint').textContent=run.outcome==='ARRIVED'?'Arrival':'Safety hold';
    update();
  } catch(e) {$('connection').textContent='OFFLINE';$('banner').className='banner error';$('banner').textContent=e.message;playing=false;}
  finally {if(id===request)$('run').disabled=false;}
}
async function evidence() {
  try {
    const e=await get('/api/evaluate');$('passed').textContent=e.passed+' / '+e.runs;
    $('violations').textContent=e.collisions+' / '+e.geofence_violations;
    $('p95').textContent=(e.planning_p95_us/1000).toFixed(2)+' ms';$('dynamics').textContent=e.dynamics_violations;
    $('suite-status').textContent=e.passed===e.runs?'● ALL EXPECTED OUTCOMES':'CHECK FAILED';
  } catch(e) {$('suite-status').textContent='EVIDENCE UNAVAILABLE';}
}
function current() {
  let low=0,high=run.frames.length-1;
  while(low<high){const mid=Math.ceil((low+high)/2);if(run.frames[mid].t<=time)low=mid;else high=mid-1;}
  return {f:run.frames[low],index:low};
}
function update() {
  if(!run)return;const {f}=current();const speed=Math.hypot(...f.v),end=run.frames.at(-1).t;
  $('status').textContent=f.status.replaceAll('_',' ');metric('speed',speed.toFixed(2),'m/s');
  metric('altitude',f.p[2].toFixed(2),'m');metric('clearance',f.clearance.toFixed(2),'m');metric('age',Math.round(f.age*1000),'ms');
  $('speed-meter').value=speed;$('time').textContent=time.toFixed(1)+' / '+end.toFixed(1)+' s';
  $('timeline').value=end?time/end*1000:0;$('play').textContent=playing?'Pause':'Play';
  $('play').setAttribute('aria-label',playing?'Pause replay':'Play replay');
  $('banner').textContent=outcomes[f.status]||f.status;
  $('banner').className='banner'+(f.status==='BRAKING'||f.status.startsWith('HOLD')?' warning':'');
}
function resize() {
  const box=canvas.getBoundingClientRect();width=box.width;height=box.height;
  const ratio=Math.min(window.devicePixelRatio||1,1.5);canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);ctx.setTransform(ratio,0,0,ratio,0,0);
}
new ResizeObserver(resize).observe(canvas);
function project(p) {
  const x=p[0]-16,y=p[1]-12,z=p[2]-3;
  const rx=x*Math.cos(yaw)-y*Math.sin(yaw),ry=x*Math.sin(yaw)+y*Math.cos(yaw);
  const scale=Math.min(width/47,height/36)*zoom;
  return [width/2+rx*scale,height/2+18+(ry*Math.sin(pitch)-z*Math.cos(pitch))*scale,ry*Math.cos(pitch)+z*Math.sin(pitch)];
}
function line(points,color,lineWidth=1,dash=[]) {
  if(!points.length)return;ctx.beginPath();points.forEach((p,i)=>{const q=project(p);if(i)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1]);});
  ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);
}
function polygon(points,fill,stroke) {
  ctx.beginPath();points.forEach((p,i)=>{const q=project(p);if(i)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1]);});ctx.closePath();ctx.fillStyle=fill;ctx.fill();
  if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=.7;ctx.stroke();}
}
function dot(p,r,color) {const q=project(p);ctx.beginPath();ctx.arc(q[0],q[1],r,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();}
function label(p,text,color='#9ab1be') {const q=project(p);ctx.fillStyle=color;ctx.font='9px monospace';ctx.fillText(text,q[0]+8,q[1]-10);}
function draw() {
  ctx.clearRect(0,0,width,height);if(!run)return;
  polygon([[0,0,0],[32,0,0],[32,24,0],[0,24,0]],'#14232c','#34505d');
  for(let x=0;x<=32;x+=2)line([[x,0,0],[x,24,0]],'#223842',.6);
  for(let y=0;y<=24;y+=2)line([[0,y,0],[32,y,0]],'#223842',.6);
  for(const [x,y]of [[0,0],[32,0],[32,24],[0,24]])line([[x,y,0],[x,y,12]],'#36515e',.6,[3,5]);
  line([[0,0,12],[32,0,12],[32,24,12],[0,24,12],[0,0,12]],'#36515e',.6,[3,5]);
  label([32,0,0],'32 m');label([32,24,12],'12 m');
  if(run.route.length)line(run.route.map(p=>[p[0],p[1],.03]),'#466253',1,[3,4]);
  const faces=[];
  for(const o of run.obstacles){
    const [x,y,z]=o.min,[X,Y,Z]=o.max;
    const verts=[[x,y,z],[X,y,z],[X,Y,z],[x,Y,z],[x,y,Z],[X,y,Z],[X,Y,Z],[x,Y,Z]];
    const sides=[[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[4,5,6,7]];
    sides.forEach((idx,i)=>{const p=idx.map(n=>verts[n]);faces.push({p,depth:p.reduce((s,q)=>s+project(q)[2],0)/4,color:o.unknown?['#43374a','#493d50','#382e40','#3d3445','#62536c'][i]:['#243b48','#2a4452','#203744','#284351','#395867'][i],stroke:o.unknown?'#a38ca9':'#4c7080'});});
  }
  faces.sort((a,b)=>a.depth-b.depth);for(const f of faces)polygon(f.p,f.color,f.stroke);
  // X-ray route overlay deliberately keeps the planned path visible through geometry.
  line(run.route,'#76e4b9',2,[5,3]);for(const p of run.route)dot(p,2.5,'#a2f2d4');
  dot(run.world?.start||run.start,4,'#76e4b9');label(run.start,'START','#a2f2d4');dot(run.goal,4,'#c1d0ee');label(run.goal,'GOAL','#c1d0ee');
  const {f,index}=current();const trail=run.frames.slice(Math.max(0,index-150),index+1).map(x=>x.p);line(trail,'#ffd58a',2.5);
  const p=f.p;line([[p[0],p[1],0],p],'#887657',1,[2,3]);
  const q=project(p);const pulse=playing?Math.sin(performance.now()/350)*2:0;
  ctx.beginPath();ctx.arc(q[0],q[1],15+pulse,0,Math.PI*2);ctx.strokeStyle='#ffd58a50';ctx.lineWidth=1;ctx.stroke();
  const arms=[[-.6,-.6],[.6,.6],[-.6,.6],[.6,-.6]];
  for(const [dx,dy]of arms){const end=[p[0]+dx,p[1]+dy,p[2]];line([p,end],'#ffe5b0',2);const e=project(end);ctx.beginPath();ctx.ellipse(e[0],e[1],5,3,0,0,2*Math.PI);ctx.strokeStyle='#ffd58a';ctx.lineWidth=1.2;ctx.stroke();}
  dot(p,4,'#ffe5b0');label([p[0]+.5,p[1],p[2]+1.2],'UAV 01','#ffe5b0');
}
function animate(stamp) {
  requestAnimationFrame(animate);if(stamp-last<33)return;const delta=Math.min((stamp-last)/1000,.1);last=stamp;
  if(run&&playing&&!document.hidden){time=Math.min(run.frames.at(-1).t,time+delta*Number($('rate').value));if(time>=run.frames.at(-1).t)playing=false;update();}
  if(!document.hidden)draw();
}
let drag=null;
canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY,yaw,pitch];canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!drag)return;yaw=drag[2]+(e.clientX-drag[0])*.008;pitch=Math.max(.2,Math.min(1.55,drag[3]+(e.clientY-drag[1])*.005));});
canvas.addEventListener('pointerup',()=>drag=null);canvas.addEventListener('pointercancel',()=>drag=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.6,Math.min(1.8,zoom*Math.exp(-e.deltaY*.001)));},{passive:false});
$('orbit').onclick=()=>{yaw=-.65;pitch=.72;zoom=1;$('orbit').className='selected';$('top').className='';};
$('top').onclick=()=>{yaw=0;pitch=1.55;zoom=1;$('top').className='selected';$('orbit').className='';};
$('scenario').onchange=describe;$('run').onclick=generate;
$('shuffle').onclick=()=>{$('seed').value=(Number($('seed').value)+1)>>>0;generate();};
$('play').onclick=()=>{if(!run)return;if(time>=run.frames.at(-1).t)time=0;playing=!playing;update();};
$('restart').onclick=()=>{time=0;playing=true;update();};
$('timeline').oninput=()=>{if(!run)return;playing=false;time=Number($('timeline').value)/1000*run.frames.at(-1).t;update();};
$('export').onclick=()=>{if(!run)return;const blob=new Blob([JSON.stringify(run,null,2)],{type:'application/json'});const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download='ruv-navigation-'+run.scenario+'-'+run.seed+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
describe();generate();evidence();requestAnimationFrame(animate);
