const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];
let statusData={},checkpoints=[],tasks=[],c12Presets=[],mapData={available:false},cloudData={available:false},workspace,currentTask=null,view={scale:24,ox:0,oy:0},cloudViewer;
let teleopArmed=false,teleopKey=null,teleopTimer=null;
function toast(text){const e=$('#toast');e.textContent=text;e.classList.add('show');setTimeout(()=>e.classList.remove('show'),2400)}
async function api(path,data){const options=data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)};const r=await fetch(path,options),j=await r.json();if(!r.ok)throw Error(j.error||'操作失败');return j}
function clock(){ $('#clock').textContent=new Date().toLocaleString('zh-CN',{hour12:false}) } clock();setInterval(clock,1000);
$$('.tab').forEach(b=>b.onclick=()=>{$$('.tab,.page').forEach(x=>x.classList.remove('active'));b.classList.add('active');$('#'+b.dataset.page).classList.add('active');if(b.dataset.page==='tasks')setTimeout(()=>Blockly.svgResize(workspace),30);drawMaps()});

async function refresh(){try{statusData=await api('/api/status');const r=statusData.robot,c=statusData.c12;$('#robotDot').className='dot '+(r.online?'ok':'warn');$('#robotStatus').textContent=r.online?'机器狗 ROS 在线':'机器狗等待接入';$('#cameraDot').className='dot '+(c.visible&&c.thermal?'ok':'warn');$('#cameraStatus').textContent=c.visible&&c.thermal?'C12 双光在线':'C12 等待连接';$('#metricPose').textContent=`${(+r.pose.x||0).toFixed(1)}, ${(+r.pose.y||0).toFixed(1)}`;$('#metricYaw').textContent=`${Math.round((+r.pose.yaw||0)*180/Math.PI)}°`;const run=statusData.task_run;$('#runState').className='run-state '+run.state;$('#runState').textContent=run.state==='idle'?'暂无运行任务':({running:'任务执行中',completed:'任务已完成',failed:'任务失败',canceled:'任务已停止'}[run.state]||run.state);$('#runMessage').textContent=run.message||'在“积木任务”中创建并运行巡逻流程。';const task=tasks.find(t=>t.id===run.task_id),total=task?.steps?.length||0;$('#runProgress').style.width=total?`${Math.min(100,(run.step/total)*100)}%`:'0';const navRunning=!!statusData.services.navigation?.running;$('#navState').textContent=navRunning?`状态：${r.nav_status||'启动中'}`:'导航未启动';$('#startNav').textContent=navRunning?'导航运行中':'启动导航';$('#startMapping').textContent=statusData.services.mapping?.running?'建图运行中':'开始建图'}catch(e){$('#robotStatus').textContent='巡逻服务离线'}}
async function loadData(){checkpoints=await api('/api/checkpoints');tasks=await api('/api/tasks');try{c12Presets=await api('/api/c12/presets')}catch(e){c12Presets=[]}$('#metricPoints').textContent=checkpoints.length;$('#metricTasks').textContent=tasks.length;renderPoints();renderTasks();try{mapData=await api('/api/map')}catch(e){}try{cloudData=await api('/api/cloud')}catch(e){}cloudViewer?.update(cloudData);drawMaps();if(workspace)workspace.updateToolbox(toolbox())}

function item(point,nav=false){const row=document.createElement('div');row.className='list-item';row.innerHTML=`<div class="item-head"><span class="item-name"></span></div><div class="item-meta">X ${point.x.toFixed(2)} · Y ${point.y.toFixed(2)} · ${Math.round(point.yaw*180/Math.PI)}°</div><div class="item-actions"></div>`;row.querySelector('.item-name').textContent=point.name;const actions=row.querySelector('.item-actions'),go=document.createElement('button');go.className='primary';go.textContent='导航到此';go.onclick=async()=>{try{if(!statusData.services?.navigation?.running)throw Error('导航服务未启动，请先点击“启动导航”');if(!confirm(`确认让机器狗自主导航到“${point.name}”？\n请握住遥控器并确保路线无人、急停可用。`))return;await api('/api/navigation/goal',{id:point.id,confirm:true});toast('导航目标已下发：'+point.name)}catch(e){toast(e.message)}};actions.append(go);if(!nav){const del=document.createElement('button');del.textContent='删除';del.onclick=async()=>{if(confirm('删除巡查点“'+point.name+'”？')){await api('/api/checkpoints/delete',{id:point.id});loadData()}};actions.append(del)}return row}
function renderPoints(){for(const selector of ['#pointList','#navPointList']){const box=$(selector);box.replaceChildren();if(!checkpoints.length){box.innerHTML='<div class="list-empty">暂无巡查点<br>在建图页面点击地图添加</div>';continue}checkpoints.forEach(p=>box.append(item(p,selector==='#navPointList')))}}

function mapTransform(canvas){const rect=canvas.getBoundingClientRect(),dpr=devicePixelRatio||1;if(canvas.width!==rect.width*dpr||canvas.height!==rect.height*dpr){canvas.width=rect.width*dpr;canvas.height=rect.height*dpr}return {ctx:canvas.getContext('2d'),w:canvas.width,h:canvas.height,dpr}}
function worldToScreen(x,y,w,h){return [w/2+(x-view.ox)*view.scale,-(y-view.oy)*view.scale+h/2]}
function screenToWorld(x,y,w,h){return [(x-w/2)/view.scale+view.ox,-(y-h/2)/view.scale+view.oy]}
function draw(canvas){if(!canvas)return;const {ctx,w,h,dpr}=mapTransform(canvas);ctx.fillStyle='#06090d';ctx.fillRect(0,0,w,h);ctx.lineWidth=dpr;const grid=view.scale*dpr;ctx.strokeStyle='#172231';ctx.beginPath();for(let x=(w/2-view.ox*view.scale*dpr)%grid;x<w;x+=grid){ctx.moveTo(x,0);ctx.lineTo(x,h)}for(let y=(h/2+view.oy*view.scale*dpr)%grid;y<h;y+=grid){ctx.moveTo(0,y);ctx.lineTo(w,y)}ctx.stroke();if(mapData.cells&&mapData.width){const res=mapData.resolution||.05,origin=mapData.origin||[0,0];ctx.fillStyle='#b8c2cc';const step=Math.max(1,Math.ceil(1/(res*view.scale)));for(let gy=0;gy<mapData.height;gy+=step)for(let gx=0;gx<mapData.width;gx+=step){const v=mapData.cells[gy*mapData.width+gx];if(v<0)continue;const [sx,sy]=worldToScreen(origin[0]+gx*res,origin[1]+gy*res,w/dpr,h/dpr);ctx.fillStyle=v>50?'#151b22':'#bbc5ce';ctx.fillRect(sx*dpr,(sy-step*res*view.scale)*dpr,Math.max(1,step*res*view.scale*dpr),Math.max(1,step*res*view.scale*dpr))}}checkpoints.forEach((p,i)=>{const [x,y]=worldToScreen(p.x,p.y,w/dpr,h/dpr);ctx.fillStyle='#39a7ff';ctx.beginPath();ctx.arc(x*dpr,y*dpr,7*dpr,0,Math.PI*2);ctx.fill();ctx.strokeStyle='#d8eeff';ctx.stroke();ctx.fillStyle='#eaf5ff';ctx.font=`${12*dpr}px sans-serif`;ctx.fillText(p.name,x*dpr+10*dpr,y*dpr-7*dpr)});const pose=statusData.robot?.pose;if(pose){const [x,y]=worldToScreen(+pose.x||0,+pose.y||0,w/dpr,h/dpr),a=-(+pose.yaw||0);ctx.save();ctx.translate(x*dpr,y*dpr);ctx.rotate(a);ctx.fillStyle='#35d582';ctx.beginPath();ctx.moveTo(13*dpr,0);ctx.lineTo(-9*dpr,-8*dpr);ctx.lineTo(-9*dpr,8*dpr);ctx.closePath();ctx.fill();ctx.restore()}ctx.fillStyle='#6f829a';ctx.font=`${11*dpr}px ui-monospace`;ctx.fillText(mapData.available?`${mapData.width}×${mapData.height} · ${(mapData.resolution||0).toFixed(2)}m`:'等待 /map，占位坐标网格可先创建点位',12*dpr,20*dpr)}
class PointCloudViewer{
 constructor(canvas){
  this.canvas=canvas;this.yaw=-.75;this.pitch=.72;this.distance=12;this.radius=5;this.updatedAt=0;this.drag=null;this.center=[0,0,0];this.selecting=false;
  this.gl=canvas.getContext('webgl',{antialias:true,alpha:false});
  if(!this.gl){$('#mapInfo').textContent='浏览器不支持WebGL';return}
  const vs=`attribute vec3 aPosition;attribute float aHeight;uniform float uYaw,uPitch,uDistance,uAspect;varying float vHeight;void main(){float cy=cos(uYaw),sy=sin(uYaw),cp=cos(uPitch),sp=sin(uPitch);vec3 q=vec3(cy*aPosition.x-sy*aPosition.y,sy*aPosition.x+cy*aPosition.y,aPosition.z);vec3 r=vec3(q.x,cp*q.y-sp*q.z,sp*q.y+cp*q.z);float depth=max(.1,uDistance-r.z);gl_Position=vec4(r.x*1.7/uAspect,r.y*1.7,depth-.2,depth);gl_PointSize=clamp(34.0/depth,1.4,5.0);vHeight=aHeight;}`;
  const fs=`precision mediump float;varying float vHeight;void main(){vec2 d=gl_PointCoord-vec2(.5);if(dot(d,d)>.25)discard;float h=clamp(vHeight,0.0,1.0);vec3 c=h<.33?mix(vec3(.08,.35,1.0),vec3(.0,1.0,.85),h/0.33):h<.66?mix(vec3(.0,1.0,.85),vec3(1.0,.9,.05),(h-.33)/.33):mix(vec3(1.0,.9,.05),vec3(1.0,.12,.08),(h-.66)/.34);gl_FragColor=vec4(c,1.0);}`;
  const shader=(type,source)=>{const s=this.gl.createShader(type);this.gl.shaderSource(s,source);this.gl.compileShader(s);if(!this.gl.getShaderParameter(s,this.gl.COMPILE_STATUS))throw Error(this.gl.getShaderInfoLog(s));return s};
  this.program=this.gl.createProgram();this.gl.attachShader(this.program,shader(this.gl.VERTEX_SHADER,vs));this.gl.attachShader(this.program,shader(this.gl.FRAGMENT_SHADER,fs));this.gl.linkProgram(this.program);this.gl.useProgram(this.program);
  this.positionBuffer=this.gl.createBuffer();this.heightBuffer=this.gl.createBuffer();this.count=0;
  this.aPosition=this.gl.getAttribLocation(this.program,'aPosition');this.aHeight=this.gl.getAttribLocation(this.program,'aHeight');
  this.uYaw=this.gl.getUniformLocation(this.program,'uYaw');this.uPitch=this.gl.getUniformLocation(this.program,'uPitch');this.uDistance=this.gl.getUniformLocation(this.program,'uDistance');this.uAspect=this.gl.getUniformLocation(this.program,'uAspect');
  canvas.addEventListener('pointerdown',e=>{canvas.setPointerCapture?.(e.pointerId);if(this.selecting)return;this.drag={x:e.clientX,y:e.clientY};canvas.classList.add('dragging')});
  canvas.addEventListener('pointermove',e=>{if(this.selecting||!this.drag)return;this.yaw+=(e.clientX-this.drag.x)*.007;this.pitch=Math.max(-1.45,Math.min(1.45,this.pitch+(e.clientY-this.drag.y)*.007));this.drag={x:e.clientX,y:e.clientY};this.render()});
  canvas.addEventListener('pointerup',e=>{if(this.selecting){const point=this.unproject(e.clientX,e.clientY,0);this.selecting=false;canvas.classList.remove('selecting');$('#pickPoint').classList.remove('success');if(point){const pose=statusData.robot?.pose||{yaw:0};openPoint(point.x,point.y,+pose.yaw||0)}else toast('未能选中地面，请重新点选');return}this.drag=null;canvas.classList.remove('dragging')});
  canvas.addEventListener('pointercancel',()=>{this.drag=null;canvas.classList.remove('dragging')});
  canvas.addEventListener('wheel',e=>{e.preventDefault();this.distance=Math.max(this.radius*.45,Math.min(this.radius*12,this.distance*Math.exp(e.deltaY*.001)));this.render()},{passive:false});
  canvas.addEventListener('dblclick',()=>this.reset());
 }
 reset(){this.yaw=-.75;this.pitch=.72;this.distance=Math.max(3,this.radius*1.6);this.render()}
 beginPick(){if(!this.count){toast('尚未收到三维点云');return}this.selecting=true;this.yaw=0;this.pitch=0;this.distance=Math.max(3,this.radius*1.6);this.canvas.classList.add('selecting');$('#pickPoint').classList.add('success');this.render();toast('已切换俯视图，请在可通行地面单击')}
 cameraToWorld(v){const cy=Math.cos(this.yaw),sy=Math.sin(this.yaw),cp=Math.cos(this.pitch),sp=Math.sin(this.pitch),qx=v[0],qy=cp*v[1]+sp*v[2],qz=-sp*v[1]+cp*v[2];return [cy*qx+sy*qy,-sy*qx+cy*qy,qz]}
 unproject(clientX,clientY,planeZ=0){const rect=this.canvas.getBoundingClientRect(),nx=2*(clientX-rect.left)/rect.width-1,ny=1-2*(clientY-rect.top)/rect.height,aspect=rect.width/rect.height;const o=this.cameraToWorld([0,0,this.distance]),d=this.cameraToWorld([nx*aspect/1.7,ny/1.7,-1]),origin=[o[0]+this.center[0],o[1]+this.center[1],o[2]+this.center[2]];if(Math.abs(d[2])<1e-5)return null;const t=(planeZ-origin[2])/d[2];if(t<=0)return null;return {x:origin[0]+t*d[0],y:origin[1]+t*d[1]}}
 project(x,y,z=0){if(!this.count)return null;const px=x-this.center[0],py=y-this.center[1],pz=z-this.center[2],cy=Math.cos(this.yaw),sy=Math.sin(this.yaw),cp=Math.cos(this.pitch),sp=Math.sin(this.pitch),qx=cy*px-sy*py,qy=sy*px+cy*py,rx=qx,ry=cp*qy-sp*pz,rz=sp*qy+cp*pz,depth=this.distance-rz;if(depth<=.1)return null;const rect=this.canvas.getBoundingClientRect(),nx=rx*1.7/(rect.width/rect.height)/depth,ny=ry*1.7/depth;if(Math.abs(nx)>1||Math.abs(ny)>1)return null;return {x:(nx+1)*rect.width/2,y:(1-ny)*rect.height/2}}
 update(data){
  if(!this.gl||!data?.available||!data.points?.length||data.updated_at===this.updatedAt)return;
  this.updatedAt=data.updated_at;const source=data.points,b=data.bounds||[[0,0,0],[1,1,1]],center=[(b[0][0]+b[1][0])/2,(b[0][1]+b[1][1])/2,(b[0][2]+b[1][2])/2];this.center=center;
  const dx=b[1][0]-b[0][0],dy=b[1][1]-b[0][1],dz=b[1][2]-b[0][2];this.radius=Math.max(1,Math.hypot(dx,dy,dz)/2);if(!this.count)this.distance=this.radius*1.6;
  const positions=new Float32Array(source.length),heights=new Float32Array(source.length/3),zspan=Math.max(.01,dz);
  for(let i=0,j=0;i<source.length;i+=3,j++){positions[i]=source[i]-center[0];positions[i+1]=source[i+1]-center[1];positions[i+2]=source[i+2]-center[2];heights[j]=(source[i+2]-b[0][2])/zspan}
  this.count=heights.length;const gl=this.gl;gl.bindBuffer(gl.ARRAY_BUFFER,this.positionBuffer);gl.bufferData(gl.ARRAY_BUFFER,positions,gl.STATIC_DRAW);gl.bindBuffer(gl.ARRAY_BUFFER,this.heightBuffer);gl.bufferData(gl.ARRAY_BUFFER,heights,gl.STATIC_DRAW);this.render();
 }
 render(){
  if(!this.gl)return;const gl=this.gl,rect=this.canvas.getBoundingClientRect(),dpr=devicePixelRatio||1,w=Math.max(1,Math.round(rect.width*dpr)),h=Math.max(1,Math.round(rect.height*dpr));if(this.canvas.width!==w||this.canvas.height!==h){this.canvas.width=w;this.canvas.height=h}gl.viewport(0,0,w,h);gl.clearColor(.015,.025,.04,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);if(!this.count)return;gl.enable(gl.DEPTH_TEST);gl.useProgram(this.program);gl.uniform1f(this.uYaw,this.yaw);gl.uniform1f(this.uPitch,this.pitch);gl.uniform1f(this.uDistance,this.distance);gl.uniform1f(this.uAspect,w/h);gl.bindBuffer(gl.ARRAY_BUFFER,this.positionBuffer);gl.enableVertexAttribArray(this.aPosition);gl.vertexAttribPointer(this.aPosition,3,gl.FLOAT,false,0,0);gl.bindBuffer(gl.ARRAY_BUFFER,this.heightBuffer);gl.enableVertexAttribArray(this.aHeight);gl.vertexAttribPointer(this.aHeight,1,gl.FLOAT,false,0,0);gl.drawArrays(gl.POINTS,0,this.count)
 }
}
cloudViewer=new PointCloudViewer($('#cloudCanvas'));
function renderCloudMarkers(){const layer=$('#cloudMarkers');if(!layer)return;layer.replaceChildren();checkpoints.forEach(point=>{const screen=cloudViewer?.project(point.x,point.y,0);if(!screen)return;const marker=document.createElement('span');marker.className='cloud-marker';marker.style.left=screen.x+'px';marker.style.top=screen.y+'px';marker.textContent=point.name;layer.append(marker)})}
function drawMaps(){draw($('#navCanvas'));cloudViewer?.render();renderCloudMarkers();$('#mapInfo').textContent=cloudData.available?`累计 ${Number(cloudData.total_points||0).toLocaleString()} 点 · 显示 ${Number(cloudData.shown_points||0).toLocaleString()} · ${cloudData.frame||'map_level'}`:'等待 MID360 /Laser_map 三维点云'}
addEventListener('resize',drawMaps);setInterval(drawMaps,1000);
function openPoint(x,y,yaw){$('#pointName').value='';$('#pointX').value=x.toFixed(2);$('#pointY').value=y.toFixed(2);$('#pointYaw').value=Math.round(yaw*180/Math.PI);$('#pointDialog').showModal()}
$('#addCurrent').onclick=()=>{const p=statusData.robot?.pose||{x:0,y:0,yaw:0};openPoint(+p.x||0,+p.y||0,+p.yaw||0)};
$('#pickPoint').onclick=()=>cloudViewer?.beginPick();
$('#confirmPoint').onclick=async e=>{e.preventDefault();try{await api('/api/checkpoints/save',{name:$('#pointName').value,x:+$('#pointX').value,y:+$('#pointY').value,yaw:+$('#pointYaw').value*Math.PI/180});$('#pointDialog').close();await loadData();toast('巡查点已保存')}catch(err){toast(err.message)}};

$('#startMapping').onclick=async()=>{try{await api('/api/mapping/start',{});toast('MID360三维建图正在启动')}catch(e){toast(e.message)}};$('#stopMapping').onclick=async()=>{await api('/api/mapping/stop',{});toast('建图服务已停止')};$('#startNav').onclick=async()=>{try{await api('/api/navigation/start',{});toast('导航服务正在启动')}catch(e){toast(e.message)}};$('#stopNav').onclick=async()=>{await api('/api/navigation/stop',{});toast('导航服务已停止')};
$('#saveMap').onclick=async()=>{const name=prompt('三维地图名称','patrol3d_'+new Date().toISOString().slice(0,10).replaceAll('-',''));if(!name)return;try{const j=await api('/api/maps/save',{name});toast(j.nav_error?`PCD已保存，但导航层失败：${j.nav_error}`:`PCD和导航层已保存：${j.name}（${Number(j.points||0).toLocaleString()}点）`)}catch(e){toast(e.message)}};

const driveKeys=new Set(['u','i','o','j','k','l','m',',','.','q','e']);
function driveButton(key){return $(`.drive-key[data-drive="${CSS.escape(key)}"]`)}
function renderTeleop(state){
 if(!state)return;
 teleopArmed=!!state.armed;
 $('#teleopState').textContent=teleopArmed?(state.message||'已解锁'):'已锁定';
 $('#teleopState').className='teleop-state '+(teleopArmed?'armed':'');
 $('#armTeleop').textContent=teleopArmed?'锁定控制':'解锁控制';
 $('#armTeleop').className=teleopArmed?'success':'danger';
 if(!teleopArmed&&teleopKey)clearDriveUI();
}
function clearDriveUI(){
 if(teleopTimer){clearInterval(teleopTimer);teleopTimer=null}
 $$('.drive-key.active').forEach(button=>button.classList.remove('active'));
 teleopKey=null;
}
async function postDrive(key,pressed){
 try{return await api('/api/teleop',{key,pressed,speed:+$('#teleopSpeed').value})}
 catch(error){clearDriveUI();toast(error.message);throw error}
}
function stopDrive(send=true){
 const previous=teleopKey;clearDriveUI();
 if(send&&previous)postDrive('k',false).catch(()=>{});
}
function startDrive(key){
 key=key.toLowerCase();
 if(key==='k'){stopDrive(false);postDrive('k',false).catch(()=>{});return}
 if(!teleopArmed){toast('请先开始建图并解锁控制');return}
 if(teleopKey===key)return;
 stopDrive();teleopKey=key;
 driveButton(key)?.classList.add('active');
 postDrive(key,true).catch(()=>{});
 teleopTimer=setInterval(()=>{if(teleopKey===key)postDrive(key,true).catch(()=>{})},100);
}
$('#armTeleop').onclick=async()=>{
 try{
  if(!teleopArmed&&!confirm('确认解锁机器狗移动控制？请确保遥控器急停可用，人员远离机器狗关节。'))return;
  stopDrive();renderTeleop(await api('/api/teleop/arm',{armed:!teleopArmed}));
  toast(teleopArmed?'控制已解锁，按住方向键运动':'控制已锁定');
 }catch(error){toast(error.message)}
};
$$('.drive-key').forEach(button=>{
 const key=button.dataset.drive;
 button.addEventListener('pointerdown',event=>{event.preventDefault();button.setPointerCapture?.(event.pointerId);startDrive(key)});
 for(const name of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(name,event=>{event.preventDefault();if(teleopKey===key)stopDrive()});
});
addEventListener('keydown',event=>{
 const key=event.key.toLowerCase();
 if(!driveKeys.has(key)||event.ctrlKey||event.altKey||event.metaKey)return;
 if(['INPUT','TEXTAREA','SELECT'].includes(event.target.tagName)||event.target.isContentEditable)return;
 event.preventDefault();if(!event.repeat)startDrive(key);
});
addEventListener('keyup',event=>{const key=event.key.toLowerCase();if(driveKeys.has(key)){event.preventDefault();if(key==='k'||teleopKey===key)stopDrive()}});
addEventListener('blur',()=>stopDrive());
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopDrive()});
addEventListener('pagehide',()=>{clearDriveUI();navigator.sendBeacon('/api/teleop',new Blob([JSON.stringify({key:'k',pressed:false})],{type:'application/json'}))});
setInterval(()=>renderTeleop(statusData.teleop),500);

function pointOptions(){return checkpoints.length?checkpoints.map(p=>[p.name,p.id]):[['请先创建巡查点','']]}
function presetOptions(){return c12Presets.length?c12Presets.map(p=>[p.name,p.id]):[['请先在C12控制台创建预置点','']]}
function toolbox(){return {kind:'categoryToolbox',contents:[{kind:'category',name:'巡逻流程',colour:'#3388cc',contents:[{kind:'block',type:'patrol_start'},{kind:'block',type:'patrol_repeat'}]},{kind:'category',name:'导航',colour:'#2b9a64',contents:[{kind:'block',type:'patrol_goto'},{kind:'block',type:'patrol_wait'}]},{kind:'category',name:'C12 双光',colour:'#b56b2f',contents:[{kind:'block',type:'patrol_palette'},{kind:'block',type:'patrol_preset'},{kind:'block',type:'patrol_photo'}]}]}}
Blockly.common.defineBlocksWithJsonArray([
 {type:'patrol_start',message0:'▶ 开始巡逻任务',nextStatement:null,colour:210,tooltip:'任务入口'},
 {type:'patrol_goto',message0:'导航到巡查点 %1 超时 %2 秒',args0:[{type:'field_dropdown',name:'POINT',options:pointOptions},{type:'field_number',name:'TIMEOUT',value:120,min:10,max:600}],previousStatement:null,nextStatement:null,colour:145},
 {type:'patrol_wait',message0:'原地等待 %1 秒',args0:[{type:'field_number',name:'SECONDS',value:5,min:0,max:3600}],previousStatement:null,nextStatement:null,colour:145},
 {type:'patrol_palette',message0:'热成像切换为 %1',args0:[{type:'field_dropdown',name:'CODE',options:[['白热','01'],['铁红','04'],['彩虹','05'],['红热','08'],['黑热','0B'],['高亮热','0C']]}],previousStatement:null,nextStatement:null,colour:30},
 {type:'patrol_preset',message0:'调用云台预置点 %1',args0:[{type:'field_dropdown',name:'PRESET',options:presetOptions}],previousStatement:null,nextStatement:null,colour:30},
 {type:'patrol_photo',message0:'记录双光巡查画面',previousStatement:null,nextStatement:null,colour:30},
 {type:'patrol_repeat',message0:'重复 %1 次',args0:[{type:'field_number',name:'COUNT',value:2,min:1,max:100}],message1:'执行 %1',args1:[{type:'input_statement',name:'DO'}],previousStatement:null,nextStatement:null,colour:210}
]);
workspace=Blockly.inject('blocklyDiv',{toolbox:toolbox(),theme:Blockly.Themes.Dark,trashcan:true,renderer:'zelos',grid:{spacing:20,length:3,colour:'#36465b',snap:true},zoom:{controls:true,wheel:true,startScale:.9,maxScale:1.5,minScale:.45}});
function compileChain(block,out){while(block){const type=block.type;if(type==='patrol_goto')out.push({type:'goto',checkpoint_id:block.getFieldValue('POINT'),timeout:+block.getFieldValue('TIMEOUT'),label:'导航到 '+(block.getField('POINT').getText())});else if(type==='patrol_wait')out.push({type:'wait',seconds:+block.getFieldValue('SECONDS'),label:'等待 '+block.getFieldValue('SECONDS')+' 秒'});else if(type==='patrol_palette')out.push({type:'palette',code:block.getFieldValue('CODE'),label:'切换热成像 '+block.getField('CODE').getText()});else if(type==='patrol_preset')out.push({type:'preset',preset_id:block.getFieldValue('PRESET'),label:'调用云台预置点 '+block.getField('PRESET').getText()});else if(type==='patrol_photo')out.push({type:'photo',label:'记录双光画面'});else if(type==='patrol_repeat'){const nested=[];compileChain(block.getInputTargetBlock('DO'),nested);for(let i=0;i<+block.getFieldValue('COUNT');i++)out.push(...nested.map(x=>({...x})))}block=block.getNextBlock()}return out}
function compile(){const start=workspace.getTopBlocks(true).find(b=>b.type==='patrol_start'),out=[];if(start)compileChain(start.getNextBlock(),out);return out}
function preview(){const steps=compile(),ol=$('#stepPreview');ol.replaceChildren();if(!steps.length)ol.innerHTML='<li class="muted">尚未连接执行积木</li>';steps.forEach(s=>{const li=document.createElement('li');li.textContent=s.label;ol.append(li)})}workspace.addChangeListener(e=>{if(!e.isUiEvent)preview()});
function renderTasks(){const box=$('#taskList');box.replaceChildren();if(!tasks.length){box.innerHTML='<div class="list-empty">暂无任务</div>';return}tasks.forEach(t=>{const row=document.createElement('div');row.className='list-item';row.innerHTML='<div class="item-head"><span class="item-name"></span></div><div class="item-meta"></div><div class="item-actions"></div>';row.querySelector('.item-name').textContent=t.name;row.querySelector('.item-meta').textContent=`${t.steps.length} 步 · ${t.updated_at}`;row.onclick=e=>{if(e.target.tagName==='BUTTON')return;currentTask=t;$('#taskName').value=t.name;workspace.clear();Blockly.serialization.workspaces.load(t.workspace,workspace);preview();toast('已载入 '+t.name)};const run=document.createElement('button');run.className='success';run.textContent='运行';run.onclick=async()=>{try{await api('/api/tasks/run',{id:t.id});toast('任务已启动')}catch(e){toast(e.message)}};const del=document.createElement('button');del.textContent='删除';del.onclick=async()=>{if(confirm('删除任务“'+t.name+'”？')){await api('/api/tasks/delete',{id:t.id});if(currentTask?.id===t.id)currentTask=null;loadData()}};row.querySelector('.item-actions').append(run,del);box.append(row)})}
$('#newTask').onclick=()=>{currentTask=null;$('#taskName').value='';workspace.clear();const b=workspace.newBlock('patrol_start');b.initSvg();b.render();b.moveBy(40,40);preview()};
$('#saveTask').onclick=async()=>{try{currentTask=await api('/api/tasks/save',{id:currentTask?.id,name:$('#taskName').value,workspace:Blockly.serialization.workspaces.save(workspace),steps:compile()});await loadData();toast('任务已保存')}catch(e){toast(e.message)}};
$('#runTask').onclick=async()=>{try{if(!currentTask)throw Error('请先保存任务');if(!confirm(`确认运行巡逻任务“${currentTask.name}”？\n请确保路线无人并握住遥控器。`))return;await api('/api/tasks/run',{id:currentTask.id,confirm:true});toast('任务已启动')}catch(e){toast(e.message)}};$('#cancelTask').onclick=async()=>{await api('/api/tasks/stop',{});toast('已请求停止任务')};

function showConfig(){const c=statusData.config||{};$('#robotName').value=c.robot_name||'';$('#sensor').value='mid360';$('#scanTopic').value=c.scan_topic||'/scan';$('#odomTopic').value=c.odom_topic||'/Odometry';$('#cmdTopic').value=c.cmd_vel_topic||'/cmd_vel';$('#mapFrame').value=c.map_frame||'map_level';$('#baseFrame').value=c.base_frame||'base_link'}
$('#saveConfig').onclick=async()=>{try{await api('/api/config',{robot_name:$('#robotName').value,sensor:$('#sensor').value,scan_topic:$('#scanTopic').value,odom_topic:$('#odomTopic').value,cmd_vel_topic:$('#cmdTopic').value,map_frame:$('#mapFrame').value,base_frame:$('#baseFrame').value});toast('设备设置已保存');refresh()}catch(e){toast(e.message)}};

(async()=>{await refresh();showConfig();await loadData();$('#newTask').click();setInterval(refresh,2000);setInterval(async()=>{try{mapData=await api('/api/map')}catch(e){}try{cloudData=await api('/api/cloud');cloudViewer?.update(cloudData)}catch(e){}drawMaps()},1500)})();
