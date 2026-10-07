// The DUCK v1 parser/FK mirrors robotctl/src/duck.rs. Logical slot count stays 15;
// fixed head slots 5..8 are visual references, never additional connected servos.
import {I,mul,transform,qmatrix,translation,perspective,lookAt} from './math.mjs';
export const IDS=[20,21,22,23,24,30,31,32,33,34,10,11,12,13,14];
export const HEAD=[.3491,.3491,0,0];
export function parseDuck(buffer){
 const d=new DataView(buffer);let at=0;
 const take=(n)=>{if(at+n>d.byteLength)throw Error('鸭子资源截断');const i=at;at+=n;return i;};
 const u16=()=>d.getUint16(take(2),true),i16=()=>d.getInt16(take(2),true),f32=()=>d.getFloat32(take(4),true);
 const vec=()=>[f32(),f32(),f32()],quat=()=>[f32(),f32(),f32(),f32()];
 if(new TextDecoder().decode(new Uint8Array(buffer,take(4),4))!=='DUCK'||d.getUint32(take(4),true)!==1)throw Error('鸭子资源版本错误');
 const nm=u16(),nb=u16(),np=u16(),meshes=[],bodies=[],parts=[];
 if(nm>1000||nb>1000||np>1000)throw Error('资源计数错误');
 for(let i=0;i<nm;i++){const nv=u16(),nt=u16(),vertices=[],triangles=[];for(let j=0;j<nv;j++)vertices.push(vec());for(let j=0;j<nt;j++){const t=[u16(),u16(),u16()];if(t.some(k=>k>=nv))throw Error('网格索引错误');triangles.push(t);}meshes.push({vertices,triangles});}
 for(let i=0;i<nb;i++){const b={parent:i16(),joint:i16(),pos:vec(),quat:quat(),axis:vec()};if(b.parent>=i||b.parent < -1||b.joint < -1||b.joint>=15)throw Error('关节层级错误');bodies.push(b);}
 for(let i=0;i<np;i++){const body=u16(),mesh=u16(),offset=take(4),rgb=[0,1,2].map(k=>d.getUint8(offset+k)/255),pos=vec(),q=quat();if(body>=nb||mesh>=nm)throw Error('部件索引错误');parts.push({body,mesh,rgb,pos,quat:q});}
 if(at!==buffer.byteLength)throw Error('资源尾部多余数据');return {meshes,bodies,parts};
}
export function poses(model,joints,q){
 const result=[];
 for(const b of model.bodies){
  let rest=mul(translation(...b.pos),qmatrix(b.quat));
  if(b.joint>=0){const angle=Number.isFinite(joints[b.joint])?joints[b.joint]:0;const s=Math.sin(angle/2);rest=mul(rest,qmatrix([Math.cos(angle/2),...b.axis.map(v=>v*s)]));}
  // Root position is not rotated around world origin: IMU describes trunk orientation.
  const world=b.parent<0?mul(translation(...b.pos),mul(qmatrix(q),qmatrix(b.quat))):mul(result[b.parent],rest);
  result.push(world);
 }return result;
}
const VS=`#version 300 es
in vec3 aPosition;in vec3 aNormal;uniform mat4 uMVP;uniform mat4 uModel;out vec3 normal;
void main(){gl_Position=uMVP*vec4(aPosition,1.0);normal=mat3(uModel)*aNormal;}`;
const FS=`#version 300 es
precision highp float;in vec3 normal;uniform vec3 uColor;out vec4 color;
void main(){float l=.58+.42*max(dot(normalize(normal),normalize(vec3(1.,-2.,4.))),0.);color=vec4(uColor*l,1.);}`;
function expanded(mesh){const p=[],n=[];for(const ids of mesh.triangles){const [a,b,c]=ids.map(i=>mesh.vertices[i]),u=b.map((v,i)=>v-a[i]),v=c.map((x,i)=>x-a[i]);const normal=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];const len=Math.hypot(...normal)||1;for(const vertex of [a,b,c]){p.push(...vertex);n.push(...normal.map(v=>v/len));}}return {p,n};}
export class DuckScene{
 constructor(canvas,overlay,onSelect){
  this.canvas=canvas;this.overlay=overlay;this.onSelect=onSelect;this.q=[1,0,0,0];this.joints=Array(15).fill(0);HEAD.forEach((v,i)=>this.joints[5+i]=v);this.az=-.9;this.el=.3;this.distance=.65;this.valid=false;this.selected=null;
  // A diagnostic URL permits exercising the real software renderer on desktops
  // that support WebGL; it changes only drawing, never the telemetry source.
  this.gl=new URLSearchParams(location.search).get('renderer')==='canvas'?null:canvas.getContext('webgl2',{antialias:true,alpha:false});this.renderer=this.gl?'WebGL2':'Canvas 软件3D';
  if(this.gl){const g=this.gl;this.program=g.createProgram();for(const [type,source] of [[g.VERTEX_SHADER,VS],[g.FRAGMENT_SHADER,FS]]){const s=g.createShader(type);g.shaderSource(s,source);g.compileShader(s);if(!g.getShaderParameter(s,g.COMPILE_STATUS))throw Error(g.getShaderInfoLog(s));g.attachShader(this.program,s);}g.linkProgram(this.program);if(!g.getProgramParameter(this.program,g.LINK_STATUS))throw Error(g.getProgramInfoLog(this.program));this.loc={};for(const k of ['uMVP','uModel','uColor'])this.loc[k]=g.getUniformLocation(this.program,k);}
  else this.ctx=canvas.getContext('2d');
  let drag=null,origin=null;canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];origin=drag;canvas.setPointerCapture(e.pointerId);});
  canvas.addEventListener('pointermove',e=>{if(!drag)return;this.az-=(e.clientX-drag[0])*.007;this.el=Math.max(-.1,Math.min(1.4,this.el+(e.clientY-drag[1])*.006));drag=[e.clientX,e.clientY];});
  canvas.addEventListener('pointerup',e=>{if(origin&&Math.hypot(e.clientX-origin[0],e.clientY-origin[1])<5)this.pick(e);drag=origin=null;});canvas.addEventListener('pointercancel',()=>{drag=origin=null;});
  canvas.addEventListener('wheel',e=>{e.preventDefault();this.distance=Math.max(.35,Math.min(1.2,this.distance+e.deltaY*.0005));},{passive:false});
  fetch('/duck.bin').then(r=>{if(!r.ok)throw Error('模型资源不可用');return r.arrayBuffer();}).then(b=>{this.model=parseDuck(b);this.geometry=this.model.meshes.map(expanded);if(this.gl)this.gpu=this.geometry.map(mesh=>this.upload(mesh));}).catch(e=>{this.error=e.message;});
  this.draw=this.draw.bind(this);requestAnimationFrame(this.draw);
 }
 upload(mesh){const g=this.gl,vao=g.createVertexArray();g.bindVertexArray(vao);for(const [name,data] of [['aPosition',mesh.p],['aNormal',mesh.n]]){const b=g.createBuffer();g.bindBuffer(g.ARRAY_BUFFER,b);g.bufferData(g.ARRAY_BUFFER,new Float32Array(data),g.STATIC_DRAW);const a=g.getAttribLocation(this.program,name);g.enableVertexAttribArray(a);g.vertexAttribPointer(a,3,g.FLOAT,false,0,0);}return {vao,count:mesh.p.length/3};}
 set(q,joints,valid,fresh=[]){if(q&&q.length===4&&q.every(Number.isFinite))this.q=q;this.jointFresh=fresh;for(let i=0;i<15;i++)if(i!==9&&Number.isFinite(joints[i]))this.joints[i]=joints[i];this.joints[9]=0;HEAD.forEach((v,i)=>this.joints[5+i]=v);this.valid=valid;}
 reset(){this.az=-.9;this.el=.3;this.distance=.65;}
 project(v){const p=transform(this.vp,v);if(p[3]<=0)return null;return [this.w/2+p[0]/p[3]*this.w/2,this.h/2-p[1]/p[3]*this.h/2,p[2]/p[3]];}
 pick(e){if(!this.model||!this.world)return;const r=this.canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;let nearest=null,best=28;this.model.bodies.forEach((b,i)=>{if(b.joint<0||[5,6,7,8].includes(b.joint))return;const p=this.project(transform(this.world[i],[0,0,0]).slice(0,3));if(p){const d=Math.hypot(p[0]-x,p[1]-y);if(d<best){best=d;nearest=IDS[b.joint];}}});if(nearest!==null){this.selected=nearest;this.onSelect?.(nearest);}}
 draw(ms){requestAnimationFrame(this.draw);if(ms-(this.last||0)<(this.gl?25:70))return;this.last=ms;const w=this.canvas.clientWidth,h=this.canvas.clientHeight;if(!w||!h)return;this.w=w;this.h=h;const ratio=Math.min(2,devicePixelRatio||1);
  for(const c of [this.canvas,this.overlay])if(c.width!==Math.round(w*ratio)||c.height!==Math.round(h*ratio)){c.width=Math.round(w*ratio);c.height=Math.round(h*ratio);}
  const eye=[Math.cos(this.az)*Math.cos(this.el)*this.distance,Math.sin(this.az)*Math.cos(this.el)*this.distance,.14+Math.sin(this.el)*this.distance];this.vp=mul(perspective(.68,w/h,.01,4),lookAt(eye,[0,0,.14],[0,0,1]));
  const ctx=this.overlay.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
  const grid=(context)=>{context.strokeStyle='#d9e7dc';context.lineWidth=1;for(let i=-5;i<=5;i++)for(const ends of [[[i*.05,-.25,0],[i*.05,.25,0]],[[-.25,i*.05,0],[.25,i*.05,0]]]){const a=this.project(ends[0]),b=this.project(ends[1]);if(a&&b){context.beginPath();context.moveTo(a[0],a[1]);context.lineTo(b[0],b[1]);context.stroke();}}};
  if(this.gl){const g=this.gl;g.viewport(0,0,this.canvas.width,this.canvas.height);g.clearColor(.96,.98,.95,1);g.clear(g.COLOR_BUFFER_BIT|g.DEPTH_BUFFER_BIT);g.enable(g.DEPTH_TEST);g.useProgram(this.program);}else{this.ctx.setTransform(ratio,0,0,ratio,0,0);this.ctx.fillStyle='#f5faf2';this.ctx.fillRect(0,0,w,h);grid(this.ctx);}
  if(this.model){this.world=poses(this.model,this.joints,this.q);const triangles=[];
   for(const part of this.model.parts){const m=mul(this.world[part.body],mul(translation(...part.pos),qmatrix(part.quat))),joint=this.model.bodies[part.body].joint;let rgb=part.rgb.map(v=>.12+.88*v);if(IDS[joint]===this.selected)rgb=[.94,.69,.26];if(!this.valid||(joint>=0&&!this.jointFresh?.[joint]))rgb=rgb.map(v=>.42+v*.3);
    if(this.gl){const g=this.gl,mesh=this.gpu[part.mesh];g.uniformMatrix4fv(this.loc.uMVP,false,mul(this.vp,m));g.uniformMatrix4fv(this.loc.uModel,false,m);g.uniform3fv(this.loc.uColor,rgb);g.bindVertexArray(mesh.vao);g.drawArrays(g.TRIANGLES,0,mesh.count);}
    else{const mesh=this.model.meshes[part.mesh];for(const tri of mesh.triangles){const points=tri.map(i=>this.project(transform(m,mesh.vertices[i]).slice(0,3)));if(points.every(Boolean))triangles.push({points,depth:points.reduce((a,p)=>a+p[2],0)/3,color:`rgb(${rgb.map(v=>Math.round(v*220)).join(',')})`});}}
   }
   if(!this.gl){triangles.sort((a,b)=>b.depth-a.depth);for(const t of triangles){this.ctx.fillStyle=t.color;this.ctx.beginPath();t.points.forEach((p,i)=>i?this.ctx.lineTo(...p.slice(0,2)):this.ctx.moveTo(...p.slice(0,2)));this.ctx.closePath();this.ctx.fill();}}
   const root=this.world[0];for(const [v,label,color] of [[[.09,0,0],'X 前','#da8468'],[[0,.09,0],'Y 左','#54a48d'],[[0,0,.09],'Z 上','#728cda']]){const a=this.project(transform(root,[0,0,0]).slice(0,3)),b=this.project(transform(root,v).slice(0,3));if(a&&b){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();ctx.fillStyle=color;ctx.font='600 11px system-ui';ctx.fillText(label,b[0]+4,b[1]);}}
   const pos=transform(root,[0,0,0]).slice(0,3),a=this.project(pos),b=this.project([pos[0],pos[1],pos[2]-.08]);if(a&&b){ctx.strokeStyle='#8196a4';ctx.setLineDash([4,4]);ctx.beginPath();ctx.moveTo(...a.slice(0,2));ctx.lineTo(...b.slice(0,2));ctx.stroke();ctx.setLineDash([]);ctx.fillText('重力',b[0]+4,b[1]);}
  }
  ctx.fillStyle='#648478';ctx.font='12px system-ui';ctx.fillText(this.error||this.renderer,16,h-16);
 }
}
