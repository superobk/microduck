// MIT. Right-handed robotics coordinates: X forward, Y left, Z up.
export const I = () => new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
export function normalize(v){const n=Math.hypot(...v); if(!Number.isFinite(n)||n<1e-10)throw new Error('Invalid norm');return v.map(x=>x/n);}
export function qmatrix(wxyz){
 const [w,x,y,z]=normalize(wxyz);
 return new Float32Array([1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w),0,
  2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w),0,
  2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y),0, 0,0,0,1]);
}
export function mul(a,b){const o=new Float32Array(16);for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)o[c*4+r]+=a[k*4+r]*b[c*4+k];return o;}
export function transform(m,v){const a=[...v,1],o=[0,0,0,0];for(let r=0;r<4;r++)for(let c=0;c<4;c++)o[r]+=m[c*4+r]*a[c];return o;}
export function translation(x,y,z){const m=I();m[12]=x;m[13]=y;m[14]=z;return m;}
export function scaling(x,y,z){const m=I();m[0]=x;m[5]=y;m[10]=z;return m;}
export function perspective(fov,aspect,near,far){const f=1/Math.tan(fov/2),m=new Float32Array(16);m[0]=f/aspect;m[5]=f;m[10]=(far+near)/(near-far);m[11]=-1;m[14]=2*far*near/(near-far);return m;}
function cross(a,b){return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
function dot(a,b){return a.reduce((n,v,i)=>n+v*b[i],0);}
export function lookAt(eye,center,up){const z=normalize(eye.map((e,i)=>e-center[i])),x=normalize(cross(up,z)),y=cross(z,x);return new Float32Array([x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,eye),-dot(y,eye),-dot(z,eye),1]);}
