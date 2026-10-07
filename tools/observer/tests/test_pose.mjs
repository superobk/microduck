// Execute with node --test. These physical invariants distinguish a heading
// display choice from freezing the entire attitude or rewriting sensor data.
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {tiltOnly} from '../static/pose.mjs';
import {qmatrix,transform} from '../static/math.mjs';
const mul=(a,b)=>{const[w,x,y,z]=a,[v,i,j,k]=b;return[w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v];};
const axis=(a,i)=>{const q=[Math.cos(a/2),0,0,0];q[i]=Math.sin(a/2);return q;};
const pose=(r,p,y)=>mul(mul(axis(y,3),axis(p,2)),axis(r,1));
const gravity=q=>{const m=qmatrix(q);return [-m[2],-m[6],-m[10]];};
const close=(a,b,t=1e-6)=>a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<t,`${a} != ${b}`));
test('arbitrary heading changes do not spin the tilt view, including upside-down body',()=>{
  for(const[r,p]of [[.3,-.4],[Math.PI-.02,-.4],[0,Math.PI/2],[.7,-Math.PI/2]]) {
    const expected=tiltOnly(pose(r,p,0));
    for(const y of [-3,-1,0,1,3]){
      const q=pose(r,p,y),copy=[...q],tilt=tiltOnly(q);
      close(tilt,expected);close(gravity(tilt),gravity(q));assert.deepEqual(q,copy);
    }
  }
});
test('real roll and pitch still move, valid upside-down attitude stays upside-down',()=>{
  assert.notDeepEqual(tiltOnly(pose(.5,.2,0)),tiltOnly(pose(0,0,0)));
  close(transform(qmatrix(tiltOnly(pose(Math.PI,0,2))),[0,0,1]).slice(0,3),[0,0,-1]);
});
test('missing, nonfinite or invalid pose never becomes measured upright',()=>{
  for(const q of [null,[],[NaN,0,0,0],[0,0,0,0]])assert.equal(tiltOnly(q),null);
});
