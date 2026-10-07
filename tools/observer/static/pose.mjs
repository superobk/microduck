// This view removes heading explicitly, without integrating gyro or overwriting
// measured quaternions. The 10 leg DOFs still use real servo angles. Raw attitude
// remains visible: a stable picture must never certify a faulty fused stream.
export function tiltOnly(q) {
  if (!Array.isArray(q) || q.length !== 4 || !q.every(Number.isFinite)) return null;
  const n = Math.hypot(...q);
  if (n < .5) return null;
  const [w,x,y,z] = q.map(v=>v/n);
  // R(q)^T * [0,0,-1] is measured gravity in body coordinates. Construct
  // R_y(pitch) R_x(roll), with display yaw explicitly zero. At vertical pitch
  // roll is unobservable from gravity; choose zero rather than random heading.
  const gx = 2*(w*y-x*z), gy = -2*(y*z+w*x), gz = -(1-2*(x*x+y*y));
  const p = Math.asin(Math.max(-1,Math.min(1,gx)));
  const r = Math.hypot(gy,gz)<1e-8 ? 0 : Math.atan2(-gy,-gz);
  const cp=Math.cos(p/2),sp=Math.sin(p/2),cr=Math.cos(r/2),sr=Math.sin(r/2);
  return [cp*cr,cp*sr,sp*cr,-sp*sr];
}
