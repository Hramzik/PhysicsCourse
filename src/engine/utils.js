export function convertToQuaternion(v){
  return new THREE.Quaternion(v.x, v.y, v.z, 0);
}

export function add(q1, q2){
  return new THREE.Quaternion(
    q1.x + q2.x,
    q1.y + q2.y,
    q1.z + q2.z,
    q1.w + q2.w
  );
}

export function multiplyScalar(q, s){
  return new THREE.Quaternion(q.x * s, q.y * s, q.z * s, q.w * s);
}

export function skew(v){
  return new THREE.Matrix3().set(
    0, -v.z, v.y,
    v.z, 0, -v.x,
    -v.y, v.x, 0
  );
}

export function addMatrix3(A, B){
  const a = A.elements;
  const b = B.elements;
  const R = new THREE.Matrix3();
  const r = R.elements;
  for(let i=0;i<9;i++) r[i] = a[i] + b[i];
  return R;
}

export function multiplyMatrix3Scalar(M, s){
  const R = M.clone();
  const r = R.elements;
  for(let i=0;i<9;i++) r[i] *= s;
  return R;
}

export function identityMatrix3(){
  const I = new THREE.Matrix3();
  I.set(1,0,0, 0,1,0, 0,0,1);
  return I;
}
