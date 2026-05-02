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

export function scale(q, s){
  return new THREE.Quaternion(q.x * s, q.y * s, q.z * s, q.w * s);
}
