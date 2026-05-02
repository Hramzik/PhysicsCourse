export class RigidBody3D{
  constructor({mass=1, size=[1,1,1], position=[0,0,0], quaternion=null, angularVelocity=[0,0,0]}){
    this.mass = mass;
    this.size = size.slice();
    this.position = new THREE.Vector3(...position);
    this.quaternion = quaternion ? quaternion.clone() : new THREE.Quaternion();
    this.angularVelocity = new THREE.Vector3(...angularVelocity);
    this._computeBodyInertia();
    this.L = this.angularMomentum();
  }

  _computeBodyInertia(){
    const [x,y,z] = this.size;
    const m = this.mass;
    const ix = (1/12)*m*(y*y+z*z);
    const iy = (1/12)*m*(x*x+z*z);
    const iz = (1/12)*m*(x*x+y*y);
    this.inertiaLocal = new THREE.Matrix3();
    this.inertiaLocal.set(
      ix,0,0,
      0,iy,0,
      0,0,iz
    );
  }

  getRotationMatrix(q){
    const m4 = new THREE.Matrix4().makeRotationFromQuaternion(q);
    const R = new THREE.Matrix3().setFromMatrix4(m4);
    return R;
  }

  // compute world inertia matrix: R * I_local * R^T
  getInertiaGlobal(){
    const R = this.getRotationMatrix(this.quaternion);
    const Rt = new THREE.Matrix3().copy(R).transpose();
    const inertiaGlobal = new THREE.Matrix3();
    inertiaGlobal.multiplyMatrices(R, this.inertiaLocal);
    inertiaGlobal.multiply(Rt);
    return inertiaGlobal;
  }

  // angular momentum L = I_world * omega
  angularMomentum(){
    const Iw = this.getInertiaGlobal();
    const w = this.angularVelocity;
    const l = new THREE.Vector3();
    l.x = Iw.elements[0]*w.x + Iw.elements[1]*w.y + Iw.elements[2]*w.z;
    l.y = Iw.elements[3]*w.x + Iw.elements[4]*w.y + Iw.elements[5]*w.z;
    l.z = Iw.elements[6]*w.x + Iw.elements[7]*w.y + Iw.elements[8]*w.z;
    return l;
  }
}
