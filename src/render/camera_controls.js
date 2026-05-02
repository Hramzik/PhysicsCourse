export class CameraControls {
  constructor(camera){
    this.camera = camera;
    this.keys = {w:false,a:false,s:false,d:false,shift:false};
    this.baseSpeed = 2.2;
    this.shiftMultiplier = 3.5;
    this.dragging = false;
    this.pointer = {x:0,y:0};
    this.rotationSpeed = 0.0035;
    this.euler = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ');
    this._bindHandlers();
  }

  _bindHandlers(){
    window.addEventListener('keydown', this._onKeyDown = (e)=>{
      if(e.code === 'KeyW') this.keys.w = true;
      if(e.code === 'KeyA') this.keys.a = true;
      if(e.code === 'KeyS') this.keys.s = true;
      if(e.code === 'KeyD') this.keys.d = true;
      if(e.code === 'ShiftLeft' || e.code === 'ShiftRight') this.keys.shift = true;
    });
    window.addEventListener('keyup', this._onKeyUp = (e)=>{
      if(e.code === 'KeyW') this.keys.w = false;
      if(e.code === 'KeyA') this.keys.a = false;
      if(e.code === 'KeyS') this.keys.s = false;
      if(e.code === 'KeyD') this.keys.d = false;
      if(e.code === 'ShiftLeft' || e.code === 'ShiftRight') this.keys.shift = false;
    });

    window.addEventListener('mousedown', this._onMouseDown = (e)=>{
      if(e.button !== 0) return;
      this.dragging = true;
      this.pointer.x = e.clientX;
      this.pointer.y = e.clientY;
      window.document.body.style.cursor = 'grabbing';
    });

    window.addEventListener('mouseup', this._onMouseUp = (e)=>{
      if(e.button !== 0) return;
      this.dragging = false;
      window.document.body.style.cursor = 'default';
    });

    window.addEventListener('mousemove', this._onMouseMove = (e)=>{
      if(!this.dragging) return;
      const dx = e.clientX - this.pointer.x;
      const dy = e.clientY - this.pointer.y;
      this.pointer.x = e.clientX;
      this.pointer.y = e.clientY;
      this.rotateCamera(dx, dy);
    });
  }

  rotateCamera(dx, dy){
    this.euler.y -= dx * this.rotationSpeed;
    this.euler.x -= dy * this.rotationSpeed;
    const limit = Math.PI / 2 - 0.05;
    this.euler.x = Math.max(-limit, Math.min(limit, this.euler.x));
    this.camera.quaternion.setFromEuler(this.euler);
  }

  update(dt){
    const dir = new THREE.Vector3();
    const right = new THREE.Vector3();
    this.camera.getWorldDirection(dir);
    if(dir.lengthSq() < 1e-6) return;
    dir.normalize();
    right.copy(dir).cross(this.camera.up).normalize();

    let speed = this.baseSpeed * dt;
    if(this.keys.shift) speed *= this.shiftMultiplier;

    const move = new THREE.Vector3();
    if(this.keys.w) move.add(dir);
    if(this.keys.s) move.sub(dir);
    if(this.keys.d) move.add(right);
    if(this.keys.a) move.sub(right);
    if(move.lengthSq() === 0) return;
    move.normalize().multiplyScalar(speed);
    this.camera.position.add(move);
  }

  dispose(){
    window.removeEventListener('keydown', this._onKeyDown);
    window.removeEventListener('keyup', this._onKeyUp);
    window.removeEventListener('mousedown', this._onMouseDown);
    window.removeEventListener('mouseup', this._onMouseUp);
    window.removeEventListener('mousemove', this._onMouseMove);
  }
}
