export function createRenderer(container){
  const renderer = new THREE.WebGLRenderer({antialias:true});
  renderer.setSize(container.clientWidth || 800, container.clientHeight || 600);
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x222222);

  const camera = new THREE.PerspectiveCamera(50, container.clientWidth/container.clientHeight, 0.1, 1000);
  camera.position.set(3,3,6);
  camera.lookAt(0,0,0);

  const light = new THREE.DirectionalLight(0xffffff,1);
  light.position.set(5,10,7);
  scene.add(light);

  const ambient = new THREE.AmbientLight(0x404040);
  scene.add(ambient);

  const yPlane = new THREE.GridHelper(20, 20, 0x666666, 0x333333);
  scene.add(yPlane);

  window.addEventListener('resize', ()=>{
    const w = container.clientWidth;
    const h = container.clientHeight;
    renderer.setSize(w,h);
    camera.aspect = w/h;
    camera.updateProjectionMatrix();
  });

  return {renderer,scene,camera};
}
