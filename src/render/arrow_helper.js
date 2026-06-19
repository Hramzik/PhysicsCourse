export function createArrow(scene, color, headScale){
  const dir = new THREE.Vector3(1,0,0);
  const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0,0,0), 1, color);
  arrow.headScale = headScale
  scene.add(arrow);
  return arrow;
}

export function updateArrow(arrow, vector, origin){
  arrow.setDirection(vector.clone().normalize());

  arrow.setLength(getArrowLenght(vector), getArrowHeadSize(arrow, vector), getArrowHeadSize(arrow, vector));
  arrow.position.copy(origin);
}

function getArrowLenght(vector){
  return Math.min(4, vector.length() * 100);
}

function getArrowHeadSize(arrow, vector){
  return Math.max(0.1, vector.length()*0.2) * arrow.headScale
}