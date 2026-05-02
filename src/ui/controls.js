export class Controls{
  constructor(){
    this.partSelect = document.getElementById('part-select');
    this.sceneSelect = document.getElementById('scene-select');
    this._cb = ()=>{};
    this.partSelect.addEventListener('change', ()=>this._onChange());
    this.sceneSelect.addEventListener('change', ()=>this._onChange());
  }

  setParts(map){
    this._parts = map;
    this.partSelect.innerHTML = '';
    for(const key of Object.keys(map)){
      const opt = document.createElement('option'); opt.value=key; opt.textContent=map[key].label;
      this.partSelect.appendChild(opt);
    }
    this._populateScenes();
  }

  _populateScenes(){
    const part = this.partSelect.value;
    const scenes = this._parts[part].scenes || {};
    this.sceneSelect.innerHTML = '';
    for(const k of Object.keys(scenes)){
      const opt = document.createElement('option'); opt.value=k; opt.textContent=scenes[k];
      this.sceneSelect.appendChild(opt);
    }
    this._onChange();
  }

  onChange(cb){ this._cb = cb; }

  _onChange(){ this._cb(this.partSelect.value, this.sceneSelect.value); }

  setStatsElements(statElems){
    if(!statElems){
      document.getElementById('L0').textContent='—';
      document.getElementById('Lcur').textContent='—';
      document.getElementById('energy').textContent='—';
      return;
    }
  }
}
