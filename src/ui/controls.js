export class Controls{
  constructor(){
    this.partSelect = document.getElementById('part-select');
    this.sceneSelect = document.getElementById('scene-select');
    this.integratorSelect = document.getElementById('integrator-select');
    this.speedRange = document.getElementById('speed-range');
    this.speedValue = document.getElementById('speed-value');
    this.dampingRange = document.getElementById('damping-range');
    this.dampingValue = document.getElementById('damping-value');
    this.speed = parseFloat(this.speedRange.value);
    this.damping = parseFloat(this.dampingRange.value);
    this._cb = ()=>{};
    this.partSelect.addEventListener('change', ()=>this._populateScenes());
    this.sceneSelect.addEventListener('change', ()=>this._onChange());
    this.integratorSelect.addEventListener('change', ()=>this._onChange());
    this.speedRange.addEventListener('input', ()=>{
      this.speed = parseFloat(this.speedRange.value);
      this.speedValue.textContent = `${this.speed.toFixed(1)}x`;
      this._onChange();
    });
    this.dampingRange.addEventListener('input', ()=>{
      this.damping = parseFloat(this.dampingRange.value);
      this.dampingValue.textContent = this.damping.toFixed(4);
      this._onChange();
    });
    this.resetSpeedButton = document.getElementById('reset-speed-button');
    this.resetDampingButton = document.getElementById('reset-damping-button');
    this.resetButton = document.getElementById('reset-button');
    this.resetSpeedButton.addEventListener('click', ()=>{
      this.speedRange.value = '1';
      this.speed = 1;
      this.speedValue.textContent = '1.0x';
      this._onChange();
    });
    this.resetDampingButton.addEventListener('click', ()=>{
      this.dampingRange.value = '0';
      this.damping = 0;
      this.dampingValue.textContent = '0.0000';
      this._onChange();
    });
    this.resetButton = document.getElementById('reset-button');
    this.resetButton.addEventListener('click', ()=>{
      if(this._resetCb) this._resetCb(this.partSelect.value, this.sceneSelect.value);
    });
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
    this._populateIntegrators();
    this._onChange();
  }

  setIntegrators(map){
    this._integrators = map;
    this._populateIntegrators();
  }

  _populateIntegrators(){
    if(!this.integratorSelect || !this._integrators) return;
    this.integratorSelect.innerHTML = '';
    for(const key of Object.keys(this._integrators)){
      const opt = document.createElement('option'); opt.value=key; opt.textContent=this._integrators[key];
      this.integratorSelect.appendChild(opt);
    }
  }

  onChange(cb){ this._cb = cb; }

  onReset(cb){ this._resetCb = cb; }

  _onChange(){ this._cb(this.partSelect.value, this.sceneSelect.value, this.integratorSelect.value, this.speed, this.damping); }

  setStatsElements(statElems){
    if(!statElems){
      document.getElementById('L0').textContent='—';
      document.getElementById('Lcur').textContent='—';
      document.getElementById('energy').textContent='—';
      const springLen = document.getElementById('springLen');
      const springForce = document.getElementById('springForce');
      const springEnergy = document.getElementById('springEnergy');
      const totalEnergy = document.getElementById('totalEnergy');
      if(springLen) springLen.textContent = '—';
      if(springForce) springForce.textContent = '—';
      if(springEnergy) springEnergy.textContent = '—';
      if(totalEnergy) totalEnergy.textContent = '—';
      return;
    }
  }
}
