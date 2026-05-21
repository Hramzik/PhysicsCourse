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
    this._resetCb = null;
    // _integratorsMap: { partKey: { integratorKey: label, … }, … }
    // or flat { integratorKey: label } for backward compat
    this._integratorsMap = {};
    // When true, _onChange() is a no-op (used during programmatic UI sync)
    this._silent = false;

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
    this.resetButton.addEventListener('click', ()=>{
      if(this._resetCb) this._resetCb(this.partSelect.value, this.sceneSelect.value, this.integratorSelect.value);
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

  /**
   * Programmatically set part/scene/integrator without triggering the onChange callback.
   * Repopulates integrator list for the given part, then sets all three values.
   */
  setSelection(part, scene, integrator){
    this._silent = true;
    try{
      // Update part
      this.partSelect.value = part;
      // Repopulate scenes for this part
      const scenes = this._parts[part].scenes || {};
      this.sceneSelect.innerHTML = '';
      for(const k of Object.keys(scenes)){
        const opt = document.createElement('option'); opt.value=k; opt.textContent=scenes[k];
        this.sceneSelect.appendChild(opt);
      }
      this.sceneSelect.value = scene;
      // Repopulate integrators for this part
      this._populateIntegrators();
      this.integratorSelect.value = integrator;
      // Update visibility
      this._updateIntegratorVisibility(part);
    } finally {
      this._silent = false;
    }
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
    this._updateIntegratorVisibility(part);
    this._onChange();
  }

  _updateIntegratorVisibility(part){
    const integratorRow = document.getElementById('integrator-row');
    if(integratorRow){
      // Show integrator selector for all parts that have integrators defined
      const map = this._getIntegratorsForPart(part);
      integratorRow.style.display = (Object.keys(map).length > 0) ? 'block' : 'none';
    }
  }

  /**
   * Set integrators. Accepts either:
   *   - flat map { key: label }  — used for all parts (backward compat)
   *   - per-part map { part1: { key: label }, part2: { key: label } }
   */
  setIntegrators(map){
    this._integratorsMap = map;
    this._populateIntegrators();
  }

  _getIntegratorsForPart(part){
    if(!this._integratorsMap) return {};
    // Check if it's a per-part map (values are objects, not strings)
    const firstVal = Object.values(this._integratorsMap)[0];
    if(firstVal && typeof firstVal === 'object'){
      return this._integratorsMap[part] || {};
    }
    // Flat map — return as-is
    return this._integratorsMap;
  }

  _populateIntegrators(){
    if(!this.integratorSelect || !this._integratorsMap) return;
    const part = this.partSelect ? this.partSelect.value : null;
    const map = part ? this._getIntegratorsForPart(part) : this._integratorsMap;
    this.integratorSelect.innerHTML = '';
    for(const key of Object.keys(map)){
      const opt = document.createElement('option'); opt.value=key; opt.textContent=map[key];
      this.integratorSelect.appendChild(opt);
    }
  }

  onChange(cb){ this._cb = cb; }

  onReset(cb){ this._resetCb = cb; }

  _onChange(){
    if(this._silent) return;
    this._cb(this.partSelect.value, this.sceneSelect.value, this.integratorSelect.value, this.speed, this.damping);
  }

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
