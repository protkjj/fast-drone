/* Classic-layout regression with real Chrome keyboard and pointer input. */
const assert=require('node:assert/strict');
const {writeFileSync}=require('node:fs');
const {join}=require('node:path');
module.exports=async function({evaluate,send,sessionId,profile}){
  const wait=async expression=>{
    const begin=Date.now();
    while(Date.now()-begin<90000){if(await evaluate(expression))return;await new Promise(r=>setTimeout(r,100));}
    throw new Error('Timed out: '+expression);
  };
  const key=async (code,key,virtual,extra={})=>{
    await send('Input.dispatchKeyEvent',{type:'keyDown',code,key,windowsVirtualKeyCode:virtual,...extra},sessionId);
    await send('Input.dispatchKeyEvent',{type:'keyUp',code,key,windowsVirtualKeyCode:virtual},sessionId);
  };
  const space=()=>key('Space',' ',32);
  const shot=async name=>{
    const image=await send('Page.captureScreenshot',{format:'png'},sessionId),file=join(profile,name+'.png');
    writeFileSync(file,Buffer.from(image.data,'base64'));console.log('SCREENSHOT '+file);
  };
  await send('Emulation.setDeviceMetricsOverride',{width:1280,height:900,deviceScaleFactor:1,mobile:false},sessionId);
  await evaluate('drawModel()');await shot('classic-initial');
  assert.equal(await evaluate("$('observe-controller').value"),'hybrid');
  await evaluate("$('seconds').value='.08';$('flight-view').focus()");await space();await wait('!running && !!results.hybrid');
  assert.equal(await evaluate('results.hybrid.counts.nmpc'),4);
  assert.equal(await evaluate('results.hybrid.metrics.solver_failures'),0);
  console.log('DEFAULT HYBRID: Space starts real IPOPT, 4 successful solves');
  await evaluate('window.savedClassicLog=JSON.stringify(results)');
  await key('KeyR','r',82);
  assert.ok(await evaluate('!replayActive && replayTime===0 && JSON.stringify(results)===window.savedClassicLog'));
  assert.deepEqual(await evaluate('vehicle.position.toArray()'),[0,0,20]);
  // PD is explicitly selected only to test long interactive UI sequences cheaply.
  await evaluate("$('observe-controller').value='pd';$('observe-controller').dispatchEvent(new Event('change'));$('seconds').value='8';$('flight-view').focus()");
  await space();await wait("running && Number($('progress').value)>.03");
  await evaluate("$('speed-slider').focus()");await key('ArrowRight','ArrowRight',39);
  assert.equal(await evaluate("$('live-target-speed').value"),'0.1');
  const slider=await evaluate("$('speed-slider').getBoundingClientRect().toJSON()"),sy=slider.y+slider.height/2;
  await send('Input.dispatchMouseEvent',{type:'mousePressed',x:slider.x+8,y:sy,button:'left',buttons:1,clickCount:1},sessionId);
  await send('Input.dispatchMouseEvent',{type:'mouseMoved',x:slider.x+8+(slider.width-16)*.025,y:sy,button:'left',buttons:1},sessionId);
  await send('Input.dispatchMouseEvent',{type:'mouseReleased',x:slider.x+8+(slider.width-16)*.025,y:sy,button:'left',buttons:0,clickCount:1},sessionId);
  assert.ok(await evaluate("Number($('live-target-speed').value)>.1"));
  await wait("$('target-state').textContent.includes('기록됨') && $('target-state').textContent.includes('목표 '+$('live-target-speed').value+' m/s')");
  await space();await wait('paused');
  const frozen=await evaluate('({p:vehicle.position.toArray(),q:vehicle.quaternion.toArray(),rotors:vehicle.userData.rotors.map(r=>r.rotation.x),hud:$("flight-position").textContent})');
  await new Promise(r=>setTimeout(r,250));
  assert.deepEqual(await evaluate('({p:vehicle.position.toArray(),q:vehicle.quaternion.toArray(),rotors:vehicle.userData.rotors.map(r=>r.rotation.x),hud:$("flight-position").textContent})'),frozen);
  assert.ok(await evaluate("$('velocity-chart').querySelector('path') && $('altitude-chart').querySelector('path')"));
  await evaluate("$('live-target-speed').focus()");await space();assert.equal(await evaluate('paused'),true);
  await evaluate("$('flight-view').focus()");await key('Space',' ',32,{autoRepeat:true});assert.equal(await evaluate('paused'),true);
  await evaluate("$('go').focus()");await space();await wait('!paused'); // Native button activation, once.
  await evaluate("$('flight-view').focus()");await space();await wait('paused');
  await key('KeyF','f',70);assert.equal(await evaluate("$('follow').getAttribute('aria-pressed')"),'false');
  await key('KeyF','f',70);assert.equal(await evaluate("$('follow').getAttribute('aria-pressed')"),'true');
  const camera=await evaluate(`(()=>{const pose=vehicle.position.toArray(),q=vehicle.quaternion.toArray(),views=[];
    for(const id of ['camera-back','camera-side','camera-top','camera-nose']){$(id).click();views.push($('model').querySelector('canvas').toDataURL());}
    $('camera-reset').click();return {distinct:new Set(views).size,unchanged:JSON.stringify([pose,q])===JSON.stringify([vehicle.position.toArray(),vehicle.quaternion.toArray()])};})()`);
  assert.equal(camera.distinct,4);assert.ok(camera.unchanged);
  const beforeWidth=await evaluate("document.querySelector('.left').getBoundingClientRect().width");
  await evaluate("$('grip-left').focus()");await key('ArrowRight','ArrowRight',39);
  assert.equal(await evaluate("document.querySelector('.left').getBoundingClientRect().width"),beforeWidth+10);
  const rect=await evaluate("$('model').getBoundingClientRect().toJSON()"),x=Math.round(rect.x+rect.width*.5),y=Math.round(rect.y+rect.height*.55);
  const beforePan=await evaluate("$('model').querySelector('canvas').toDataURL()");
  await send('Input.dispatchMouseEvent',{type:'mousePressed',x,y,button:'left',buttons:1,clickCount:1,modifiers:8},sessionId);
  await send('Input.dispatchMouseEvent',{type:'mouseMoved',x:x+35,y:y+15,button:'left',buttons:1,modifiers:8},sessionId);
  await send('Input.dispatchMouseEvent',{type:'mouseReleased',x:x+35,y:y+15,button:'left',buttons:0,clickCount:1,modifiers:8},sessionId);
  assert.notEqual(await evaluate("$('model').querySelector('canvas').toDataURL()"),beforePan);
  await shot('classic-paused');
  await evaluate("$('flight-view').focus()");await key('KeyR','r',82);await wait('!running && !resetRequested');
  assert.ok(await evaluate("results.pd.status==='stopped' && !replayActive"));
  assert.deepEqual(await evaluate('vehicle.position.toArray()'),[0,0,20]);
  await evaluate("$('timeline').focus()");await key('End','End',35);
  assert.equal(await evaluate('replayTime'),await evaluate('results.pd.simulated_seconds'));
  assert.deepEqual(await evaluate('vehicle.position.toArray()'),await evaluate('results.pd.final.slice(0,3)'));
  await space();await wait('playing');await space();assert.equal(await evaluate('playing'),false);
  console.log('CLASSIC INPUT: slider-focus Space, frozen state/rotors, numeric input, native button, F, R, pan, panel resize, live charts and replay passed');
  for(const [width,height] of [[1280,900],[390,844]]){
    await send('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile:false},sessionId);
    await evaluate('window.scrollTo(0,0);drawModel()');
    const layout=await evaluate(`(()=>{const rect=id=>$(id).getBoundingClientRect().toJSON();return {
      overflow:document.documentElement.scrollWidth>innerWidth,view:rect('flight-view'),controls:rect('live-controls'),
      timeline:rect('timeline'),chart:rect('velocity-chart'),left:document.querySelector('.left').getBoundingClientRect().toJSON()};})()`);
    assert.equal(layout.overflow,false);assert.ok(layout.view.width>300);
    if(width>900){assert.ok(layout.view.x>=layout.left.right);assert.ok(layout.chart.bottom<=height);}
    else assert.ok(layout.view.top<layout.controls.top);
    console.log('CLASSIC LAYOUT '+JSON.stringify({width,...layout}));await shot('classic-'+width);
  }
};
