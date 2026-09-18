/* Real browser interaction/pose regression, invoked by browser_smoke --flight. */
const assert=require('node:assert/strict');
const {writeFileSync}=require('node:fs');
const {join}=require('node:path');
module.exports=async function({evaluate,send,sessionId,profile}){
  const wait=async expression=>{
    const begin=Date.now();
    while(Date.now()-begin<90000){if(await evaluate(expression))return;await new Promise(r=>setTimeout(r,100));}
    throw new Error('Timed out: '+expression);
  };
  await evaluate("$('mode-observe').click()");
  assert.equal(await evaluate("$('observe-controller').value"),'hybrid');
  await evaluate("$('speed-slider').focus()");
  await send('Input.dispatchKeyEvent',{type:'keyDown',key:'ArrowRight',code:'ArrowRight',windowsVirtualKeyCode:39},sessionId);
  await send('Input.dispatchKeyEvent',{type:'keyUp',key:'ArrowRight',code:'ArrowRight',windowsVirtualKeyCode:39},sessionId);
  await wait("$('live-target-speed').value==='0.1'");
  await evaluate("$('hover-target').click()");
  for(const controller of ['hybrid','nmpc']){
    await evaluate(`$('observe-controller').value=${JSON.stringify(controller)};$('observe-controller').dispatchEvent(new Event('change'));
      $('seconds').value='.08';$('run').click();
      window.flightOriginalHandler=worker.onmessage;window.flightInputQueued=false;
      worker.onmessage=event=>{
        window.flightOriginalHandler(event);
        // Respond to a delivered frame, not a wall-time polling guess: a fast
        // host can finish an 80 ms flight between two CDP polls.
        if(event.data.type==='progress'&&event.data.t>0&&!window.flightInputQueued){
          window.flightInputQueued=true;
          $('speed-slider').value='.2';$('speed-slider').dispatchEvent(new Event('input'));
          $('speed-slider').dispatchEvent(new Event('change'));$('live-wind-speed').value='.2';
          $('live-wind-speed').dispatchEvent(new Event('change'));
        }
      };`);
    await wait('!running');
    await evaluate('worker.onmessage=window.flightOriginalHandler');
    const result=await evaluate(`(()=>{const r=results[${JSON.stringify(controller)}];return {controller:r?.configuration.controller,
      counts:r?.counts,failure:r?.failure,errors:$('errors').textContent,commands:r?.configuration.commands,
      pose:vehicle.position.toArray(),final:r?.final.slice(0,3),solves:r?.metrics.solver_failures};})()`);
    assert.equal(result.controller,controller);assert.equal(result.counts.nmpc,4);
    assert.equal(result.counts.indi,controller==='hybrid'?80:0);assert.equal(result.errors,'');assert.equal(result.failure,null);
    assert.ok(result.commands.some(c=>c.t>0&&c.speed===.2&&c.wind_speed===.2));assert.deepEqual(result.pose,result.final);
    console.log('FLIGHT '+JSON.stringify(result));
    await evaluate("$('live-target-speed').value='0';$('wind-calm').click()");
  }
  await evaluate("$('view-mode').value='world';$('view-mode').dispatchEvent(new Event('change'));$('camera-reset').click();$('view-mode').value='follow';$('view-mode').dispatchEvent(new Event('change'))");
  // More than an initialization smoke: the default Hybrid actually travels.
  await evaluate("$('observe-controller').value='hybrid';$('observe-controller').dispatchEvent(new Event('change'));$('seconds').value='2';$('live-target-speed').value='1';$('apply-target').click();$('run').click()");
  await wait('!running');
  const travel=await evaluate('({x:results.hybrid.final[0],v:results.hybrid.final[3],status:results.hybrid.status,solves:results.hybrid.counts.nmpc,failures:results.hybrid.metrics.solver_failures,pose:vehicle.position.toArray(),final:results.hybrid.final.slice(0,3)})');
  assert.equal(travel.status,'completed');assert.equal(travel.solves,100);assert.ok(travel.x>.1);assert.equal(travel.failures,0);assert.deepEqual(travel.pose,travel.final);
  console.log('HYBRID TRAVEL '+JSON.stringify(travel));
  await evaluate("$('hover-target').click()");
  await evaluate("$('observe-controller').value='pd';$('observe-controller').dispatchEvent(new Event('change'));$('seconds').value='8';$('run').click()");
  await wait("running && Number($('progress').value)>.03");
  await evaluate("$('speed-slider').value='3';$('speed-slider').dispatchEvent(new Event('input'));$('speed-slider').dispatchEvent(new Event('change'));$('live-target-altitude').value='21';$('live-target-altitude').dispatchEvent(new Event('change'));$('live-wind-speed').value='2';$('live-wind-speed').dispatchEvent(new Event('change'))");
  await wait("$('target-state').textContent.includes('기록됨') && vehicle.position.x>.1");
  await evaluate("$('pause').click()");await wait('paused');
  const frozen=await evaluate('({p:vehicle.position.toArray(),q:vehicle.quaternion.toArray(),rotors:vehicle.userData.rotors.map(r=>r.rotation.x),hud:$("flight-position").textContent})');
  await new Promise(r=>setTimeout(r,250));
  assert.deepEqual(await evaluate('({p:vehicle.position.toArray(),q:vehicle.quaternion.toArray(),rotors:vehicle.userData.rotors.map(r=>r.rotation.x),hud:$("flight-position").textContent})'),frozen);
  await evaluate("$('live-wind-angle').value='180';$('live-wind-angle').dispatchEvent(new Event('change'));$('pause').click()");
  await wait("!paused && $('target-state').textContent.includes('180°')");
  await evaluate("$('stop').click()");await wait('!running');
  const pd=await evaluate('({commands:results.pd.configuration.commands,status:results.pd.status,seconds:results.pd.simulated_seconds,final:results.pd.final,pose:vehicle.position.toArray(),error:$("errors").textContent})');
  assert.equal(pd.status,'stopped');assert.equal(pd.error,'');assert.ok(pd.final[0]>.1);
  assert.ok(pd.commands.some(c=>c.wind_angle===180));assert.deepEqual(pd.pose,pd.final.slice(0,3));
  console.log('PAUSE/STOP '+JSON.stringify(pd));
  const replay=await evaluate(`(()=>{const before=JSON.stringify(results);$('record-panel').open=true;
    $('timeline').value=String(results.pd.simulated_seconds/2);$('timeline').dispatchEvent(new Event('input'));
    return {pose:vehicle.position.toArray(),expected:ResearchResults.sample(results.pd.trace,replayTime).state.slice(0,3),
      immutable:before===JSON.stringify(results)};})()`);
  assert.deepEqual(replay.pose,replay.expected);assert.ok(replay.immutable);
  await evaluate("$('play').click()");await wait('!playing');
  await evaluate("$('compare-record').click()");
  assert.ok(await evaluate("mode==='compare' && recordedCommands.some(c=>c.wind_angle===180) && $('preview').value==='false'"));
  await evaluate("$('mode-observe').click();$('record-panel').open=false;window.scrollTo(0,0)");
  for(const [width,height] of [[1280,900],[390,844]]){
    await send('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile:false},sessionId);
    const layout=await evaluate(`(()=>{drawModel();return {width:innerWidth,overflow:document.documentElement.scrollWidth>innerWidth,
      model:$("model").getBoundingClientRect().toJSON(),controls:$("live-controls").getBoundingClientRect().toJSON()};})()`);
    assert.equal(layout.overflow,false);assert.ok(layout.model.top<height);
    if(width<850)assert.ok(layout.model.top<layout.controls.top);
    const shot=await send('Page.captureScreenshot',{format:'png'},sessionId);
    const file=join(profile,`flight-${width}.png`);writeFileSync(file,Buffer.from(shot.data,'base64'));
    console.log('LAYOUT '+JSON.stringify({...layout,screenshot:file}));
  }
};
