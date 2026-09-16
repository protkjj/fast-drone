/* Browser-only regression of the restored original app (not the research UI). */
const assert=require('node:assert/strict');
const {writeFileSync}=require('node:fs');
const {join}=require('node:path');
module.exports=async function({evaluate,send,sessionId,profile}){
  const wait=async expression=>{
    const begin=Date.now();
    while(Date.now()-begin<30000){if(await evaluate(expression))return;await new Promise(r=>setTimeout(r,100));}
    throw new Error('Timed out: '+expression);
  };
  const key=async(code,key,virtual)=>{
    await send('Input.dispatchKeyEvent',{type:'keyDown',code,key,windowsVirtualKeyCode:virtual},sessionId);
    await send('Input.dispatchKeyEvent',{type:'keyUp',code,key,windowsVirtualKeyCode:virtual},sessionId);
  };
  const space=()=>key('Space',' ',32),blur=()=>evaluate('document.activeElement.blur()');
  await wait('typeof veh!=="undefined" && veh?.userData.rotors?.length===4');
  const initial=await evaluate(`({title:document.title,ctrl:CTRL,speed:$('#spd').value,altitude:$('#alt').value,
    z:X[2],mass:P.mass,check:selfCheck(),lqrCheck:lqrCheck(),options:Object.keys(CTRLS)})`);
  assert.equal(initial.ctrl,'lqr');assert.equal(initial.speed,'60');assert.equal(initial.altitude,'200');
  assert.equal(initial.mass,8);assert.equal(initial.z,0);assert.ok(initial.check<1e-9&&initial.lqrCheck<1e-9);
  for(const controller of ['hybrid','sqprti','nmpc'])assert.ok(initial.options.includes(controller));
  console.log('ORIGINAL DEFAULTS '+JSON.stringify(initial));
  await blur();await space();await wait('T>.25 && running');
  await evaluate("$('#spd').focus()");await key('ArrowRight','ArrowRight',39);
  assert.equal(await evaluate("$('#spd').value"),'61');
  await space();assert.equal(await evaluate('running'),false);
  const frozen=await evaluate('({state:X.slice(),t:T,rotors:veh.userData.rotors.map(r=>r.rotation.x)})');
  await new Promise(r=>setTimeout(r,200));
  assert.deepEqual(await evaluate('({state:X.slice(),t:T,rotors:veh.userData.rotors.map(r=>r.rotation.x)})'),frozen);
  await evaluate("$('#n_spd').focus()");await space();assert.equal(await evaluate('running'),false);
  await evaluate("$('#go').focus()");await space();await wait('running && T>'+frozen.t);
  await blur();await space();assert.equal(await evaluate('running'),false);
  await key('KeyF','f',70);assert.equal(await evaluate('ORB.follow'),false);
  await key('KeyF','f',70);assert.equal(await evaluate('ORB.follow'),true);
  const time=await evaluate('T');
  await evaluate("$('#segbar').focus()");await key('Home','Home',36);assert.equal(await evaluate('T'),0);
  await key('End','End',35);assert.ok(await evaluate('T')>.1);assert.ok(await evaluate('T')<=time);
  await evaluate("$('#wsp').value='2';$('#wsp').dispatchEvent(new Event('input'));$('#c_C_A0').value=String(P.C_A0+.01);$('#c_C_A0').dispatchEvent(new Event('input'))");
  assert.equal(await evaluate('windNow()[1]'),2);assert.ok(await evaluate('P.C_A0>P0.C_A0'));
  await evaluate("$('#def').click();$('#wsp').value='0';$('#wsp').dispatchEvent(new Event('input'));$('#scoreBtn').click()");
  assert.equal(await evaluate("$('#score').hidden"),false);assert.ok(await evaluate('scoreCsv().length')>100);
  await evaluate("$('#scoreClose').click()");await blur();await key('KeyR','r',82);
  assert.equal(await evaluate('T'),0);assert.equal(await evaluate('running'),false);
  console.log('ORIGINAL INPUT: Space/range/number/button, frozen state and rotors, F/R, rewind, wind, coefficients and score passed');
  for(const controller of ['sqprti','hybrid','nmpc']){
    await evaluate(`$('#ctrl').value=${JSON.stringify(controller)};$('#ctrl').dispatchEvent(new Event('change'));$('#rst').click()`);
    await blur();await space();await wait('T>=.3 || !!physicsFailure');await blur();await space();
    const result=await evaluate('({controller:CTRL,t:T,failure:physicsFailure,finite:X.every(Number.isFinite),pose:veh.position.toArray(),state:X.slice(0,3)})');
    assert.equal(result.controller,controller);assert.equal(result.failure,'');assert.ok(result.finite&&result.t>=.3);
    assert.deepEqual(result.pose,result.state);console.log('ORIGINAL CONTROLLER '+JSON.stringify(result));
  }
  await evaluate("$('#rst').click()");
  for(const [width,height] of [[1280,900],[390,844]]){
    await send('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile:false},sessionId);
    await evaluate('window.scrollTo(0,0);resize3D();paint(diag())');
    assert.ok(await evaluate('document.documentElement.scrollWidth<=innerWidth'));
    const shot=await send('Page.captureScreenshot',{format:'png'},sessionId),file=join(profile,`original-${width}.png`);
    writeFileSync(file,Buffer.from(shot.data,'base64'));console.log('SCREENSHOT '+file);
  }
};
