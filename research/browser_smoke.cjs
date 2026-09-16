/* Isolated headless Chrome smoke test. Never attaches to a user's browser/profile. */
const {spawn}=require('node:child_process');
const {mkdtempSync,existsSync,readFileSync}=require('node:fs');
const {join}=require('node:path');
const {tmpdir}=require('node:os');

async function main(){
  const seconds=Number(process.argv[3]||'.2');
  const uiOnly=process.argv.includes('--ui-only');
  if(!Number.isFinite(seconds)||seconds<.02||seconds>2)throw new Error('Smoke-test duration must be 0.02–2 s');
  const executable=process.env.RESEARCH_CHROME||'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
  const profile=mkdtempSync(join(tmpdir(),'drone-browser-qa-'));
  const child=spawn(executable,['--headless=new','--no-first-run','--no-default-browser-check',
    '--remote-debugging-port=0','--user-data-dir='+profile,'about:blank'],{stdio:['ignore','ignore','pipe']});
  let socket;
  try {
    const endpoint=await new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>reject(new Error('Chrome startup timed out')),15000);
      child.once('error',error=>{clearTimeout(timer);reject(error);});
      child.stderr.on('data',bytes=>{const match=bytes.toString().match(/DevTools listening on (ws:\/\/\S+)/);
        if(match){clearTimeout(timer);resolve(match[1]);}});
    });
    socket=new WebSocket(endpoint);
    await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
    let serial=0;const pending=new Map(),errors=[];
    socket.onmessage=event=>{
      const message=JSON.parse(event.data);
      if(message.id&&pending.has(message.id)){
        const p=pending.get(message.id);pending.delete(message.id);clearTimeout(p.timer);
        if(message.error)p.reject(new Error(JSON.stringify(message.error)));else p.resolve(message.result);
      } else if(message.method==='Runtime.exceptionThrown') errors.push(message.params.exceptionDetails);
    };
    const send=(method,params={},sessionId)=>new Promise((resolve,reject)=>{
      const id=++serial,timer=setTimeout(()=>{pending.delete(id);reject(new Error(method+' timed out'));},120000);
      pending.set(id,{resolve,reject,timer});socket.send(JSON.stringify({id,method,params,sessionId}));
    });
    try {
      const {targetId}=await send('Target.createTarget',{url:process.argv[2]||'https://protkjj.github.io/fast-drone/research/'});
      const {sessionId}=await send('Target.attachToTarget',{targetId,flatten:true});
      await send('Runtime.enable',{},sessionId);
      const evaluate=async expression=>{
        const output=await send('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true},sessionId);
        if(output.exceptionDetails)throw new Error(JSON.stringify(output.exceptionDetails));
        return output.result.value;
      };
      for(let i=0;i<100;i++){
        if(await evaluate('!!document.getElementById("run") && typeof vehicle!=="undefined" && !!vehicle'))break;
        await new Promise(resolve=>setTimeout(resolve,100));
      }
      if(await evaluate('vehicle?.userData.rotors.length')!==4)throw new Error('STL rotor assembly did not load');
      if(uiOnly){
        const file=process.argv[process.argv.indexOf('--log')+1];
        if(!process.argv.includes('--log')||!existsSync(file))throw new Error('--ui-only requires an existing --log file');
        const {root:doc}=await send('DOM.getDocument',{},sessionId);
        const {nodeId}=await send('DOM.querySelector',{nodeId:doc.nodeId,selector:'#import-file'},sessionId);
        await send('DOM.setFileInputFiles',{nodeId,files:[file]},sessionId);
        for(let i=0;i<100;i++){
          if(await evaluate('imported'))break;
          await new Promise(resolve=>setTimeout(resolve,100));
        }
        if(!await evaluate('imported'))throw new Error('UI fixture import failed');
      }
      if(!uiOnly)for(const feedback of ['truth','eskf']){
        console.log('START browser hover both/'+feedback);
        const result=await evaluate(`(async()=>{
          document.getElementById('seconds').value=${JSON.stringify(String(seconds))};document.getElementById('controller').value='both';
          document.getElementById('feedback').value=${JSON.stringify(feedback)};
          document.getElementById('log-hz').value=${JSON.stringify(feedback==='eskf'?'1000':'50')};
          document.getElementById('altitude').value=${JSON.stringify(feedback==='eskf'?'33':'20')};
          const phases=vehicle.userData.rotors.map(r=>r.rotation.x);let animated=false;
          document.getElementById('run').click();
          const begin=Date.now();while(document.getElementById('run').disabled&&Date.now()-begin<90000){
            await new Promise(resolve=>setTimeout(resolve,200));
            animated ||= vehicle.userData.rotors.every((r,i)=>r.rotation.x!==phases[i]);
          }
          const stopped=vehicle.userData.rotors.map(r=>r.rotation.x);
          await new Promise(resolve=>setTimeout(resolve,120));
          return {busy:document.getElementById('run').disabled,error:document.getElementById('errors').textContent,
            animated,paused:vehicle.userData.rotors.every((r,i)=>r.rotation.x===stopped[i]&&!r.userData.blur.visible),
            stl:document.getElementById('model-caption').textContent,
            reports:Object.fromEntries(Object.entries(results).map(([k,r])=>[k,{failure:r.failure,
              solverFailures:r.metrics.solver_failures,counts:r.counts,seconds:r.simulated_seconds,
              firstAltitude:r.trace[0].state[2],frames:r.trace.length,logHz:r.configuration.log_hz}]))};
        })()`);
        console.log(JSON.stringify({feedback,...result}));
        if(result.busy||result.error||Object.keys(result.reports).length!==2)throw new Error('Browser comparison failed');
        if(!result.animated||!result.paused)throw new Error('STL rotor animation/pause failed');
        if(Object.values(result.reports).some(r=>r.failure))throw new Error('Browser plant run failed');
        if(Object.values(result.reports).some(r=>r.solverFailures))throw new Error('Browser IPOPT solve failed');
        if(Object.values(result.reports).some(r=>r.firstAltitude!==(feedback==='eskf'?33:20)||r.frames!==Math.ceil(seconds*r.logHz)+1))throw new Error('Altitude/log-rate settings were not applied');
      }
      if(process.argv.includes('--extended')){
        // Exercise real download/upload controls in the isolated profile only.
        await send('Browser.setDownloadBehavior',{behavior:'allow',downloadPath:profile});
        await evaluate("document.getElementById('download').click()");
        const downloaded=join(profile,'drone-research-results.json');
        for(let i=0;i<100&&!existsSync(downloaded);i++)await new Promise(resolve=>setTimeout(resolve,100));
        if(!existsSync(downloaded))throw new Error('JSON download did not complete');
        const saved=JSON.parse(readFileSync(downloaded,'utf8'));
        if(saved.results.hybrid.schema_version!==2)throw new Error('Not a v2 export');
        const validation=await evaluate(`(()=>{
          $('scenario').value='gust';$('scenario').dispatchEvent(new Event('change'));
          const adjusted=Number($('seconds').value);$('seconds').value='.2';$('run').click();
          return {adjusted,error:$('errors').textContent,busy:running};
        })()`);
        if(validation.adjusted<3.1||validation.busy||!validation.error.includes('3.1'))throw new Error('Scenario validation failed');
        const {root:doc}=await send('DOM.getDocument',{},sessionId);
        const {nodeId}=await send('DOM.querySelector',{nodeId:doc.nodeId,selector:'#import-file'},sessionId);
        await send('DOM.setFileInputFiles',{nodeId,files:[downloaded]},sessionId);
        for(let i=0;i<100;i++){
          if(await evaluate("imported && !$('errors').textContent"))break;
          await new Promise(resolve=>setTimeout(resolve,100));
        }
        const playback=await evaluate(`(async()=>{
          const before=JSON.stringify(results);$('replay-controller').value='nmpc';$('replay-controller').dispatchEvent(new Event('change'));
          $('timeline').value=String(results.nmpc.simulated_seconds/2);$('timeline').dispatchEvent(new Event('input'));
          const expected=ResearchResults.sample(results.nmpc.trace,replayTime).state;
          const pose=vehicle.position.toArray().every((v,i)=>Math.abs(v-expected[i])<1e-10);
          $('reuse').click();const reused=$('profile').value==='selected'&&$('feedback').value==='eskf'&&$('scenario').value==='hover'&&
            Number($('altitude').value)===results.nmpc.configuration.altitude&&Number($('log-hz').value)===results.nmpc.configuration.log_hz;
          $('play').click();const begin=performance.now();
          while(playing&&performance.now()-begin<5000)await new Promise(resolve=>setTimeout(resolve,20));
          return {imported,pose,reused,ended:!playing&&replayTime===results.nmpc.simulated_seconds,
            immutable:before===JSON.stringify(results),error:$('errors').textContent};
        })()`);
        console.log('REPLAY '+JSON.stringify(playback));
        if(!playback.imported||!playback.pose||!playback.reused||!playback.ended||!playback.immutable||playback.error)throw new Error('Replay/import/reuse failed');
        if(!uiOnly){const stopped=await evaluate(`(async()=>{
          $('controller').value='nmpc';$('seconds').value='2';$('scenario').value='hover';$('run').click();
          let begin=Date.now();while(running&&!/NMPC [1-9][0-9]*회/.test($('status').textContent)&&Date.now()-begin<45000)
            await new Promise(resolve=>setTimeout(resolve,25));
          $('stop').click();begin=Date.now();while(running&&Date.now()-begin<45000)await new Promise(resolve=>setTimeout(resolve,50));
          const r=results.nmpc;return {busy:running,error:$('errors').textContent,status:r?.status,
            seconds:r?.simulated_seconds,solves:r?.counts.nmpc,frames:r?.trace.length,download:!$('download').disabled};
        })()`);
        console.log('STOP '+JSON.stringify(stopped));
        if(stopped.busy||stopped.error||stopped.status!=='stopped'||stopped.seconds>=2||!stopped.frames||!stopped.download)throw new Error('Solve-boundary stop failed');
        }
        for(const [width,height] of [[390,844],[1280,900]]){
          await send('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile:false},sessionId);
          const layout=await evaluate(`(()=>{if(innerWidth<850)$('settings').open=false;
            return {width:innerWidth,overflow:document.documentElement.scrollWidth>innerWidth,modelTop:$('model').getBoundingClientRect().top,
              overflowing:[...document.querySelectorAll('body *')].filter(e=>e.getBoundingClientRect().right>innerWidth+1).map(e=>e.id||e.tagName).slice(0,15)};})()`);
          console.log('LAYOUT '+JSON.stringify(layout));if(layout.overflow)throw new Error('Horizontal overflow');
        }
      }
      if(errors.length)throw new Error(JSON.stringify(errors));
    } finally {await send('Browser.close');}
  } finally {socket?.close();console.log('QA profile retained at '+profile);}
}
main().catch(error=>{console.error(error);process.exitCode=1;});
