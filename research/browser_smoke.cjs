/* Isolated headless Chrome smoke test. Never attaches to a user's browser/profile. */
const {spawn}=require('node:child_process');
const {mkdtempSync}=require('node:fs');
const {join}=require('node:path');
const {tmpdir}=require('node:os');

async function main(){
  const seconds=Number(process.argv[3]||'.2');
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
      for(const feedback of ['truth','eskf']){
        console.log('START browser hover both/'+feedback);
        const result=await evaluate(`(async()=>{
          document.getElementById('seconds').value=${JSON.stringify(String(seconds))};document.getElementById('controller').value='both';
          document.getElementById('feedback').value=${JSON.stringify(feedback)};
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
              solverFailures:r.metrics.solver_failures,counts:r.counts,seconds:r.simulated_seconds}]))};
        })()`);
        console.log(JSON.stringify({feedback,...result}));
        if(result.busy||result.error||Object.keys(result.reports).length!==2)throw new Error('Browser comparison failed');
        if(!result.animated||!result.paused)throw new Error('STL rotor animation/pause failed');
        if(Object.values(result.reports).some(r=>r.failure))throw new Error('Browser plant run failed');
        if(Object.values(result.reports).some(r=>r.solverFailures))throw new Error('Browser IPOPT solve failed');
      }
      if(errors.length)throw new Error(JSON.stringify(errors));
    } finally {await send('Browser.close');}
  } finally {socket?.close();console.log('QA profile retained at '+profile);}
}
main().catch(error=>{console.error(error);process.exitCode=1;});
