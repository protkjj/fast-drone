/* IPOPT runs here, never on the rendering/input thread. */
importScripts('./runtime.js');
let stopping=false,busy=false,paused=false,observing=false,pendingTarget=null,ca=null,ipoptLoaded=false;
const loaded=new Map();
async function loadCasadi() {
  const base=new URL('./vendor/casadi/',self.location.href).href;
  const originalFetch=self.fetch.bind(self);
  self.fetch=(input,options)=>originalFetch(typeof input==='string'&&/^libcasadi_[\w]+\.so$/.test(input)?base+input:input,options);
  // Adapted from the official package's MIT-0 _casadi_browser.js example.
  async function commonJS(file, requireFn) {
    const response=await fetch(base+file);
    if(!response.ok) throw new Error(`${file}: HTTP ${response.status}`);
    const module={exports:{}};
    const evaluate=new Function('module','exports','require','__dirname','__filename',
                                await response.text()+'\nreturn module.exports;');
    return evaluate(module,module.exports,requireFn,base.slice(0,-1),base+file);
  }
  const wasm=await commonJS('casadi_wasm.js',name=>{throw new Error('Unexpected dependency '+name);});
  const create=await commonJS('casadi.js',name=>{
    if(name.endsWith('casadi_wasm.js')) return wasm;
    if(name==='path') return {join:(directory,file)=>new URL(file,directory+'/').href};
    throw new Error('Unexpected dependency '+name);
  });
  const result=await create();
  await result.load_interpolant('linear');
  return result;
}
async function loadProfile(name) {
  if(!['simple','selected'].includes(name)) throw new Error('Unknown aircraft');
  if(!loaded.has(name)) {
    const response=await fetch(`./generated/${name}.json.gz`);
    if(!response.ok) throw new Error(`Model bundle: HTTP ${response.status}`);
    const stream=response.body.pipeThrough(new DecompressionStream('gzip'));
    loaded.set(name,await new Response(stream).json());
  }
  return loaded.get(name);
}
self.onmessage=async event=>{
  if(event.data.type==='stop') {stopping=true;return;}
  if(event.data.type==='pause'&&busy&&observing){paused=!!event.data.paused;return;}
  if(event.data.type==='target'&&busy&&observing){
    try{
      const command={t:0,speed:event.data.speed,altitude:event.data.altitude};
      ResearchRuntime.validateOptions({scenario:'schedule',commands:[command]});
      pendingTarget={speed:command.speed,altitude:command.altitude};
    }catch(error){postMessage({type:'command-error',text:error.message});}
    return;
  }
  if(event.data.type!=='run'||busy) return;
  busy=true;stopping=false;paused=false;pendingTarget=null;observing=event.data.mode==='observe';
  try {
    if(event.data.mode&&!['observe','compare'].includes(event.data.mode))throw new Error('Unknown execution mode');
    const controllers=observing?['pd']:event.data.controllers;
    if(!Array.isArray(controllers)||!controllers.length||controllers.length>2||
      controllers.some(c=>!['hybrid','nmpc',...(observing?['pd']:[])].includes(c)))throw new Error('Invalid controllers');
    postMessage({type:'status',text:'선정 기체의 공통 물리 계산 준비 중… 첫 실행에는 시간이 걸립니다.'});
    if(!ca) ca=await loadCasadi();
    if(!observing&&!ipoptLoaded){await ca.load_nlpsol('ipopt');ipoptLoaded=true;}
    const data=await loadProfile(event.data.profile);
    postMessage({type:'status',text:observing?'관찰 시작 · PD–INDI · 물리 적분 1 ms 유지':'정밀 비교 중. IPOPT 풀이를 기다린 뒤 시뮬레이션 시간이 진행됩니다.'});
    const results=[];
    for(const controller of controllers) {
      if(stopping) break;
      postMessage({type:'progress',controller,t:0,total:event.data.options.seconds,solves:0,
        v:data.initial.slice(3,6),z:event.data.options.altitude??20,p:[0,0,event.data.options.altitude??20],
        q:data.initial.slice(6,10),rpm:data.initial.slice(13,17)});
      const clock=new ResearchRuntime.ObservationClock();
      const hooks=observing?{
        beforeControl:async t=>{
          await clock.wait(t,()=>stopping||paused);
          if(paused){
            postMessage({type:'paused',paused:true,t});
            while(paused&&!stopping)await new Promise(resolve=>setTimeout(resolve,20));
            clock.reset(t);postMessage({type:'paused',paused:false,t});
          }
        },
        command:()=>{const command=pendingTarget;pendingTarget=null;return command;},
        commandApplied:command=>postMessage({type:'target-applied',command})
      }:{};
      const result=await ResearchRuntime.run(ca,data,{...event.data.options,controller},
        value=>postMessage({type:'progress',controller,...value}),()=>stopping,hooks);
      result.execution={mode:observing?'observe':'compare',pacing:observing?'wall-paced-fixed-step':'offline-fixed-step',
        fixed_dt_s:.001,effective_speed:result.wall_seconds>0?result.simulated_seconds/result.wall_seconds:null};
      results.push(result);
      postMessage({type:'result',result});
    }
    postMessage({type:'done',stopped:stopping,count:results.length});
  } catch(error) {postMessage({type:'error',text:String(error.stack||error.message||error)});}
  finally {busy=false;observing=false;paused=false;pendingTarget=null;}
};
