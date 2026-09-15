/* IPOPT runs here, never on the rendering/input thread. */
importScripts('./runtime.js');
let stopping=false, busy=false, ca=null;
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
  await result.load_nlpsol('ipopt');
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
  if(event.data.type!=='run'||busy) return;
  busy=true; stopping=false;
  try {
    postMessage({type:'status',text:'CasADi/IPOPT 및 기체 모델 준비 중… 첫 실행에는 시간이 걸립니다.'});
    if(!ca) ca=await loadCasadi();
    const data=await loadProfile(event.data.profile);
    postMessage({type:'status',text:'실험 계산 중. 시뮬레이션 시간은 풀이 완료 후 진행됩니다.'});
    const results=[];
    for(const controller of event.data.controllers) {
      if(stopping) break;
      const result=await ResearchRuntime.run(ca,data,{...event.data.options,controller},
        value=>postMessage({type:'progress',controller,...value}),()=>stopping);
      results.push(result);
      postMessage({type:'result',result});
    }
    postMessage({type:'done',stopped:stopping,count:results.length});
  } catch(error) {postMessage({type:'error',text:String(error.stack||error.message||error)});}
  finally {busy=false;}
};
