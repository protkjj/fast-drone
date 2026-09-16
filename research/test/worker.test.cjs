const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const R=require('../runtime.js');

test('worker observation supports target acknowledgement, pause, stop and subsequent real IPOPT comparison',async()=>{
  const ca=await require('@casadi/casadi-wasm')();await ca.load_interpolant('linear');
  const data=JSON.parse(fs.readFileSync(path.join(__dirname,'../generated/selected.json')));
  const messages=[],results=[];let done,pauseStarted=false,pausedTick=null,pauseVerified=false;
  const ctx={ResearchRuntime:R,performance,setTimeout,clearTimeout,console,importScripts:()=>{},injectedCasadi:ca,injectedData:data};
  ctx.self=ctx;ctx.postMessage=message=>{
    messages.push(message);
    if(message.type==='error')throw new Error(message.text);
    if(message.type==='progress'&&message.controller==='pd'&&message.t>=.04&&!pauseStarted){
      pauseStarted=true;ctx.onmessage({data:{type:'target',speed:3,altitude:21}});ctx.onmessage({data:{type:'pause',paused:true}});
    }
    if(message.type==='paused'&&message.paused){
      pausedTick=message.t;const count=messages.filter(m=>m.type==='progress').length;
      setTimeout(()=>{
        pauseVerified=count===messages.filter(m=>m.type==='progress').length;
        ctx.onmessage({data:{type:'pause',paused:false}});
      },50);
    }
    if(message.type==='progress'&&message.controller==='pd'&&message.t>=.16)ctx.onmessage({data:{type:'stop'}});
    if(message.type==='result')results.push(message.result);
    if(message.type==='done')done();
  };
  vm.createContext(ctx);vm.runInContext(fs.readFileSync(path.join(__dirname,'../worker.js'),'utf8'),ctx);
  // Exercise the actual message loop and runtime; only network bootstrap is replaced.
  vm.runInContext('loadCasadi=async()=>injectedCasadi;loadProfile=async()=>injectedData;',ctx);
  const observe=R.validateOptions({controller:'pd',scenario:'schedule',seconds:1,preview:false,commands:[{t:0,speed:0,altitude:20}]});
  await new Promise(resolve=>{done=resolve;ctx.onmessage({data:{type:'run',mode:'observe',profile:'selected',options:observe}});});
  assert.ok(pauseVerified);assert.ok(pausedTick>=.04);
  assert.equal(results[0].status,'stopped');assert.equal(results[0].counts.nmpc,0);
  assert.equal(results[0].execution.mode,'observe');assert.ok(results[0].simulated_seconds<1);
  const applied=messages.find(m=>m.type==='target-applied');assert.ok(applied);assert.equal(applied.command.speed,3);
  const precise=R.validateOptions({scenario:'hover',seconds:.02});
  await new Promise(resolve=>{done=resolve;ctx.onmessage({data:{type:'run',mode:'compare',profile:'selected',controllers:['nmpc'],options:precise}});});
  assert.equal(results[1].execution.mode,'compare');assert.equal(results[1].counts.nmpc,1);
  assert.equal(results[1].counts.indi,0);assert.equal(results[1].metrics.solver_failures,0);
});
