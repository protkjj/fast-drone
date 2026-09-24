/* UI state-machine test without a browser. Layout/rendering is not asserted. */
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const R=require('../runtime.js'),Results=require('../results.js');
function fixture(url='http://localhost/sim.html'){
  const html=fs.readFileSync(path.join(__dirname,'../index.html'),'utf8'),elements=new Map(),messages=[],documentEvents={};
  class Element{
    constructor(){this.value='';this.disabled=false;this.options=[];this.events={};this.textContent='';this.classList={toggle(){}};this.style={setProperty(){}};}
    addEventListener(type,fn){this.events[type]=fn;}
    click(){if(!this.disabled)this.events.click?.({target:this});}
    append(...nodes){for(const node of nodes){if(node.id)elements.set(node.id,node);this.options.push(node);node.parentElement=this;}}
    replaceChildren(){this.options=[];}
    setAttribute(key,value){this[key]=value;}
    querySelector(selector){if(selector==='thead tr')return this.head||=(new Element());return this.options.find(o=>selector.includes(`"${o.value}"`))||null;}
  }
  for(const match of html.matchAll(/<(\w+)\b([^>]*\bid="([^"]+)"[^>]*)>/g)){
    const element=new Element();element.id=match[3];element.value=match[2].match(/\bvalue="([^"]*)"/)?.[1]||'';
    element.tagName=match[1].toUpperCase();element.type=match[2].match(/\btype="([^"]*)"/)?.[1];
    element.disabled=/\bdisabled\b/.test(match[2]);element.parentElement=new Element();
    if(match[1]==='select'){
      const content=html.slice(match.index+match[0].length).split('</select>')[0];
      for(const option of content.matchAll(/<option\b([^>]*)>([^<]*)/g)){
        const node=new Element();node.value=option[1].match(/value="([^"]*)"/)?.[1]||'';node.textContent=option[2];element.append(node);
        if(!element.value||/\bselected\b/.test(option[1]))element.value=node.value;
      }
    }
    elements.set(element.id,element);
  }
  const aircraft=new Element(),classes=new Map();
  const context={console,URL,performance,setTimeout,clearTimeout,Blob,ResearchRuntime:R,ResearchResults:Results,
    location:{href:url},history:{replaceState(){}},matchMedia:()=>({matches:false}),
    requestAnimationFrame:()=>1,
    document:{getElementById:id=>elements.get(id),createElement:()=>new Element(),createElementNS:()=>new Element(),
      addEventListener:(type,fn)=>documentEvents[type]=fn,
      querySelector:()=>aircraft,querySelectorAll:selector=>{
        if(selector.includes('.controls'))return [...elements.values()].filter(e=>['profile','controller','feedback','scenario','seconds','speed','altitude','preview','mismatch','seed','log-hz'].includes(e.id));
        if(!classes.has(selector))classes.set(selector,[new Element()]);return classes.get(selector);
      }},
    Worker:class{constructor(url){this.url=url;context.activeWorker=this;}postMessage(message){messages.push(message);}}
  };
  vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(__dirname,'../page.js'),'utf8'),context);
  const key=(code,target={tagName:'DIV'},extra={})=>{
    let prevented=false;documentEvents.keydown({code,key:code==='Space'?' ':code.replace('Key','').toLowerCase(),
      target,preventDefault(){prevented=true;},...extra});return prevented;
  };
  return {context,elements,messages,key,evaluate:code=>vm.runInContext(code,context)};
}
test('Space starts, pauses and resumes, including after range input; typing and native buttons retain their keys',()=>{
  const {context,elements:e,messages,key,evaluate}=fixture();
  for(const target of [e.get('live-target-speed'),e.get('observe-controller'),e.get('go'),{tagName:'DIV',isContentEditable:true}]){
    assert.equal(key('Space',target),false);assert.equal(messages.length,0);
  }
  for(const extra of [{repeat:true},{ctrlKey:true},{metaKey:true},{altKey:true},{isComposing:true}])assert.equal(key('Space',undefined,extra),false);
  assert.equal(key('Space'),true);assert.equal(messages.at(-1).type,'run');assert.equal(evaluate('running'),true);
  assert.equal(key('Space',e.get('speed-slider')),true);assert.equal(messages.at(-1).type,'pause');assert.equal(messages.at(-1).paused,true);
  const count=messages.length;key('Space');assert.equal(messages.length,count); // Await acknowledgement, not double-toggle.
  context.activeWorker.onmessage({data:{type:'paused',paused:true,t:.1}});
  assert.match(e.get('go').textContent,/계속/);key('Space');assert.equal(messages.at(-1).paused,false);
  context.activeWorker.onmessage({data:{type:'paused',paused:false,t:.1}});assert.match(e.get('go').textContent,/일시정지/);
});
test('R waits for orderly stop, resets the view and preserves the recorded result; F toggles the camera button',()=>{
  const {context,elements:e,messages,key,evaluate}=fixture();
  let followed=0;e.get('follow').events.click=()=>followed++;key('KeyF');assert.equal(followed,1);
  key('KeyF',e.get('live-target-speed'));assert.equal(followed,1);
  key('Space');key('KeyR');assert.equal(messages.at(-1).type,'stop');assert.equal(evaluate('resetRequested'),true);
  context.activeWorker.onmessage({data:{type:'done',stopped:true,count:0}});
  assert.equal(evaluate('running'),false);assert.equal(evaluate('resetRequested'),false);assert.match(e.get('status').textContent,/초기 상태/);
  evaluate('results.hybrid={saved:true};');key('KeyR');assert.equal(evaluate('results.hybrid.saved'),true);
  assert.equal(e.get('timeline').value,'0');assert.match(e.get('go').textContent,/시작/);
});
test('live angle is a display-only air-relative body-axis angle, and chart labels use actual pixel dimensions',()=>{
  const {elements:e,evaluate}=fixture();
  assert.equal(evaluate('airAngle({q:[0,0,0,1],v:[1,0,0]})'),0);
  assert.equal(evaluate('airAngle({q:[0,0,0,1],v:[0,1,0]})'),90);
  assert.equal(evaluate('airAngle({q:[0,0,0,1],v:[-1,0,0]})'),180);
  assert.equal(evaluate('airAngle({q:[0,0,0,1],v:[1,0,0],environment:[1,0,0,0,0,0]})'),null);
  e.get('velocity-chart').clientWidth=220;e.get('velocity-chart').clientHeight=135;
  evaluate('drawCharts()');assert.equal(e.get('velocity-chart').viewBox,'0 0 220 135');
});
test('unified page defaults to selected observation, preserves comparison settings and sends honest mode/controller options',()=>{
  const {context,elements:e,messages,evaluate}=fixture();
  assert.equal(evaluate('mode'),'observe');assert.equal(e.get('profile').value,'selected');
  assert.equal(e.get('seconds').value,'30');assert.equal(e.get('mode-observe')['aria-pressed'],'true');
  e.get('mode-compare').click();assert.equal(evaluate('mode'),'compare');assert.equal(e.get('seconds').value,'0.2');
  e.get('seconds').value='1';e.get('mode-observe').click();assert.equal(e.get('seconds').value,'30');
  e.get('run').click();const request=messages.at(-1);
  assert.equal(request.mode,'observe');assert.equal(request.options.controller,'hybrid');assert.equal(request.profile,'selected');
  assert.equal(request.options.scenario,'schedule');assert.equal(request.options.preview,false);
  assert.equal(request.options.hybrid_actuator_feedback,false);
  assert.equal(e.get('mode-compare').disabled,true);
  e.get('live-target-speed').value='3';e.get('live-target-altitude').value='21';e.get('apply-target').click();
  assert.equal(messages.at(-1).type,'target');assert.equal(messages.at(-1).speed,3);
  context.activeWorker.onmessage({data:{type:'done',stopped:true,count:0}});
  assert.match(e.get('target-state').textContent,/적용되지 않았/);
});
test('actuator-aware Hybrid is explicit opt-in, without replacing flight controls',()=>{
  const {elements:e,messages}=fixture('http://localhost/research/index.html?actuator_feedback=1');
  assert.match(e.get('mode-note').textContent,/실험용 Hybrid 모터 능력 제약 ON/);
  e.get('run').click();assert.equal(messages.at(-1).options.hybrid_actuator_feedback,true);
});
test('sliders and typed targets work before takeoff and while running; wind is sent, not just displayed',()=>{
  const {elements:e,messages,evaluate}=fixture();
  e.get('speed-slider').value='2.5';e.get('speed-slider').events.input();e.get('speed-slider').events.change();
  e.get('live-target-altitude').value='230';e.get('live-target-altitude').events.change();
  assert.equal(e.get('altitude-slider').max,'300');assert.equal(e.get('altitude-slider').value,'230');
  e.get('live-wind-speed').value='4';e.get('wind-head').click();
  e.get('observe-controller').value='nmpc';e.get('run').click();
  const config=messages.at(-1).options;
  assert.equal(config.controller,'nmpc');assert.equal(config.commands[0].speed,2.5);
  assert.equal(config.commands[0].altitude,230);assert.equal(config.altitude,20); // target != initial state
  assert.equal(config.commands[0].wind_speed,4);assert.equal(config.commands[0].wind_angle,180);
  e.get('wind-calm').click();assert.equal(messages.at(-1).wind_speed,0);
  e.get('hover-target').click();assert.equal(messages.at(-1).speed,0);
  e.get('live-target-speed').value='';e.get('live-target-altitude').value='-1';e.get('apply-target').click();
  assert.match(e.get('errors').textContent,/고도|일정/);
  evaluate('setBusy(false)');
});
test('holding a slider drag still applies targets periodically without waiting for release',async()=>{
  const {elements:e,messages}=fixture();e.get('run').click();
  for(const value of ['1','2']){
    e.get('speed-slider').value=value;e.get('speed-slider').events.input();
    await new Promise(resolve=>setTimeout(resolve,100));
    assert.equal(messages.at(-1).type,'target');assert.equal(messages.at(-1).speed,Number(value));
  }
  const count=messages.length;e.get('live-target-speed').value='';e.get('apply-target').click();
  assert.equal(messages.length,count);assert.match(e.get('errors').textContent,/일정/);
});
test('observed command schedule transfers to precise comparison without overwriting the source log',()=>{
  const {context,elements:e,evaluate,messages}=fixture();
  const state=[0,0,20,0,0,0,0,-Math.SQRT1_2,0,Math.SQRT1_2,0,0,0,1500,1500,1500,1500,1];
  const cfg=R.validateOptions({controller:'pd',scenario:'schedule',seconds:30,feedback:'eskf',seed:7,preview:false,hybrid_actuator_feedback:false,
    commands:[{t:0,speed:0,altitude:20},{t:.04,speed:3,altitude:21}]});
  context.report={profile_id:'selected-6931',configuration:cfg,simulated_seconds:.2,status:'stopped',metrics:{},counts:{},
    final:state,trace:[0,.2].map(t=>({t,state,v:state.slice(3,6),z:20,reference:[0,0,0,20]}))};
  const before=JSON.stringify(context.report);
  evaluate('results.pd=report;$("replay-controller").value="pd";showResults();setupReplay();');
  e.get('compare-record').click();assert.equal(evaluate('mode'),'compare');assert.equal(e.get('seconds').value,'0.2');
  assert.equal(e.get('scenario').value,'schedule');assert.equal(e.get('preview').value,'false');
  assert.equal(e.get('feedback').value,'eskf');assert.equal(e.get('seed').value,'7');
  e.get('run').click();const options=messages.at(-1).options;
  assert.equal(messages.at(-1).mode,'compare');assert.deepEqual(options.commands,cfg.commands);
  assert.equal(options.hybrid_actuator_feedback,false);
  assert.equal(JSON.stringify(context.report),before);
  // Choosing a shorter prefix trims only the NEW experiment, never the log.
  evaluate('setBusy(false)');e.get('seconds').value='0.02';e.get('run').click();
  assert.equal(messages.at(-1).options.commands.length,1);
  assert.equal(JSON.stringify(context.report),before);
});
