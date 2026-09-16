/* UI state-machine test without a browser. Layout/rendering is not asserted. */
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const R=require('../runtime.js'),Results=require('../results.js');
function fixture(){
  const html=fs.readFileSync(path.join(__dirname,'../index.html'),'utf8'),elements=new Map(),messages=[];
  class Element{
    constructor(){this.value='';this.disabled=false;this.options=[];this.events={};this.textContent='';this.classList={toggle(){}};}
    addEventListener(type,fn){this.events[type]=fn;}
    click(){if(!this.disabled)this.events.click?.({target:this});}
    append(...nodes){for(const node of nodes){if(node.id)elements.set(node.id,node);this.options.push(node);node.parentElement=this;}}
    replaceChildren(){this.options=[];}
    setAttribute(key,value){this[key]=value;}
    querySelector(selector){if(selector==='thead tr')return this.head||=(new Element());return this.options.find(o=>selector.includes(`"${o.value}"`))||null;}
  }
  for(const match of html.matchAll(/<(\w+)\b([^>]*\bid="([^"]+)"[^>]*)>/g)){
    const element=new Element();element.id=match[3];element.value=match[2].match(/\bvalue="([^"]*)"/)?.[1]||'';
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
    location:{href:'http://localhost/sim.html'},history:{replaceState(){}},matchMedia:()=>({matches:false}),
    requestAnimationFrame:()=>1,
    document:{getElementById:id=>elements.get(id),createElement:()=>new Element(),createElementNS:()=>new Element(),
      querySelector:()=>aircraft,querySelectorAll:selector=>{
        if(selector.includes('.controls'))return [...elements.values()].filter(e=>['profile','controller','feedback','scenario','seconds','speed','altitude','preview','mismatch','seed','log-hz'].includes(e.id));
        if(!classes.has(selector))classes.set(selector,[new Element()]);return classes.get(selector);
      }},
    Worker:class{constructor(url){this.url=url;context.activeWorker=this;}postMessage(message){messages.push(message);}}
  };
  vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(__dirname,'../page.js'),'utf8'),context);
  return {context,elements,messages,evaluate:code=>vm.runInContext(code,context)};
}
test('unified page defaults to selected observation, preserves comparison settings and sends honest mode/controller options',()=>{
  const {elements:e,messages,evaluate}=fixture();
  assert.equal(evaluate('mode'),'observe');assert.equal(e.get('profile').value,'selected');
  assert.equal(e.get('seconds').value,'30');assert.equal(e.get('mode-observe')['aria-pressed'],'true');
  e.get('mode-compare').click();assert.equal(evaluate('mode'),'compare');assert.equal(e.get('seconds').value,'0.2');
  e.get('seconds').value='1';e.get('mode-observe').click();assert.equal(e.get('seconds').value,'30');
  e.get('run').click();const request=messages.at(-1);
  assert.equal(request.mode,'observe');assert.equal(request.options.controller,'pd');assert.equal(request.profile,'selected');
  assert.equal(request.options.scenario,'schedule');assert.equal(request.options.preview,false);
  assert.equal(e.get('mode-compare').disabled,true);
  e.get('live-target-speed').value='3';e.get('live-target-altitude').value='21';e.get('apply-target').click();
  assert.equal(messages.at(-1).type,'target');assert.equal(messages.at(-1).speed,3);
});
test('observed command schedule transfers to precise comparison without overwriting the source log',()=>{
  const {context,elements:e,evaluate,messages}=fixture();
  const state=[0,0,20,0,0,0,0,-Math.SQRT1_2,0,Math.SQRT1_2,0,0,0,1500,1500,1500,1500,1];
  const cfg=R.validateOptions({controller:'pd',scenario:'schedule',seconds:30,feedback:'eskf',seed:7,preview:false,
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
  assert.equal(JSON.stringify(context.report),before);
  // Choosing a shorter prefix trims only the NEW experiment, never the log.
  evaluate('setBusy(false)');e.get('seconds').value='0.02';e.get('run').click();
  assert.equal(messages.at(-1).options.commands.length,1);
  assert.equal(JSON.stringify(context.report),before);
});
