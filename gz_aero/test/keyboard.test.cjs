const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');

function keyboard(){
  // Read the actual generated handler, not a second implementation of its rules.
  const html=fs.readFileSync(path.join(__dirname,'../../results/flight_sim.html'),'utf8');
  const handler=html.match(/addEventListener\("keydown", e => \{[\s\S]*?\n  \}\);/)[0];
  const calls=[];let onKey;
  vm.runInNewContext(handler,{addEventListener:(type,fn)=>onKey=fn,toggleRun:()=>calls.push('toggle'),
    $:id=>({click:()=>calls.push(id)})});
  return {calls,key:(code,target={tagName:'DIV'},extra={})=>{
    let prevented=false;onKey({code,key:code==='Space'?' ':code.replace('Key','').toLowerCase(),target,
      preventDefault:()=>prevented=true,...extra});return prevented;
  }};
}
test('original simulator Space works after a slider drag without stealing numeric or native button input',()=>{
  const {calls,key}=keyboard();
  assert.equal(key('Space'),true);assert.equal(key('Space',{tagName:'INPUT',type:'range'}),true);
  for(const target of [{tagName:'INPUT',type:'number'},{tagName:'SELECT'},{tagName:'BUTTON'},{tagName:'SUMMARY'},{isContentEditable:true}])assert.equal(key('Space',target),false);
  for(const extra of [{repeat:true},{metaKey:true},{ctrlKey:true},{altKey:true},{isComposing:true},{defaultPrevented:true}])assert.equal(key('Space',undefined,extra),false);
  assert.deepEqual(calls,['toggle','toggle']);
});
test('original reset/follow shortcuts remain available outside editing controls',()=>{
  const {calls,key}=keyboard();key('KeyR');key('KeyF');
  key('KeyR',{tagName:'INPUT',type:'range'});key('KeyF',{tagName:'INPUT',type:'number'});
  assert.deepEqual(calls,['#rst','#follow']);
});
