// Same experiment as the browser, for repeatable command-line checks.
const fs = require('node:fs');
const path = require('node:path');
const runtime = require('./runtime.js');
async function main() {
  const profile=process.argv[2]||'simple';
  const controller=process.argv[3]||'hybrid';
  const feedback=process.argv[4]||'truth';
  const seconds=Number(process.argv[5]||'1');
  const scenario=process.argv[6]||'hover';
  if(!['simple','selected'].includes(profile)) throw new Error('Unknown profile');
  const ca=await require('@casadi/casadi-wasm')({print:()=>{},printErr:message=>{
    if(!message.includes('unsupported syscall: __syscall_getrusage')) process.stderr.write(message+'\n');
  }});
  await ca.load_nlpsol('ipopt');
  await ca.load_interpolant('linear');
  const data=JSON.parse(fs.readFileSync(path.join(__dirname,'generated',profile+'.json'),'utf8'));
  const result=await runtime.run(ca,data,{controller,feedback,seconds,scenario});
  const {trace,solves,...summary}=result;
  console.log(JSON.stringify({...summary,first_solves:solves.slice(0,3)},null,2));
  if(process.argv[7]) fs.writeFileSync(process.argv[7],JSON.stringify(result,null,2)+'\n');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
