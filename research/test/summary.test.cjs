const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),os=require('node:os');
const crypto=require('node:crypto');
const {execFileSync}=require('node:child_process');

test('summary commit is verified against recorded sources, never inferred from checkout alone',()=>{
  const root=path.resolve(__dirname,'../..');
  const revision=execFileSync('git',['rev-parse','HEAD'],{cwd:root,encoding:'utf8'}).trim();
  const hash=crypto.createHash('sha256').update(execFileSync('git',['show',`${revision}:research/model.py`],{cwd:root})).digest('hex');
  // Use the checked-out commit, including on CI's shallow clone. Historical Git
  // objects are not needed to test that a mismatched log loses its commit label.
  const reports=[7,42,2026].flatMap(seed=>['hybrid','nmpc'].map(controller=>({
    configuration:{scenario:'gust',controller,seed},implementation:{source_sha256:{'model.py':hash}},
    metrics:{velocity_rmse_mps:1,altitude_rmse_m:1,electrical_energy_J:1,outside_prop_map_fraction:0,solver_failures:0},
    counts:{nmpc:1},solves:[],trace:[{t:0,state:[0],q_xyzw:[0,0,0,1],sensor:{capture_t_s:0,gyro_t_s:0,rotor_t_s:0}}]
  })));
  const folder=fs.mkdtempSync(path.join(os.tmpdir(),'drone-summary-test-'));
  const input=path.join(folder,'batch.json'),output=path.join(folder,'summary.json');
  const batch={started_at:'2026-09-16',updated_at:'2026-09-16',cases:Array(3).fill({}),reports};
  fs.writeFileSync(input,JSON.stringify(batch),{flag:'wx'});
  const script=path.resolve(__dirname,'../summarize_v2.cjs');
  execFileSync(process.execPath,[script,input,output,revision]);
  assert.equal(JSON.parse(fs.readFileSync(output)).source_commit,revision);
  // A changed source hash is sufficient to refuse the otherwise valid Git label.
  reports[0].implementation={...reports[0].implementation,source_sha256:{'model.py':'0'.repeat(64)}};
  const changed=path.join(folder,'changed-batch.json'),changedOutput=path.join(folder,'changed-summary.json');
  fs.writeFileSync(changed,JSON.stringify(batch),{flag:'wx'});
  execFileSync(process.execPath,[script,changed,changedOutput,revision]);
  assert.equal(JSON.parse(fs.readFileSync(changedOutput)).source_commit,null);
  // Existing output is protected; re-running cannot silently replace prior evidence.
  assert.throws(()=>execFileSync(process.execPath,[script,input,output,revision],{stdio:'pipe'}),/EEXIST/);
});
