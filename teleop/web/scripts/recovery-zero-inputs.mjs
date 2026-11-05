// Zero velocity with the real production deadman mappings; no robot connection.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';
import crypto from 'node:crypto';
const root=path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const source=fs.readFileSync(path.join(root,'src/input.ts'),'utf8');
const compiled=ts.transpileModule(source,{compilerOptions:{target:ts.ScriptTarget.ES2022,module:ts.ModuleKind.ESNext}}).outputText;
const {fromGamepad,fromKeyboard,command}=await import('data:text/javascript;base64,'+Buffer.from(compiled).toString('base64'));
const profile={translation_m_s:.002,rotation_rad_s:Math.PI/180,gripper_s:.05};
if(process.argv[3]) {
  const fixture=JSON.parse(fs.readFileSync(process.argv[3],'utf8'));
  if(fixture.production_input_sha256!==crypto.createHash('sha256').update(source).digest('hex'))throw new Error('Fixture mapper SHA differs');
  Object.assign(profile,fixture.profile);
}
const signals={};
for(const name of ['gamepad','keyboard']) {
  const buttons=Array.from({length:17},()=>({pressed:false,value:0}));buttons[4]={pressed:true,value:1};
  const input=name==='gamepad'?fromGamepad({connected:true,id:'synthetic standard gamepad',mapping:'standard',axes:[0,0,0,0],buttons,index:0,timestamp:0}):fromKeyboard(new Set(['Space']));
  signals[name]={...command(input,'translation','base',true,profile),source:name,raw:{...input.raw,test_only:true}};
  if(!signals[name].deadman || signals[name].twist.some(Boolean) || signals[name].gripper_rate)throw new Error('Expected production-mapped zero increment');
}
const result={production_input_sha256:crypto.createHash('sha256').update(source).digest('hex'),profile,signals};
if(!process.argv[2])throw new Error('New output file required');
fs.writeFileSync(process.argv[2],JSON.stringify(result),{flag:'wx'});
