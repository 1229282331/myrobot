// Execute the actual production TypeScript mappers, without a browser or robot.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';
import crypto from 'node:crypto';
const root=path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const source=fs.readFileSync(path.join(root,'src/input.ts'),'utf8');
const compiled=ts.transpileModule(source,{compilerOptions:{target:ts.ScriptTarget.ES2022,module:ts.ModuleKind.ESNext}}).outputText;
const {fromGamepad,fromKeyboard,command}=await import('data:text/javascript;base64,'+Buffer.from(compiled).toString('base64'));
if(process.argv[3] && !['initial','delivery'].includes(process.argv[3]))throw new Error('Unknown test profile');
const profile=process.argv[3]==='delivery'?{translation_m_s:.005,rotation_rad_s:2*Math.PI/180,gripper_s:.1}:{translation_m_s:.002,rotation_rad_s:Math.PI/180,gripper_s:.05};
const gripperProfile=process.argv[4]??'plan32';
const gripperSettings=JSON.parse(fs.readFileSync(path.join(root,'../config/gripper_profiles.json'),'utf8'))[gripperProfile];
if(!gripperSettings)throw new Error('Unknown gripper command profile');
if(gripperProfile==='empty64-medium')profile.gripper_s=gripperSettings.gripper_s;
if(process.argv[5] && process.argv[5]!=='full')throw new Error('Unknown gamepad amplitude selection');
const amplitude=process.argv[3]==='delivery'||process.argv[5]==='full'?1:.3;
const sha=crypto.createHash('sha256').update(source).digest('hex');
const rows=[];
for(const source of ['gamepad','keyboard'])for(const mode of ['translation','rotation'])for(const frame of ['base','tool'])for(const fine of [false,true])for(let axis=0;axis<4;axis++)for(const sign of [-1,1]){
 let input;
 if(source==='gamepad'){
  const axes=[0,0,0,0],buttons=Array.from({length:17},()=>({pressed:false,value:0}));buttons[4]={pressed:true,value:1};buttons[5]={pressed:fine,value:Number(fine)};
  if(axis<3)axes[[0,1,3][axis]]=sign*(axis===0?1:-1)*amplitude;else buttons[sign>0?6:7]={pressed:true,value:amplitude};
  input=fromGamepad({connected:true,id:'synthetic standard gamepad',mapping:'standard',axes,buttons,index:0,timestamp:0});
 }else{
  const keys=new Set(['Space']);if(fine)keys.add('ShiftLeft');keys.add((sign>0?['KeyD','KeyW','KeyE','KeyF']:['KeyA','KeyS','KeyQ','KeyR'])[axis]);
  input=fromKeyboard(keys);
 }
 rows.push({source,mode,frame,fine,axis,sign,input:{...command(input,mode,frame,true,profile),source,raw:{...input.raw,test_only:true,fixture_source_sha256:sha}}});
}
const neutral={};
for(const source of ['gamepad','keyboard']){
 const i=source==='gamepad'?fromGamepad({connected:true,id:'synthetic standard gamepad',mapping:'standard',axes:[0,0,0,0],buttons:Array.from({length:17},()=>({pressed:false,value:0})),index:0,timestamp:0}):fromKeyboard(new Set());
 neutral[source]={...command(i,'translation','base',true,profile),source,raw:{...i.raw,test_only:true,fixture_source_sha256:sha}};
}
const multi=[];
for(const source of ['gamepad','keyboard'])for(const mode of ['translation','rotation'])for(const frame of ['base','tool'])for(const fine of [false,true])for(const sign of [-1,1]){
 let input;
 if(source==='gamepad'){
  const buttons=Array.from({length:17},()=>({pressed:false,value:0}));buttons[4]={pressed:true,value:1};buttons[5]={pressed:fine,value:Number(fine)};
  input=fromGamepad({connected:true,id:'synthetic standard gamepad',mapping:'standard',axes:[sign*amplitude,-sign*amplitude,0,-sign*amplitude],buttons,index:0,timestamp:0});
 }else{
  const keys=new Set(['Space',...(sign>0?['KeyD','KeyW','KeyE']:['KeyA','KeyS','KeyQ'])]);if(fine)keys.add('ShiftLeft');input=fromKeyboard(keys);
 }
 multi.push({source,mode,frame,fine,sign,axes:[0,1,2],input:{...command(input,mode,frame,true,profile),source,raw:{...input.raw,test_only:true,fixture_source_sha256:sha}}});
}
const output=JSON.stringify({simulation_mapping:true,production_input_sha256:sha,gamepad_signal_amplitude:amplitude,gripper_command_profile:gripperProfile,profile,neutral,rows,multi});
if(process.argv[2])fs.writeFileSync(process.argv[2],output);else process.stdout.write(output);
