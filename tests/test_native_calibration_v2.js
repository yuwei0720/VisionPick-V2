'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../src/operator_console/static/v2/app.js'),'utf8');
function previewHarness(){
  const nodes=new Map(),$=id=>{if(!nodes.has(id))nodes.set(id,{disabled:true,textContent:''});return nodes.get(id);};
  const pending=[],errors=[];
  const c=vm.createContext({$,tableCapture:{kind:'station'},roiRequest:0,roiPayload:()=>({capture_id:'frozen',roi_xyxy:[10,10,30,30]}),
    api:()=>new Promise((resolve,reject)=>pending.push({resolve,reject})),error:e=>errors.push(e)});
  vm.runInContext(source.slice(source.indexOf('async function updateRoiPreview()'),source.indexOf('async function saveRoi(')),c);
  return {c,$,pending,errors};
}
const result=z=>({robot_surface_xyz_mm:[10,20,z],depth_valid_ratio:.75,workspace_ok:true});
test('slower previous ROI response cannot replace the latest selection',async()=>{
  const h=previewHarness(),first=h.c.updateRoiPreview(),second=h.c.updateRoiPreview();
  h.pending[1].resolve(result(120));await second;
  h.pending[0].resolve(result(90));await first;
  assert.match(h.$('roi-preview').textContent,/120\.00/);
  assert.equal(h.c.tableCapture.preview.robot_surface_xyz_mm[2],120);
});
test('cancelled ROI response cannot enable a save action',async()=>{
  const h=previewHarness(),request=h.c.updateRoiPreview();h.c.tableCapture=null;h.c.roiRequest++;
  h.pending[0].resolve(result(120));await request;
  assert.equal(h.$('roi-save').disabled,true);
  assert.equal(h.$('table-roi-save').disabled,true);
});
test('depth failure keeps saving disabled and provides recovery feedback',async()=>{
  const h=previewHarness(),request=h.c.updateRoiPreview();h.pending[0].reject(Error('有效 depth pixel 不足'));await request;
  assert.equal(h.c.tableCapture.preview,null);assert.equal(h.$('roi-save').disabled,true);
  assert.match(h.$('roi-preview').textContent,/depth pixel/);assert.equal(h.errors.length,1);
});
test('pointer coordinates stay within the frozen image despite CSS scaling and outside release',()=>{
  const c=vm.createContext({$:()=>({width:640,height:480,getBoundingClientRect:()=>({left:100,top:50,width:320,height:240})})});
  vm.runInContext(source.slice(source.indexOf('function canvasPointer('),source.indexOf("$('scene').addEventListener('pointerdown'")),c);
  assert.deepEqual(Array.from(c.canvasPointer({clientX:260,clientY:170})),[320,240]);
  assert.deepEqual(Array.from(c.canvasPointer({clientX:500,clientY:10})),[640,0]);
});
