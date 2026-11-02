"use strict";
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const source=fs.readFileSync('src/operator_console/static/v2/app.js','utf8');
function context(rotation=0){
  const events=[];const ctx={canvas:{width:640,height:480},save(){},restore(){},beginPath(){},closePath(){},moveTo(...p){events.push(['move',...p]);},lineTo(...p){events.push(['line',...p]);},quadraticCurveTo(...p){events.push(['curve',...p]);},fill(){events.push(['fill',this.fillStyle]);},stroke(){events.push(['stroke',this.strokeStyle]);},setLineDash(p){events.push(['dash',...p]);},measureText:t=>({width:t.length*7}),fillRect(){},fillText(t){events.push(['text',t]);}};
  const c={Math,Number,Array,objectName:n=>n};vm.createContext(c);
  vm.runInContext(source.slice(source.indexOf('function rotatePoint('),source.indexOf('function displayData(')),c);
  vm.runInContext(source.slice(source.indexOf('function previewPoint('),source.indexOf('function drawScene(')),c);
  return {c,ctx,events,d:{width:640,height:480,rotation_deg:rotation}};
}
const preview={step_id:1,target:'eraser',feasible:true,rz_deg:72,center_px:[100,100],source_center_px:[200,200],polygon_px:[[90,90],[110,90],[110,110],[90,110]],envelope_px:[[80,80],[120,80],[120,120],[80,120]],direction_px:[[100,100],[115,100]]};
for(const rotation of [0,90,180,270])test('preview shares image transform at '+rotation+' degrees',()=>{
  const {c,ctx,events,d}=context(rotation);c.paintPlacementPreviews(ctx,d,[preview],true);
  const p=c.rotatePoint([90,90],640,480,rotation);
  assert(events.some(e=>e[0]==='move'&&e[1]===p[0]&&e[2]===p[1]));
  assert(events.some(e=>e[0]==='text'&&e[1].includes('RZ 72.0°')));
  assert.equal(events.filter(e=>e[0]==='curve').length,1);
});
test('envelope toggle adds one independent dashed polygon',()=>{
  const a=context(),b=context();a.c.paintPlacementPreviews(a.ctx,a.d,[preview],false);b.c.paintPlacementPreviews(b.ctx,b.d,[preview],true);
  assert.equal(b.events.filter(e=>e[0]==='stroke').length,a.events.filter(e=>e[0]==='stroke').length+1);
  assert(b.events.some(e=>e[0]==='dash'&&e[1]===2&&e[2]===5));
});
test('rotated clearance follows every supplied vertex without an AABB reconstruction',()=>{
  const {c,ctx,events,d}=context(),vertices=[[50,40],[140,110],[110,150],[20,80]];
  c.paintPlacementPreviews(ctx,d,[{...preview,envelope_px:vertices}],true);
  const start=events.findIndex(e=>e[0]==='move'&&e[1]===50&&e[2]===40);
  assert(start>=0);
  assert.deepEqual(events.slice(start,start+4),[['move',50,40],['line',140,110],['line',110,150],['line',20,80]]);
});
test('rejected candidates never render as successful placement targets',()=>{
  const {c,ctx,events,d}=context();c.paintPlacementPreviews(ctx,d,[preview,{...preview,step_id:3,feasible:false,predicted_scene:true}],false);
  assert(events.some(e=>e[0]==='text'&&e[1].startsWith('Step 1:')));
  assert(!events.some(e=>e[0]==='text'&&e[1].includes('Step 3:')));
  assert(!events.some(e=>e[0]==='stroke'&&e[1]==='#BE123C'));
});
test('missing and nonfinite projections draw no invented polygon',()=>{
  const {c,ctx,events,d}=context();c.paintPlacementPreviews(ctx,d,[{step_id:1},{...preview,center_px:[NaN,2]}],true);assert.equal(events.length,0);
});
test('new controls are labelled, preview defaults on, envelope defaults off',()=>{
  const html=fs.readFileSync('src/operator_console/static/v2/index.html','utf8');
  assert.match(html,/id="show-placement-preview"[^>]*checked/);
  const tag=html.match(/<input id="show-placement-envelope"[^>]*>/)[0];assert(!tag.includes('checked'));
  assert.match(source,/placement_orientation/);assert.match(source,/directRunId=null/);
});

test('a new frame never paints previews from the previous frame',()=>{const {c,ctx,events,d}=context();d.frame_id='current';c.paintPlacementPreviews(ctx,d,[{...preview,frame_id:'old'}],true);assert.equal(events.length,0);});


test('post-return scene omits placement overlays belonging to the last picking frame',()=>{
  const {c,ctx,events,d}=context();
  const frame='current';d.frame_id=frame;
  c.paintPlacementPreviews(ctx,d,[{...preview,frame_id:frame}],true);
  assert(events.some(e=>e[0]==='text'&&e[1].includes('Step 1')));
  const final=context();final.d.frame_id='post-return';
  final.c.paintPlacementPreviews(final.ctx,final.d,[{...preview,frame_id:frame}],true);
  assert.equal(final.events.length,0);
});
