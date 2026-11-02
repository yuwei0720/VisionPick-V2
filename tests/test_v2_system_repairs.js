'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../src/operator_console/static/v2/app.js'),'utf8');
function chunk(start,end){const i=source.indexOf(start);assert(i>=0,start);const j=source.indexOf(end,i+start.length);assert(j>i,end);return source.slice(i,j);}
function preferences(raw){
  const nodes=new Map(),storage=new Map([['visionpick_v2_prefs',raw]]);
  const $=id=>{if(!nodes.has(id))nodes.set(id,{value:({rotation:'0','confirmation-mode':'once','destination-mode':'fixed',mode:'offline','semantic-source':'demo'})[id]||'',setAttribute(k,v){this[k]=v;}});return nodes.get(id);};
  const c=vm.createContext({$,viewRotation:null,localStorage:{getItem:k=>storage.get(k),setItem:(k,v)=>storage.set(k,v)}});
  vm.runInContext(chunk('const preferenceKey=','async function api('),c);return {c,$,storage};
}
test('five preferences survive a new page and restore the direct switch state',()=>{
  const h=preferences('{}'),values={rotation:'270','confirmation-mode':'direct','destination-mode':'strict',mode:'real','semantic-source':'api'};
  for(const [k,v] of Object.entries(values))h.$(k).value=v;
  h.c.savePreferences();const next=preferences(h.storage.get('visionpick_v2_prefs'));next.c.restorePreferences();
  for(const [k,v] of Object.entries(values))assert.equal(next.$(k).value,v);
  assert.equal(next.c.viewRotation,270);assert.equal(next.$('confirmation-mode')['aria-checked'],'true');
});
test('malformed, null, array and invalid individual preferences safely retain defaults',()=>{
  for(const raw of ['{','null','[]','42','{"rotation":45,"mode":"remote","confirmation-mode":"auto"}']){
    const {c,$}=preferences(raw);c.restorePreferences();assert.equal(c.viewRotation,null);assert.equal($('mode').value,'offline');assert.equal($('confirmation-mode').value,'once');
  }
});
test('storage denial is nonfatal and does not write backend settings',()=>{
  const {c}=preferences('{}');c.localStorage={getItem(){throw Error('denied');},setItem(){throw Error('denied');}};
  assert.doesNotThrow(()=>c.restorePreferences());assert.doesNotThrow(()=>c.savePreferences());
  assert.doesNotMatch(chunk('const preferenceKey=','async function api('),/fetch\(|api\(/);
});
test('polling retains selected rotation even for an active self-relative plan',()=>{
  const c=vm.createContext({viewRotation:180,state:null,$:()=>({value:''})});
  const render=chunk('function render(next){','  const preview=');
  vm.runInContext(render+'}',c);
  for(let i=0;i<5;i++)c.render({status:'waiting_answer',direction_frame:{rotation_deg:90},pending:{candidate:{tasks:[{destination_type:'self_relative'}]}}});
  assert.equal(c.viewRotation,180);c.viewRotation=null;c.render({direction_frame:{rotation_deg:270}});assert.equal(c.viewRotation,270);
});

test('Start consent distinguishes unknown, holding, confirmed empty and motion disabled',()=>{
  const dialogs=[],c=vm.createContext({bootState:{motion_enabled:true,empty_hand_confirmed:false,devices:{camera:{detected:true,count:1},robot:{detected:true}}},state:null,window:{confirm:m=>{dialogs.push(m);return true;}}});
  vm.runInContext(chunk('function sceneStartConsent(','async function refreshPreview('),c);
  assert.equal(c.sceneStartConsent('real').source,'operator_empty_hand_check');assert.equal(dialogs.length,1);
  c.bootState.empty_hand_confirmed=true;assert.equal(c.sceneStartConsent('real').move_to_start,true);assert.equal(dialogs.length,1);
  c.state={mode:'real',empty_hand_confirmed:false,holding_uncertain:true};c.window.confirm=()=>false;assert.equal(c.sceneStartConsent('real'),null);
  c.bootState.motion_enabled=false;assert.deepEqual(Object.keys(c.sceneStartConsent('real')),[]);
});


function voiceHarness(){
  const {c,$}=preferences('{}'),events=[],polls=[];
  $('voice-input').classList={toggle(){}};
  Object.assign(c,{voiceSession:null,error:m=>events.push(m),toast:m=>events.push(m),
    sourceNote(){},syncButtons(){},activeTask:()=>false,busy:false,
    setTimeout:(fn,ms)=>{polls.push(ms);fn();}});
  vm.runInContext(chunk('function voiceErrorMessage(',"$('voice-input').title="),c);
  return {c,$,events,polls};
}
test('all host voice failures have explicit Traditional Chinese feedback',()=>{
  const {c}=voiceHarness();
  for(const [cause,pattern] of [[{name:'NotAllowedError'},/權限遭拒絕/],
    [{code:'voice_network_error'},/連線失敗或逾時/],[{message:'Failed to fetch'},/連線失敗/],
    [{code:'voice_empty_audio'},/未錄到有效語音/],[{code:'voice_empty_transcript'},/未錄到有效語音/],
    [{code:'voice_busy'},/忙碌/],[{code:'voice_operation_active'},/忙碌/],
    [{code:'voice_dependency_missing'},/缺少必要套件/],
    [{code:'voice_host_microphone_unavailable'},/無法收音/],
    [{code:'voice_audio_capture_failed'},/無法收音/],
    [{code:'voice_audio_too_large'},/超出限制/],[null,/語音輸入失敗/],
    [{code:'unknown'},/語音輸入失敗/]])assert.match(c.voiceErrorMessage(cause),pattern);
});
test('recording and transcription mark the editable input busy and finish restores state',()=>{
  const {c,$}=voiceHarness(),session={original:'mic',wasReadOnly:false};c.voiceSession=session;
  for(const phase of ['starting','recording','transcribing']){
    c.voiceVisual(session,phase);assert.equal($('instruction')['aria-busy'],'true');
    assert.notEqual($('instruction').readOnly,true);
  }
  c.finishVoice(session);assert.equal($('instruction')['aria-busy'],'false');
  assert.equal($('voice-input')['aria-pressed'],'false');assert.equal(c.voiceSession,null);
});
test('voice completion fills text without parsing or dispatching a task',async()=>{
  const {c,$}=voiceHarness(),session={original:'mic'};c.voiceSession=session;
  await c.followVoice(session,{voice_id:'v',status:'completed',text:'把筆放到咖啡盒子'});
  assert.equal($('instruction').value,'把筆放到咖啡盒子');assert.equal($('semantic-source').value,'api');
  assert.equal(c.voiceSession,null);assert.equal($('voice-input').disabled,false);
  assert.doesNotMatch(chunk('async function followVoice(','async function stopVoiceInput('),/\/runs|confirm\(|requestSubmit/);
});
test('host live interim replaces text and polls every 150 ms without duplicate final text',async()=>{
  const {c,$,polls}=voiceHarness(),values=[],session={original:'mic'};c.voiceSession=session;
  Object.defineProperty($('instruction'),'value',{get(){return values.at(-1);},set(v){values.push(v);},configurable:true});
  const jobs=[{status:'recording',text:'把筆',voice_id:'v'},{status:'completed',text:'把筆放下',voice_id:'v'}];
  c.api=async()=>jobs.shift();
  await c.followVoice(session,{status:'recording',text:'把',voice_id:'v'});
  assert.deepEqual(values,['把','把筆','把筆放下','把筆放下']);
  assert.deepEqual(polls,[150,150]);assert.equal(c.voiceSession,null);
});
test('single host microphone start never uses browser audio or a fallback',async()=>{
  const {c,$}=voiceHarness(),posts=[];
  c.navigator={get mediaDevices(){assert.fail('must not access browser microphone');}};
  c.api=async(url,body)=>{posts.push({url,body});return {status:'completed',voice_id:'v',text:'主機逐字稿'};};
  await c.startVoiceInput();
  assert.deepEqual(JSON.parse(JSON.stringify(posts)),[{url:'/api/v2/voice',body:{source:'microphone'}}]);
  assert.equal($('instruction')['aria-busy'],'false');
  assert.doesNotMatch(chunk('function voiceErrorMessage(',"$('voice-input').title="),/getUserMedia|AudioContext|hostVoiceFallback|encodeVoicePCM|closeVoiceAudio|voiceSession\.sequence/);
});
test('network failure displays its reason, stops the job and clears busy without retry',async()=>{
  const {c,$,events}=voiceHarness(),posts=[],session={original:'mic',job:{voice_id:'v'}};c.voiceSession=session;
  c.api=async(url,body)=>posts.push({url,body});
  c.voiceVisual(session,'transcribing');
  await c.failVoice(session,{code:'voice_network_error'});
  assert(events.some(m=>/語音：連線失敗或逾時/.test(m)));
  assert.equal($('instruction')['aria-busy'],'false');assert.equal(c.voiceSession,null);
  assert.equal(posts.length,1);assert.equal(posts[0].body.action,'stop');
});


test('completion refresh has capture feedback but selecting instances remains unobscured',()=>{
  const c=vm.createContext({});vm.runInContext(chunk('function activityState(','function inspectorArtwork('),c);
  assert.equal(c.activityState({status:'completed',completion_capture:{status:'capturing'}},null,null,false).activity,'capture');
  assert.equal(c.activityState({status:'completed',completion_capture:{status:'captured'}},null,null,false).activity,null);
  assert.equal(c.activityState({status:'waiting_instance'},'capture',null,false).activity,null);
});

test('terminal image preference never replaces a new scene with the initial preview',()=>{
  const c=vm.createContext({});vm.runInContext(chunk('function sceneDisplay(','function render('),c);
  const initial={frame_id:'initial'},last={frame_id:'last-step'},final={frame_id:'post-return'};
  assert.equal(c.sceneDisplay({final_preview:final,sequence:{display:last},preview:initial}),final);
  assert.equal(c.sceneDisplay({completion_capture:{status:'failed'},sequence:{display:last},preview:initial}),last);
  assert.equal(c.sceneDisplay({preview:initial}),initial);
});
