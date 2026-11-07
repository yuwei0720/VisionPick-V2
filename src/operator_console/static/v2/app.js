'use strict';
const $ = id => document.getElementById(id);
const labels = {left_of:'左邊',right_of:'右邊',in_front_of:'前面',behind:'後面',above:'上方',below:'下方',near:'附近',on_top_of:'上面',relative:'相對位置',self_relative:'依自身原位平移'};
const shiftLabels={left_of:'左移',right_of:'右移',in_front_of:'前移',behind:'後移'};
const statuses = {idle:'等待任務',parsing:'正在解析整份指令',waiting_answer:'等待補充回答',waiting_confirmation:'等待人工確認',confirmed:'計畫已確認',running:'機械臂執行中',executing:'機械臂執行中',waiting_instance:'請選擇當步物件',waiting_step_recovery:'當步缺件，等待處理',completed:'動作完成',failed:'流程已停止',cancelled:'已取消任務',expired:'確認已逾時',blocked:'計畫未通過檢查'};
const settingNames = {pose_verify_stable_polls:'連續到位次數',pose_verify_position_tolerance_mm:'位置容差（mm）',pose_verify_orientation_tolerance_deg:'姿態容差（°）',motion_timeout_s:'到位逾時（秒）',motion_poll_interval_s:'輪詢間隔（秒）',place_joint_verify_tolerance_deg:'關節容差（°）'};
let token, runId, state, busy=false, settingsRevision, timer, sceneImage, sceneKey, drawEpoch=0, generation=0, objectLabels={}, defaultReady=false;
let bootState, engineeringState, engineeringRevision, lastDisplay, viewRotation=null, directRunId=null, homeRunId=null, stopRequested=false, tableCapture=null, lastQuestionKey=null;
const autoClaims=new Set(), localErrors=[];
let voiceSession=null;
let roiRequest=0,roiPointer=null;
const terminal=new Set(['completed','failed','cancelled','expired','blocked']);
const offlineMotionNotice='⚠️ 尚未偵測到相機或機械臂（離線語意測試模式），無法啟動實機運動。請先連接設備並點擊重新核對。';
const preferenceKey='visionpick_v2_prefs';
const preferenceOptions={rotation:['0','90','180','270'],'confirmation-mode':['once','direct'],'destination-mode':['fixed','strict'],mode:['offline','real'],'semantic-source':['demo','api']};
function readPreferences(){
  try{
    const raw=JSON.parse(localStorage.getItem(preferenceKey)||'{}');
    if(!raw||typeof raw!=='object'||Array.isArray(raw))return {};
    return Object.fromEntries(Object.entries(preferenceOptions).filter(([key,values])=>(typeof raw[key]==='string'||key==='rotation'&&typeof raw[key]==='number')&&values.includes(String(raw[key]))).map(([key])=>[key,String(raw[key])]));
  }catch{return {};}
}
function savePreferences(){
  const values={};
  for(const [key,allowed] of Object.entries(preferenceOptions))if(allowed.includes(String($(key).value)))values[key]=String($(key).value);
  try{localStorage.setItem(preferenceKey,JSON.stringify(values));}catch{/* Storage can be disabled; current-page controls still work. */}
}
function restorePreferences(){
  const values=readPreferences();
  for(const [key,value] of Object.entries(values))$(key).value=value;
  if(values.rotation!==undefined)viewRotation=Number(values.rotation);
  if($('semantic-source').value==='demo')$('mode').value='offline';
  $('confirmation-mode').setAttribute('aria-checked',String($('confirmation-mode').value==='direct'));
}
async function api(path,payload){
  const response=await fetch(path,{method:payload===undefined?'GET':'POST',headers:payload===undefined?{}:{'Content-Type':'application/json','X-VisionPick-Token':token},body:payload===undefined?undefined:JSON.stringify(payload)});
  const data=await response.json();if(!response.ok)throw new Error(data.error||'HTTP '+response.status);return data;
}
function translateError(raw, next=state){
  const reason=String(raw||'');
  if(reason.startsWith('語音：'))return [reason.slice(3),'',''];
  if(/hardware_not_detected/.test(reason))return [offlineMotionNotice,'重新核對','devices'];
  if(/no_transfer_tasks/.test(reason))return ['⚠️ 目前相機畫面中未偵測到符合指令的可抓取物品，請確認物品已放入視野後點擊「重新取像」。','🔄 重新取像','capture'];
  if(/unsupported_visual_reference/.test(reason))return [next?.pending?.question?.message||'請改選相機支援的參照物或已啟用站點。','重新取像','capture'];
  if(/task_plan_v2_request_failed|APIConnectionError|APIStatusError/.test(reason+' '+(next?.error?.type||'')))return ['AI 服務請求未完成，尚未取得可執行計畫。','查看解析紀錄','engineering'];
  if(/semantic_candidate_missing/.test(reason))return ['AI 回覆未包含可用的任務資料，請檢查解析紀錄。','查看解析紀錄','engineering'];
  if(/target_not_detected/.test(reason))return ['畫面中未偵測到指定目標物，請確認物品在視野內。','重新取像','capture'];
  if(/no_eligible_(target|reference)_instance/.test(reason)){
    const role=reason.includes('target')?'目標物':'參考物',field=role==='目標物'?'target':'reference';
    const task=next?.sequence?.steps?.[(next?.sequence?.step_id||1)-1]?.task;
    const seen=next?.sequence?.display?.instances?.some(i=>i.class_name===task?.[field]);
    return [seen?role+'已辨識，但深度或工作範圍未通過檢查。':'畫面中未偵測到指定的'+role+'，請確認物品在視野內。','重新取像','capture'];
  }
  if(/holding_uncertain|empty_gripper/.test(reason))return ['持物狀態不確定，請先在現場確認夾爪空手且機械臂已停止。','執行空手復原','recover'];
  if(/named_landing_occupied|target_occluded/.test(reason))return ['放置區被占用或目標物被遮擋，請移開障礙物。','重新取像','capture'];
  if(/stale_engineering|stale_arrival/.test(reason))return ['設定已被更新，請重新讀取後再儲存。','重新讀取設定','engineering'];
  if(/configuration_changed|stale_plan_version/.test(reason))return ['計畫使用的設定或版本已改變，請重新解析指令。','重新填寫','reset'];
  if(/arrival_timeout/.test(reason))return ['機械臂未在設定時間內通過到位檢查，請檢查現場與回饋。','查看到位紀錄','engineering'];
  const controllerCode = (reason.match(/controller_error[=_:](\d+)/)?.[1]
    ?? next?.sequence?.error?.controller_error
    ?? next?.events?.slice()?.reverse()?.find(e => (e.event === 'dispatch_failed' || e.data?.event === 'dispatch_failed'))?.data?.controller_error);
  if (/dispatch_failed/.test(reason) && Number(controllerCode) === 2) {
    return ['機械臂姿態到達運動學極限或逆解無解（代碼 2），請調整目標位置或姿態。', '開啟工程模式', 'engineering'];
  }
  if(/dispatch_failed|連線失敗|connect|hardware_disabled|real_motion_not_enabled/.test(reason))return ['硬體連線或運動授權未就緒，請核對啟動方式與設備。','開啟工程模式','engineering'];
  if(/ownership_quarantined/.test(reason))return ['設備連線未能確認釋放，請先停止現場操作並檢查工程紀錄。','查看工程紀錄','engineering'];
  if(/scene_snapshot_stale|depth|frame_encode|影像載入失敗/.test(reason))return ['影格已過期或深度資料不足，請重新取像。','重新取像','capture'];
  if(/out_of_range|must|invalid_.*setting|tcp_xy|unknown_engineering/.test(reason))return ['參數格式或數值超出允許範圍，請依工程欄位修正。','檢查參數','engineering'];
  if(/not_registered|not_ready|default_profile|default.*站點/.test(reason))return ['指定站點尚未登錄或未就緒，請選擇有效站點。','管理站點','engineering'];
  if(/api_key_missing/.test(reason))return ['尚未設定 Gemini 金鑰，請先完成本機環境設定。','開啟工程模式','engineering'];
  if(/demo_accepts/.test(reason))return ['離線固定案例只接受畫面上的單步範例，其他指令請切換語意來源。','切換語意來源','engineering'];
  if(/worker_busy|locked|operation_active|run_active/.test(reason))return ['目前工作仍占用設備，請等待結束或取消任務後再操作。','查看工作狀態','status'];
  if(/fresh_pose_read_required/.test(reason))return ['示教前必須取得 60 秒內的六軸角度與 TCP 讀值。','讀取現場位姿','pose'];
  if(/out.*workspace|workspace|outside_workspace/.test(reason))return ['物件或放置位置超出已設定工作範圍，請調整位置。','重新取像','capture'];
  if(/^[\u3400-\u9fff]/u.test(reason))return [reason,'查看紀錄','engineering'];
  return ['本次操作未通過檢查，請查看工程紀錄中的具體原因。','查看工程紀錄','engineering'];
}
function error(message){
  $('error').hidden=!message;
  if(!message)return;
  const [text,actionName,actionKey]=translateError(message);
  $('error-message').textContent=text;
  $('error-action').textContent=actionName;$('error-action').hidden=!actionName;$('error-action').dataset.action=actionKey;
  if(localErrors.at(-1)?.reason!==message)localErrors.push({at:new Date().toISOString(),reason:String(message)});
  updateRecord();
}
async function guarded(work){
  if(busy||voiceSession)return;
  busy=true;generation++;error('');syncButtons();
  try{await work();}catch(e){error(e.message);}
  finally{busy=false;syncButtons.activity=null;syncButtons();maybeAutoConfirm();}
}
function button(text,click,disabled=false){const b=document.createElement('button');b.type='button';b.textContent=text;b.disabled=disabled;b.className='v2-choice';b.onclick=click;return b;}
async function bootstrap(){
  const b=await api('/api/v2/bootstrap');token=b.session;
  objectLabels=b.object_labels||{};
  settingsRevision=b.settings.revision;
  defaultReady=b.catalog.default_ready;
  $('settings-fields').replaceChildren();
  for(const [key,value] of Object.entries(b.settings.values)){
    const label=document.createElement('label'); label.textContent=settingNames[key];
    const input=document.createElement('input');input.type='number';input.required=true;input.min=key==='pose_verify_stable_polls'?'1':'0.001';input.step=key==='pose_verify_stable_polls'?'1':'any';input.name=key;input.value=value;
    const source=document.createElement('small');source.className='v2-note';source.textContent=({ui_override:'已儲存設定',config:'專案設定',program_default:'程式預設'})[b.settings.sources[key]];
    label.append(input,source);$('settings-fields').append(label);
  }
  $('default-station').replaceChildren(...b.catalog.stations.filter(s=>s.ready).map(s=>{const o=document.createElement('option');o.value=s.profile_id;o.textContent=s.name;o.selected=s.profile_id===b.catalog.default_profile_id;return o;}));
  $('current-default-station').textContent=`目前預設：${b.catalog.default_station || '未設定／待重驗'}；變更供新計畫使用。`;
  $('readiness').textContent=`有效站點 ${b.catalog.ready_count}／${b.catalog.stations.length}；桌面：${b.table.ready?'已核對':'待建立／重驗'}；Start 六軸角度：${b.start_ready?'已設定':'待設定'}；預設站點：${b.catalog.default_station || '未設定／待重驗'}。${b.holding_uncertain?'持物狀態不確定，需人工確認。':''}`;
  $('mode').querySelector('[value=real]').disabled=false;
  if(typeof renderBootstrap==='function')renderBootstrap(b);
  if(!runId&&b.active_run){runId=b.active_run;await poll();}
}

function activeTask(){return Boolean(state&&!terminal.has(state.status));}
// One visual activity spans the POST wait and the subsequent worker polling.
function activityState(next,request,home,selecting){
  const parsing=request==='parsing'||request==='capture-parsing'||next?.status==='parsing';
  const camera=next?.devices?.camera;
  const capturing=request==='capture-parsing'||(next?.status==='parsing'&&next.mode==='real'&&camera?.detected===true&&camera?.count!==0);
  let activity=home||request==='home'?'home':request==='capture'?'capture':parsing?(capturing?'capture':'parsing'):['running','executing'].includes(next?.status)?(next.operation==='preview'?'capture':'moving'):null;
  if(next?.completion_capture?.status==='capturing')activity='capture';
  if(selecting||next?.status==='waiting_instance')activity=null;
  return {parsing,capturing,activity};
}
function inspectorArtwork(parsing,visible){
  const art=$('inspector-artwork'),gif=$('inspector-gif'),svg=$('inspector-svg');
  const reduced=globalThis.matchMedia?.('(prefers-reduced-motion: reduce)').matches;
  const slot=parsing?'parsing':'idle',key=slot+':'+Boolean(visible)+':'+Boolean(reduced);
  if(inspectorArtwork.key===key)return;
  inspectorArtwork.key=key;
  const epoch=(inspectorArtwork.epoch||0)+1;inspectorArtwork.epoch=epoch;
  art.dataset.slot=slot;art.dataset.artwork='svg';gif.hidden=true;svg.hidden=false;
  gif.onload=null;gif.onerror=null;gif.removeAttribute('src');
  if(!visible||reduced||typeof Image!=='function')return;
  // Inspector slots own their requests; a missing right-side GIF never borrows
  // the canvas mascot, and a late idle response cannot replace parsing artwork.
  const url='/v2/assets/mascot_inspector_'+slot+'.gif',probe=new Image();
  const current=()=>epoch===inspectorArtwork.epoch;
  const fallback=()=>{if(!current())return;gif.hidden=true;svg.hidden=false;art.dataset.artwork='svg';};
  probe.onload=()=>{
    if(!current())return;
    gif.onload=()=>{if(!current())return;gif.hidden=false;svg.hidden=true;art.dataset.artwork='gif';};
    gif.onerror=fallback;gif.src=url;
  };
  probe.onerror=fallback;probe.src=url;
}
function syncActivity(){
  const feedback=activityState(state,syncButtons.activity,homeRunId,$('table-roi').hidden===false&&!tableCapture);
  const {parsing,capturing,activity}=feedback,parse=$('parse');
  if(parse.classList.contains('is-loading')!==parsing){
    parse.classList.toggle('is-loading',parsing);
    parse.innerHTML=parsing?'<span class="button-spinner" aria-hidden="true"></span>AI 解析中…':'<svg aria-hidden="true"><use href="#i-spark"/></svg>AI 解析任務';
  }
  parse.setAttribute('aria-busy',String(parsing));
  $('inspector-panel').setAttribute('aria-busy',String(parsing));
  if(parsing){
    $('status').dataset.state='parsing';$('status').dataset.capture=String(capturing);
    $('status').textContent=capturing?'📸 前置取像與 AI 解析中…':'✨ 正在解析整份指令…';
    renderPlan({status:'parsing'});spotlight('parsing');
    $('decision-title').textContent='AI 正在理解整份指令';
    $('decision-note').textContent='解析完成後會顯示明細；需要補充的內容會在這裡提示。';
    renderMotion({status:'parsing'});
  }else if($('plan-empty').classList.contains('is-parsing')){
    renderPlan(state||{status:'idle'});
    if(!state){$('status').dataset.state='idle';$('status').textContent='等待任務';$('decision-title').textContent='準備好，就交給工作台';$('decision-note').textContent='填入指令並解析，這裡會顯示需要你確認的內容。';renderMotion({status:'idle'});spotlight('idle');}
  }
  const overlay=$('canvas-activity-overlay');overlay.hidden=!activity;
  if(activity){
    overlay.dataset.activity=activity;
    const text={capture:['擷取畫面中','回 Start 後更新 RGB-D 視覺定位'],home:['正在返回 Start','張開夾爪後移至待命姿態'],moving:['抓放任務執行中',$('motion-detail').textContent],parsing:['AI 解析中','辨識目標、位置與任務順序']};
    const [title,detail]=state?.completion_capture?.status==='capturing'?['更新完成後畫面','擷取最新現場影像']:text[activity];
    $('activity-title').textContent=title;$('activity-detail').textContent=' · '+detail;
    overlay.setAttribute('aria-label',title+'，'+detail);overlay.title=title+'，'+detail;
  }
  inspectorArtwork(parsing,!$('plan-empty').hidden);
  $('execution-mode-hint').textContent=$('confirmation-mode').value==='direct'?'解析無歧義後直接執行':'確認一次後執行';
  $('execution-mode-hint').dataset.direct=String($('confirmation-mode').value==='direct');
}
function syncButtons(){
  const active=activeTask(),working=busy||Boolean(state?.worker_busy)||Boolean(homeRunId)||Boolean(typeof voiceSession!=='undefined'&&voiceSession);
  const selfFrame=active&&state?.pending?.candidate?.tasks?.some(t=>t.destination_type==='self_relative');
  $('rotation').disabled=working||Boolean(selfFrame);
  $('rotation').title=selfFrame?'計畫方向採工作台 '+state.direction_frame?.rotation_deg+'°；顯示角度保留你的設定。':'';
  $('parse').disabled=working||active||Boolean(typeof tableCapture!=='undefined'&&tableCapture);
  $('cancel').disabled=working||!active;
  $('confirm').disabled=working||state?.status!=='waiting_confirmation'||state?.read_only||hardwareUnavailable(state);
  if(state?.status==='waiting_confirmation')$('decision-note').textContent=hardwareUnavailable(state)?offlineMotionNotice:selfFrame?'自身位移方向採工作台 '+state.direction_frame?.rotation_deg+'°，目前顯示 '+viewRotation+'°；請依計畫方向核對中心位移距離。':'請核對每一步目標物、參考方向與放置站點。';
  $('refresh-scene').disabled=working||Boolean(typeof tableCapture!=='undefined'&&tableCapture)||(active&&!['waiting_instance','waiting_answer','waiting_step_recovery'].includes(state?.status));
  $('save-settings').disabled=working||active;
  $('save-engineering').disabled=working||(active&&state?.status!=='waiting_confirmation');
  for(const control of (document.querySelectorAll?.('.placement-control select,.placement-control input')||[]))control.disabled=working||state?.status!=='waiting_confirmation';
  $('return-home').disabled=working||active||!bootState?.hardware_enabled||!bootState?.motion_enabled;
  for(const id of ['default-station','destination-mode','confirmation-mode','semantic-source','mode'])$(id).disabled=working||active;
  for(const b of $('choices').querySelectorAll('button'))b.disabled=working||b.dataset.ineligible==='true';
  for(const id of ['gripper-open','gripper-close','read-pose','teach-station','station-metadata','station-activate','station-disable','station-delete','table-capture','table-auto','roi-start','teaching-enter','teaching-exit'])$(id).disabled=working||active;
  if(typeof tableCapture!=='undefined'){ $('roi-save').disabled=working||!tableCapture?.preview||tableCapture.kind!=='station';$('table-roi-save').disabled=working||!tableCapture?.preview||tableCapture.kind!=='table';}
  syncActivity();
}
function objectName(name){return objectLabels[name]||({table:'桌面',coffee_box:'咖啡盒子',battery:'電池',screwdriver:'螺絲起子',pen:'筆',eraser:'橡皮擦',refill_box:'筆芯盒'})[name]||name||'待補物件';}
function renderBootstrap(b){
  const first=!bootState;bootState=b;
  if(state)state.devices=b.devices;
  $('table-status').textContent=b.table.ready?'桌面基準：'+Number(b.table.z_mm).toFixed(1)+' mm':'桌面基準：待量測';
  const deviceLabel=d=>d?.active?'已連線':d?.detected===true&&d?.count!==0?'已偵測（待命）':'未偵測到裝置（離線）';
  $('device-status').textContent='手臂：'+deviceLabel(b.devices?.robot)+'／相機：'+deviceLabel(b.devices?.camera);
  if(first&&b.hardware_enabled&&b.motion_enabled){$('mode').value='real';$('semantic-source').value='api';}
  if(first)restorePreferences();
  if(viewRotation===null&&b.direction_frame){viewRotation=b.direction_frame.rotation_deg;$('rotation').value=String(viewRotation);}
  sourceNote();
  syncButtons();
}
function sourceNote(){
  const demo=$('semantic-source').value==='demo';
  if(!demo&&bootState?.controlled_semantic){$('source-badge').textContent='受控語意替身／離線驗收';$('source-note').textContent='本次使用受控語意與合成影像，未呼叫真 API 或連線實體設備。';return;}
  $('source-badge').textContent=demo?'離線固定案例':($('mode').value==='offline'?'AI 解析／離線演練':'Gemini／實機抓放');
  $('source-note').textContent=demo?'固定案例使用合成影像與受控回饋，不代表實物抓放成功。':'整份指令呼叫一次 Gemini；本地補值與點框選取不再呼叫模型。';
}
function renderPlan(next){
  const parsing=next.status==='parsing';
  $('plan-empty').classList.toggle('is-parsing',parsing);$('plan-skeleton').hidden=!parsing;
  $('plan-empty-title').textContent=parsing?'AI 正在理解指令並拆解步驟…':'從指令，到每一步的意圖';
  $('plan-empty-description').textContent=parsing?'正在整理目標物、目的地與執行順序，請稍候。':'目標物、目的地與參考方向，會在這裡逐步展開，供你核對。';
  if(parsing){$('plan').hidden=true;$('plan-empty').hidden=false;$('plan-count').textContent='解析中…';renderPlan.lastKey=null;return;}
  const rows=next.sequence?.steps||next.pending?.candidate?.tasks?.map((task,i)=>({task,step_id:i+1}))||[];
  const key=JSON.stringify([next.run_id,next.status,rows.map(r=>[r.step_id,r.task,r.status]),next.pending?.defaults,next.sequence?.step_id,next.pending?.version,next.placement_previews]);
  if(renderPlan.lastKey===key)return;renderPlan.lastKey=key;
  $('task-rows').replaceChildren();$('plan').hidden=!rows.length;$('plan-empty').hidden=Boolean(rows.length);
  $('plan-count').textContent=rows.length?rows.length+' / 20 步':'等待解析';
  for(const row of rows){
    const t=row.task||{},tr=document.createElement('tr'),index=row.step_id;
    tr.className=index===next.sequence?.step_id&&['running','executing','waiting_instance'].includes(next.status)?'active-task':'';
    const automatic=next.pending?.defaults?.some(d=>d.task_index===index);
    const self=t.destination_type==='self_relative';
    const cells=[['#'+index,row.status==='skipped'?'已跳過':''],[objectName(t.target),t.target||'待補'],[t.destination_type==='named'?(automatic?'自動預設站點':'具名站點'):self?'自身位移':t.destination_type==='relative'?'相對位置':'待補基準',''],[t.destination_type==='named'?(t.destination_name||'待補站點'):self?'自身原位':objectName(t.reference),self?'各步抓取前當幀':t.destination_type==='relative'?(t.reference||'待補'):''],[self?(shiftLabels[t.relation]||'待補方向')+(Number.isFinite(t.displacement_mm)?' '+t.displacement_mm+' mm':' · 待補距離'):t.relation?(t.relation==='at_initial'?'原始位置':labels[t.relation]||t.relation):'—',self?'中心位移':(t.relation||'')+(Number.isFinite(t.gap_mm)?' · 間距 '+t.gap_mm+' mm':'')]];
    cells.forEach(([title,sub],i)=>{const td=document.createElement('td'),strong=document.createElement(i===4&&t.relation?'span':'strong');strong.textContent=title;if(i===4&&t.relation)strong.className='relation-tag '+(t.relation==='on_top_of'?'top':t.relation==='near'?'near':'');td.append(strong);if(sub){const small=document.createElement('small');small.textContent=sub;td.append(small);}tr.append(td);});
    if(next.run_id)renderOrientationControl(tr,t,index,next);
    $('task-rows').append(tr);
  }
  $('plan-note').textContent=rows.length>4?'共 '+rows.length+' 步；可在此表格內捲動核對完整計畫。':next.pending?.defaults?.length?'省略的目的地已帶入目前預設站點；明示位置優先。':'每一步保留目標物、目的地與參考方向，供你核對。';
}
function renderOrientationControl(tr,task,index,next){
  const td=document.createElement('td'),wrap=document.createElement('div');wrap.className='placement-control';
  const preview=next.placement_previews?.find(p=>p.step_id===index),settings=next.pending?.settings||{};
  const value=settings.placement_orientation_overrides?.[String(index)]||{policy:'default',rz_deg:-45};
  const label=document.createElement('label');label.className='sr-only';label.textContent='第 '+index+' 步放置朝向';
  const select=document.createElement('select');select.setAttribute('aria-label',label.textContent);select.dataset.step=String(index);
  const choices=[['default','依工程預設'],['keep_target','保留自身角']];
  if(task.destination_type==='relative'&&task.relation!=='at_initial')choices.push(['align_reference','平行參照物']);
  choices.push(['fixed_deg','自訂角度']);
  for(const [key,text] of choices){const option=document.createElement('option');option.value=key;option.textContent=text;select.append(option);}
  select.value=value.policy;
  const angle=document.createElement('input');angle.type='number';angle.min='-180';angle.max='180';angle.step='any';angle.value=Number.isFinite(value.rz_deg)?value.rz_deg:-45;
  angle.setAttribute('aria-label','第 '+index+' 步自訂 RZ 角度（度）');angle.hidden=select.value!=='fixed_deg';
  const detail=document.createElement('small');detail.className=preview?.feasible===false?'placement-warning':'';
  detail.textContent=(Number.isFinite(preview?.rz_deg)?preview.rz_deg.toFixed(1)+'°':'等待幾何')+(preview?.feasible===false?' · ! 放置檢查未通過':preview?.predicted_scene?' · 後續預估':'');
  detail.title=preview?.reason||'每步執行前會重新取像檢查；角度為機械臂工具 RZ。';
  let angleTimer;
  const apply=()=>{
    clearTimeout(angleTimer);
    angle.hidden=select.value!=='fixed_deg';angle.required=select.value==='fixed_deg';
    if(angle.required&&(!angle.value.trim()||!angle.checkValidity())){angle.reportValidity();return;}
    // Editing revokes this run's direct-mode claim; the revised plan needs a click.
    directRunId=null;
    guarded(()=>action('placement_orientation',{version:state.pending.version,step_id:index,policy:select.value,rz_deg:select.value==='fixed_deg'?Number(angle.value):null}));
  };
  select.onchange=apply;angle.onchange=apply;
  angle.oninput=()=>{clearTimeout(angleTimer);if(angle.value.trim()&&angle.checkValidity())angleTimer=setTimeout(()=>{if(state?.run_id===next.run_id&&state?.pending?.version===next.pending?.version)apply();},350);};
  select.disabled=angle.disabled=next.status!=='waiting_confirmation'||Boolean(next.worker_busy)||next.read_only;
  wrap.append(label,select,angle,detail);td.append(wrap);tr.append(td);
}
function spotlight(status){
  let ids=['waiting_answer','waiting_step_recovery'].includes(status)?['decision-panel']:status==='waiting_instance'?['vision-panel']:status==='waiting_confirmation'?['inspector-panel','decision-panel']:['running','executing'].includes(status)?['motion-panel']:['instruction-panel'];
  if(status==='parsing')ids=$('status').dataset.capture==='true'?['inspector-panel','vision-panel']:['inspector-panel'];
  for(const panel of document.querySelectorAll('.spotlight-panel')){panel.classList.toggle('spotlight-active',ids.includes(panel.id));panel.classList.toggle('spotlight-dimmed',!ids.includes(panel.id));}
  $('emergency-stop').classList.toggle('spotlight-active',['running','executing'].includes(status));
}
function updateRecord(){
  $('record').textContent=JSON.stringify({run_id:state?.run_id,error:state?.sequence?.error||state?.error,cleanup_error:state?.cleanup_error,events:state?.events,pending_history:state?.pending?.history,step_history:state?.sequence?.history,ui_errors:localErrors},null,2);
}
function recoveryPayload(q){return {run_id:q.run_id,step_id:q.step_id,frame_id:q.frame_id,plan_version:q.plan_version};}
function recoveryNotice(q){
  const name=objectName(q.class_name),detail=q.detected_but_ineligible?'「'+name+'」已辨識，但深度或工作範圍未通過檢查':'畫面中未偵測到「'+name+'」';
  $('question').textContent='⚠️ 第 '+q.step_id+' 步：'+detail+'，機械臂已安全停於初始位置，請選擇處理方式：';
  $('decision-note').textContent=q.skip_allowed?'可重新取像、跳過此步或中止整批任務。':'交換任務不可單步跳過；請重新取像或中止任務。';
}
function renderRecovery(q){
  $('choices').append(button('🔄 重新取像再試一次',()=>guarded(()=>action('retry_step',recoveryPayload(q)))));
  const skip=button('⏭️ 跳過此步，繼續搬下一個',()=>guarded(()=>action('skip_step',recoveryPayload(q))),!q.skip_allowed);
  skip.dataset.ineligible=String(!q.skip_allowed);skip.title=q.skip_allowed?'':'交換任務不可單步跳過';$('choices').append(skip);
  $('choices').append(button('✕ 中止任務',()=>guarded(()=>action('cancel'))));
}
function sceneDisplay(next){return next.final_preview||next.sequence?.display||next.preview;}
function render(next){
  state=next;
  if(next.direction_frame&&viewRotation===null){
    viewRotation=next.direction_frame.rotation_deg;$('rotation').value=String(viewRotation);
  }
  const preview=next.operation==='preview',status=preview&&next.status==='completed'?'idle':next.status;
  $('status').textContent=statuses[status]||'等待任務';$('status').dataset.state=status;
  if(next.connections){const names={connected:'已連線',detected:'已偵測（待命）',connecting:'連線中',disconnected:'未偵測到裝置（離線）',unknown:'狀態待查',simulated:'離線演練'};$('device-status').textContent='手臂：'+names[next.connections.robot]+'／相機：'+names[next.connections.camera];}
  renderPlan(next);spotlight(status);
  const q=next.sequence?.question||next.pending?.question;
  const qkey=q?JSON.stringify([next.run_id,next.status,q.version,q.step_id,q.frame_id,q.role,q.slot,q.skip_allowed]):'';
  $('question').hidden=!q;
  $('question').textContent=next.status==='waiting_instance'?'偵測到多個'+objectName(q?.class_name)+'，請選擇'+(q?.role==='reference'?'參考物':'目標物')+'；選取後將繼續執行。':q?.message||'';
  $('selection-guide').hidden=next.status!=='waiting_instance';
  if(next.status==='waiting_instance')$('selection-guide').textContent='偵測到多個同類物品，請點擊畫面框線選擇'+(q?.role==='reference'?'參考物':'目標物');
  $('answer-form').hidden=next.status!=='waiting_answer';
  if(qkey!==lastQuestionKey){
  $('choices').replaceChildren();
  if(next.status==='waiting_step_recovery'){
    renderRecovery(q);
  }else if(next.status==='waiting_instance'){
    for(const [i,choice] of (q?.choices||[]).entries()){
      const b=button((i+1)+'. '+objectName(choice.class_name)+(choice.eligible?'':'（資料未通過）'),()=>chooseInstance(choice.instance_id,q),!choice.eligible);
      b.dataset.ineligible=String(!choice.eligible);b.dataset.instanceId=choice.instance_id;$('choices').append(b);
    }
  }else if(next.status==='waiting_answer'){
    for(const value of q?.choices||[])$('choices').append(button(q.slot==='displacement_mm'?value+' mm':labels[value]||objectName(value),()=>answer(value,q)));
    for(const alternative of q?.alternatives||[])$('choices').append(button(alternative.label,()=>answer(alternative.value,q)));
    if(q?.refresh_scene)$('choices').append(button('🔄 重新取像',()=>guarded(refreshPreview)));
  }
  const input=$('answer-value');input.value='';input.type=q?.input_type==='number'?'number':'text';
  input.step='any';input.required=true;input.setAttribute('aria-label',q?.slot==='displacement_mm'?'自身位移距離（毫米，大於 0）':'補充答案');
  input.setAttribute('aria-describedby','question');input.placeholder=q?.slot==='displacement_mm'?'輸入毫米數，例如 30':'輸入答案';
  input.removeAttribute('min');input.setCustomValidity('');lastQuestionKey=qkey;
  }
  const display=sceneDisplay(next);
  if(display){lastDisplay=display;$('scene-title').textContent=next.final_preview?'任務完成後畫面':preview?'相機視覺':next.status==='completed'?'最後有效畫面':'第 '+(next.sequence?.step_id||1)+' 步畫面';drawScene(display,q);renderCoordinates(display,next);}
  else if(lastDisplay){renderCoordinates(lastDisplay,next);$('frame-caption').textContent='前次影格；執行時重新取像'+(lastDisplay.synthetic?' · 合成資料':'');}
  $('decision-panel').classList.toggle('answering',['waiting_answer','waiting_instance','waiting_step_recovery'].includes(next.status));
  const titles={waiting_confirmation:'核對明細後，開始執行',waiting_answer:'補充一個答案，就能繼續',waiting_instance:'直接在畫面選擇物件',waiting_step_recovery:'當步缺件，請選擇處理方式',running:preview?'Start 復位後重新取像':'工作台正在執行',executing:preview?'Start 復位後重新取像':'工作台正在執行',completed:preview?'畫面已更新，可以安排任務':'本次動作完成',failed:'流程已停止，請查看原因',cancelled:'已取消，可以重新安排'};
  $('decision-title').textContent=titles[next.status]||'準備好，就交給工作台';
  $('decision-state').textContent=next.mode==='real'?'實機抓放':'離線演練';
  $('decision-note').textContent=next.status==='waiting_confirmation'?'請核對每一步目標物、參考方向與放置站點。':['running','executing'].includes(next.status)?'每步回 Start、重新取像；需要時可隨時停止後續動作。':next.status==='completed'&&!preview?'控制流程已完成，請依現場觀察記錄物品結果。':'填入指令並解析，這裡會顯示需要你確認的內容。';
  if(next.completion_capture?.status==='failed')$('decision-note').textContent='抓放流程已完成；完成後取像失敗，保留最後有效影格。請按刷新重新取像。';
  if(next.status==='waiting_step_recovery')recoveryNotice(q);
  $('outcome-panel').hidden=next.status!=='completed'||preview;
  $('outcome-status').textContent=next.manual_outcome?'已記錄人工結果：'+({success:'成功',failure:'失敗',unverified:'未驗收'})[next.manual_outcome.outcome]:'';
  renderMotion(next);updateRecord();
  const reason=next.sequence?.error?.reason||next.error?.reason||(next.mode==='real'&&Array.isArray(next.semantic_scene?.visible_objects)&&next.pending?.issues?.some(i=>i.code==='no_transfer_tasks')?'no_transfer_tasks':null);if(reason)error(reason);
  syncButtons();maybeAutoConfirm();
}
function renderMotion(next){
  const steps=next.sequence?.steps||[],n=next.sequence?.step_id||1;
  $('step-count').textContent=steps.length?'Step '+n+' / '+steps.length:next.operation==='preview'?'Start → 取像':'等待任務';
  const events=next.events||[],latest=[...events].reverse().find(e=>(e.data||e).label||['start_verified_before_capture','object_released_at_place'].includes(e.event)),data=latest?.data||latest||{};
  const label=data.label||'',event=latest?.event||'';
  let phase=0;
  if(/initialization|start_/.test(label+' '+event))phase=1;
  if(next.status==='waiting_instance'||next.sequence?.display||next.preview)phase=2;
  if(/pick_pregrasp/.test(label))phase=3;
  if(/grasp_descent|grasp_close|grasp_lift/.test(label))phase=4;
  if(/place_pre_place/.test(label))phase=5;
  if(/place_descent|place_release|place_retreat|object_released/.test(label+' '+event))phase=6;
  if(next.status==='completed'&&next.operation!=='preview')phase=7;
  [...$('motion-stepper').children].forEach((li,i)=>{li.classList.toggle('done',i+1<phase);li.classList.toggle('current',i+1===phase);});
  const transfer=[...events].reverse().find(e=>e.event==='transfer_selected');
  const method=transfer?.data?.transfer_mode||transfer?.transfer_mode;
  const methods={preferred_ik:'Preferred IK',standard_ik:'標準 IK',taught_joint:'配對示教角',cartesian_fallback:'笛卡兒備援',cartesian:'笛卡兒備援'};
  const stageNames={initialization_before_capture:'Start 復位',pick_pregrasp:'移至抓取上方',grasp_descent:'直線下壓夾取',grasp_close:'夾爪閉合',grasp_lift:'抓取後抬升',place_pre_place:'移至放置上方',place_descent:'下壓放置',place_retreat:'放置後退回'};
  const friendly=Object.entries(stageNames).find(([key])=>label.startsWith(key))?.[1];
  $('motion-detail').textContent=(steps.length?'Step '+n+'/'+steps.length+'　':'')+(friendly||statuses[next.status]||'等待任務')+'　｜　路徑：'+(methods[method]||'待選用');
}
function xyz(point){return Array.isArray(point)&&point.length===3&&point.every(Number.isFinite)?'X '+point[0].toFixed(1)+'　Y '+point[1].toFixed(1)+'　Z '+point[2].toFixed(1)+' mm':'X —　Y —　Z — mm';}
function renderCoordinates(display,next){
  const list=display.instances||[],task=next.sequence?.steps?.[(next.sequence?.step_id||1)-1]?.task||next.pending?.candidate?.tasks?.[0]||{target:'eraser',reference:'pen'},selected=next.sequence?.selected||{};
  for(const role of ['target','reference']){
    const matches=list.filter(i=>i.class_name===task[role]),original=role==='reference'&&task.relation==='at_initial',item=original?(selected.initial_reference||null):list.find(i=>i.instance_id===selected[role])||(matches.length===1?matches[0]:null);
    $(role+'-name').textContent=item?objectName(item.class_name):matches.length>1?'待點選（'+matches.length+' 件）':role==='reference'&&task.destination_type==='named'?(task.destination_name||'具名站點'):'等待辨識';
    $(role+'-coordinates').textContent=xyz(item?.xyz_mm);
    if(role==='target')$('target-confidence').textContent='信心度 '+(Number.isFinite(item?.confidence)?(item.confidence*100).toFixed(1)+'%':'—');
    else $('reference-height').textContent=original?'初始位置目標；空位依本步新影格檢查':'RGB-D 頂部高度 '+(Number.isFinite(item?.top_z_mm)?item.top_z_mm.toFixed(1)+' mm':'—');
  }
  $('detection-list').replaceChildren();
  list.forEach(i=>{const span=document.createElement('span');span.textContent=objectName(i.class_name)+'　'+xyz(i.xyz_mm);span.title=span.textContent;$('detection-list').append(span);});
  $('frame-source').textContent=display.synthetic?'受控合成影像':'RGB-D 相機影格';
  $('frame-caption').textContent=display.synthetic?'合成影像 · 離線驗收資料':'RGB-D 視覺與物件定位';
  $('frame-size').textContent=display.width+' × '+display.height;
}
function rotatePoint(p,w,h,r){return r===90?[p[1],w-p[0]]:r===180?[w-p[0],h-p[1]]:r===270?[h-p[1],p[0]]:p;}
function displayData(d){return {...d,rotation_deg:viewRotation===null?d.rotation_deg:viewRotation};}
function polygonFor(choice,d){return choice.corners_px.map(p=>rotatePoint(p,d.width,d.height,d.rotation_deg));}
function paint(original,q){
  const d=displayData(original),canvas=$('scene'),r=d.rotation_deg;
  canvas.width=r===90||r===270?d.height:d.width;canvas.height=r===90||r===270?d.width:d.height;
  $('rotation').value=String(r);
  const c=canvas.getContext('2d');c.save();
  if(r===90){c.translate(0,d.width);c.rotate(-Math.PI/2);}else if(r===180){c.translate(d.width,d.height);c.rotate(Math.PI);}else if(r===270){c.translate(d.height,0);c.rotate(Math.PI/2);}
  c.drawImage(sceneImage,0,0,d.width,d.height);c.restore();
  const selecting=state?.status==='waiting_instance'&&Array.isArray(q?.choices);
  const choices=selecting?q.choices:d.instances||[];
  choices.forEach((choice,i)=>{const pts=polygonFor(choice,d),selected=Object.values(state?.sequence?.selected||{}).includes(choice.instance_id);c.beginPath();pts.forEach((p,n)=>n?c.lineTo(...p):c.moveTo(...p));c.closePath();c.strokeStyle=choice.eligible?(selected?'#0D9488':'#2563EB'):'#D97706';c.lineWidth=selected?4:2;c.stroke();const text=selecting?String(i+1):objectName(choice.class_name);c.font='600 13px Microsoft JhengHei, sans-serif';const width=c.measureText(text).width+14,x=Math.max(0,Math.min(pts[0][0],canvas.width-width)),y=Math.max(21,pts[0][1]);c.fillStyle=choice.eligible?'#2563EB':'#D97706';c.fillRect(x,y-21,width,21);c.fillStyle='#fff';c.fillText(text,x+7,y-6);});
  if(!tableCapture&&state?.placement_previews?.length&&$('show-placement-preview').checked)paintPlacementPreviews(c,d,state.placement_previews,$('show-placement-envelope').checked);
  if(tableCapture?.rect)paintRoi(c,tableCapture.rect);
  canvas.hidden=false;$('scene-empty').hidden=true;
}
function previewPoint(point,d){return rotatePoint(point,d.width,d.height,d.rotation_deg);}
function paintPlacementPreviews(c,d,previews,showEnvelope){
  const colors=['#0F766E','#7C3AED','#0369A1','#BE185D'];
  const scale=Math.max(1,c.canvas.width/(c.canvas.getBoundingClientRect?.().width||c.canvas.width));
  const valid=p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite);
  function polygon(points,color,fill,dash){
    if(!Array.isArray(points)||points.length!==4||!points.every(valid))return;
    const pts=points.map(p=>previewPoint(p,d));c.beginPath();pts.forEach((p,i)=>i?c.lineTo(...p):c.moveTo(...p));c.closePath();
    c.fillStyle=fill;c.fill();c.setLineDash(dash);c.strokeStyle=color;c.lineWidth=2.5;c.stroke();c.setLineDash([]);
  }
  function arrow(a,b,color,curved){
    if(!valid(a)||!valid(b))return;
    a=previewPoint(a,d);b=previewPoint(b,d);const dx=b[0]-a[0],dy=b[1]-a[1],length=Math.hypot(dx,dy);if(length<1)return;
    const control=[(a[0]+b[0])/2-dy*.12,(a[1]+b[1])/2+dx*.12];c.beginPath();c.moveTo(...a);
    if(curved)c.quadraticCurveTo(...control,...b);else c.lineTo(...b);
    c.strokeStyle=color;c.lineWidth=2;c.stroke();
    const theta=Math.atan2(b[1]-(curved?control[1]:a[1]),b[0]-(curved?control[0]:a[0]));
    c.beginPath();c.moveTo(...b);c.lineTo(b[0]-8*Math.cos(theta-.45),b[1]-8*Math.sin(theta-.45));c.lineTo(b[0]-8*Math.cos(theta+.45),b[1]-8*Math.sin(theta+.45));c.closePath();c.fillStyle=color;c.fill();
  }
  c.save();
  for(const p of previews){
    if(p.frame_id&&d.frame_id&&p.frame_id!==d.frame_id)continue;
    // Rejected placements are NOT planned destinations. Reason stays in
    // the step detail; never draw an unsafe diagnostic candidate as a target.
    if(p.feasible!==true||!valid(p.center_px))continue;
    const color=p.feasible===false?'#BE123C':colors[(p.step_id-1)%colors.length];c.globalAlpha=p.predicted_scene ? .66 : .9;
    polygon(p.polygon_px,color,p.feasible===false?'#FFE4E680':color+'24',p.feasible===false?[3,4]:[9,5]);
    // The server supplies the rotated clearance vertices; retain their order.
    if(showEnvelope&&p.envelope_px?.length===4)polygon(p.envelope_px,color,'#FFFFFF00',[2,5]);
    arrow(p.source_center_px,p.center_px,color,true);
    if(p.direction_px?.length===2)arrow(p.direction_px[0],p.direction_px[1],color,false);
    const center=previewPoint(p.center_px,d),text=(p.feasible===false?'! ':'')+'Step '+p.step_id+': '+objectName(p.target)+'預計落點 · RZ '+(Number.isFinite(p.rz_deg)?p.rz_deg.toFixed(1)+'°':'—')+(p.predicted_scene?'（前步完成後）':'');
    c.font='600 '+(12*scale)+'px Microsoft JhengHei, sans-serif';const width=Math.min(c.measureText(text).width+12*scale,c.canvas.width),x=Math.max(0,Math.min(center[0]+12,c.canvas.width-width)),y=Math.max(22*scale,Math.min(center[1]-16*scale-(p.step_id-1)%3*22*scale,c.canvas.height-6*scale));
    c.globalAlpha=1;c.fillStyle='#FFFFFFEB';c.fillRect(x,y-17*scale,width,22*scale);c.fillStyle=color;c.fillText(text,x+6*scale,y-2*scale,width-10*scale);
  }
  c.restore();
}
function drawScene(d,q){
  const key=d.frame_id+':'+d.image_url;
  if(sceneKey===key&&sceneImage?.complete){paint(d,q);return;}
  sceneKey=key;const epoch=++drawEpoch,img=new Image();
  img.onload=()=>{if(epoch!==drawEpoch)return;sceneImage=img;paint(d,state?.sequence?.question);};
  img.onerror=()=>error('影像載入失敗，請重新取像。');img.src=d.image_url;
}
function contains(point,poly){let hit=false;for(let i=0,j=poly.length-1;i<poly.length;j=i++){const a=poly[i],b=poly[j];if((a[1]>point[1])!==(b[1]>point[1])&&point[0]<(b[0]-a[0])*(point[1]-a[1])/(b[1]-a[1])+a[0])hit=!hit;}return hit;}
$('scene').addEventListener('click',e=>{
  if(busy||!lastDisplay)return;
  const box=e.currentTarget.getBoundingClientRect(),point=[(e.clientX-box.left)*e.currentTarget.width/box.width,(e.clientY-box.top)*e.currentTarget.height/box.height];
  if(tableCapture)return;
  const q=state?.sequence?.question,d=state?.sequence?.display;if(!q||state.status!=='waiting_instance')return;
  const hits=q.choices.filter(choice=>contains(point,polygonFor(choice,displayData(d))));
  if(hits.length!==1){error(hits.length?'框重疊，請使用編號按鈕唯一選定。':'請點選物件框內，或使用編號按鈕。');return;}
  if(!hits[0].eligible){error('該物件資料未通過深度或工作範圍檢查，請重新取像。');return;}chooseInstance(hits[0].instance_id,q);
});
async function action(name,payload={}){
  const capture=['retry_step','refresh','refresh_scene'].includes(name);
  if(capture){syncButtons.activity='capture';syncButtons();}
  try{const id=runId,next=await api('/api/v2/runs/'+id+'/'+name,payload);if(id===runId){if(capture)syncButtons.activity=null;render(next);}}
  finally{if(capture){syncButtons.activity=null;syncButtons();}}
}
function chooseInstance(instanceId,q){guarded(()=>action('select',{run_id:q.run_id,step_id:q.step_id,frame_id:q.frame_id,plan_version:q.plan_version,role:q.role,instance_id:instanceId}));}
function answer(value,q){
  if(!q)return;
  if(q.slot==='displacement_mm'){
    const input=$('answer-value'),number=typeof value==='number'?value:Number(String(value).trim());
    input.setCustomValidity('');
    if(String(value).trim()===''||!Number.isFinite(number)||number<=0){input.setCustomValidity('請輸入大於 0 的有限毫米數。');input.reportValidity();input.focus();return;}
    value=number;
  }
  guarded(()=>action('answer',{value,version:q.version,task_index:q.task_index,slot:q.slot}));
}
function hardwareUnavailable(next=state){
  if(next?.mode!=='real'||next.operation==='preview')return false;
  const d=next.devices;
  return d?.camera?.detected!==true||d?.camera?.count===0||d?.robot?.detected!==true;
}
function autoConfirmationKey(next,consentRun,already){
  if(!next||next.run_id!==consentRun||next.status!=='waiting_confirmation'||next.worker_busy||next.read_only||next.sequence||!next.pending?.plan||next.pending?.question||next.pending?.instance_constraints?.length||hardwareUnavailable(next))return null;
  const key=next.run_id+':'+next.pending.version;return already.has(key)?null:key;
}
function maybeAutoConfirm(){
  if(busy)return;
  const key=autoConfirmationKey(state,directRunId,autoClaims);
  if(!key)return;
  autoClaims.add(key);guarded(()=>action('confirm',{version:state.pending.version}));
}
async function poll(){
  if(runId&&!busy&&!homeRunId&&!tableCapture)try{const expected=generation,id=runId,next=await api('/api/v2/runs/'+id);if(expected===generation&&id===runId&&!busy&&!homeRunId&&!tableCapture)render(next);}catch(e){error(e.message);}
  const active=['running','executing','capturing'].includes(state?.status)||Boolean(state?.worker_busy);
  clearTimeout(timer);timer=setTimeout(poll,active?250:700);
}
function sceneStartConsent(mode){
  const b=bootState,d=state?.mode===mode?state.devices||b?.devices:b?.devices;
  if(mode!=='real'||!b?.motion_enabled||d?.robot?.detected!==true||d?.camera?.detected!==true||d?.camera?.count===0)return {};
  const confirmed=state?.mode===mode&&typeof state.empty_hand_confirmed==='boolean'?state.empty_hand_confirmed:b?.empty_hand_confirmed;
  if(confirmed)return {move_to_start:true};
  if(!window.confirm('取像前需要回 Start。請在現場確認夾爪空手、機械臂已停止且路徑淨空；確認後將張開夾爪並回 Start，是否繼續？'))return null;
  return {move_to_start:true,source:'operator_empty_hand_check'};
}
async function refreshPreview(){
  cancelRoi();
  if(state?.status==='waiting_step_recovery'){await action('retry_step',recoveryPayload(state.sequence.question));return;}
  if(state?.status==='waiting_instance'){await action('refresh');return;}
  if(state?.status==='waiting_answer'){
    const consent=sceneStartConsent(state.mode);if(consent===null)return;
    await action('refresh_scene',{version:state.pending.version,...consent});return;
  }
  if(activeTask())await action('cancel');
  const consent=sceneStartConsent($('mode').value);if(consent===null)return;
  syncButtons.activity='capture';syncButtons();
  const next=await api('/api/v2/preview',{mode:$('mode').value,...consent});runId=next.run_id;directRunId=null;syncButtons.activity=null;render(next);
}
$('instruction').oninput=()=>error('');
$('instruction-form').onsubmit=e=>{
  e.preventDefault();guarded(async()=>{
    const payload={text:$('instruction').value,mode:$('mode').value,semantic_source:$('semantic-source').value,destination_mode:$('destination-mode').value};
    if(payload.destination_mode==='fixed'&&!defaultReady)throw new Error('請先將有效站點設為預設放置點。');
    const consent=sceneStartConsent(payload.mode);if(consent===null)return;Object.assign(payload,consent);
    stopRequested=false;runId=null;state=null;cancelRoi();
    $('table-roi').hidden=true;$('selection-guide').hidden=true;
    const camera=bootState?.devices?.camera;
    syncButtons.activity=payload.mode==='real'&&camera?.detected===true&&camera?.count!==0?'capture-parsing':'parsing';syncButtons();
    const next=await api('/api/v2/runs',payload);runId=next.run_id;syncButtons.activity=null;
    directRunId=$('confirmation-mode').value==='direct'?runId:null;
    if(stopRequested){directRunId=null;await action('stop');}else render(next);
  });
};
$('answer-form').onsubmit=e=>{e.preventDefault();answer($('answer-value').value,state?.pending?.question);};
$('answer-value').addEventListener('input',()=> $('answer-value').setCustomValidity(''));
$('confirm').onclick=()=>guarded(()=>action('confirm',{version:state.pending.version}));
$('cancel').onclick=()=>guarded(async()=>{directRunId=null;await action('cancel');$('instruction').focus();});
$('refresh-scene').onclick=()=>guarded(refreshPreview);
$('rotation').onchange=()=>{if(tableCapture){tableCapture.rect=null;tableCapture.preview=null;roiRequest++;$('roi-save').disabled=true;$('table-roi-save').disabled=true;}viewRotation=Number($('rotation').value);savePreferences();if(lastDisplay&&sceneImage)paint(lastDisplay,state?.sequence?.question);};
$('settings-form').onsubmit=e=>{e.preventDefault();guarded(async()=>{const overrides={};for(const input of $('settings-fields').querySelectorAll('input'))overrides[input.name]=Number(input.value);await api('/api/v2/settings',{overrides,expected_revision:settingsRevision});await bootstrap();toast('到位設定已儲存，供新計畫使用。');});};
$('refresh-readiness').onclick=()=>guarded(bootstrap);
$('set-default-station').onclick=()=>guarded(()=>stationAction('default'));
$('default-station').onchange=()=>guarded(async()=>{await api('/api/v2/places/manage',{action:'default',profile_id:$('default-station').value});await bootstrap();});
$('recover-empty').onclick=()=>guarded(async()=>{if(window.confirm('已在現場人工確認夾爪空手，且已停止機械臂？')){await api('/api/v2/recover-empty',{confirm:'EMPTY_GRIPPER'});await bootstrap();toast('已記錄現場空手確認。');}});
for(const b of document.querySelectorAll('[data-outcome]'))b.onclick=()=>guarded(()=>action('outcome',{outcome:b.dataset.outcome}));
$('semantic-source').onchange=()=>{if($('semantic-source').value==='demo'){$('instruction').value='把橡皮擦放到筆的右邊';$('mode').value='offline';}savePreferences();sourceNote();};
$('mode').onchange=()=>{savePreferences();sourceNote();};
$('destination-mode').onchange=savePreferences;
$('confirmation-mode').onclick=()=>{const direct=$('confirmation-mode').value!=='direct';$('confirmation-mode').value=direct?'direct':'once';$('confirmation-mode').setAttribute('aria-checked',String(direct));savePreferences();syncButtons();};
$('emergency-stop').onclick=async()=>{
  stopRequested=true;directRunId=null;generation++;
  try{if(homeRunId)await api('/api/v2/runs/'+homeRunId+'/stop',{});else if(runId)await action('stop');else toast('已取消；目前沒有連線中的任務。');}catch(e){error(e.message);}
};
$('return-home').onclick=()=>guarded(async()=>{
  if(!window.confirm('回 Start 會先張開夾爪。請確認夾爪空手、路徑淨空，是否繼續？'))return;
  syncButtons.activity='home';syncButtons();
  const result=await api('/api/v2/robot/home',{confirm:'RETURN_HOME'});homeRunId=result.run_id;$('status').textContent='Start 復位中';$('status').dataset.state='running';spotlight('running');toast('已送出 Start 復位；可使用緊急停止。');
  const watch=async()=>{try{const next=await api('/api/v2/runs/'+homeRunId),status=next.run?.status||next.status;if(['completed','failed','blocked','aborted','cancelled','error'].includes(status)){homeRunId=null;syncButtons.activity=null;$('status').textContent=status==='completed'?'Start 復位完成':'Start 復位已停止';$('status').dataset.state=status==='completed'?'idle':'failed';spotlight('idle');syncButtons();toast(status==='completed'?'Start 已復位，請核對現場。':'復位流程已停止，請檢查工程紀錄。');}else setTimeout(watch,250);}catch(e){error(e.message);homeRunId=null;syncButtons.activity=null;$('status').textContent='Start 復位狀態讀取失敗';$('status').dataset.state='failed';syncButtons();}};setTimeout(watch,250);
});
const examples={single:'把橡皮擦放到筆的右邊',purpose:'把可以鎖螺絲的工具放到咖啡盒子',top:'把橡皮擦放在電池上面',near:'找一個可以寫字的工具，放到電池附近',multi:'先把橡皮擦放到筆的右邊，再把電池放到咖啡盒子，最後把螺絲起子放在電池附近'};
for(const b of document.querySelectorAll('[data-example]'))b.onclick=()=>{if(activeTask()||busy)return;$('instruction').value=examples[b.dataset.example];if(b.dataset.example!=='single'){$('semantic-source').value='api';sourceNote();}$('instruction').focus();};
function voiceErrorMessage(value){
  const code=[value?.code,value?.name,value?.message,typeof value==='string'?value:''].filter(Boolean).join(' ');
  if(/NotAllowedError|not-allowed|SecurityError|voice_permission_denied/.test(code))return '麥克風權限遭拒絕；請確認系統已允許存取麥克風。';
  if(/voice_host_microphone_unavailable|voice_audio_capture_failed/.test(code))return '無法收音；請確認主機已連接麥克風、允許錄音，並安裝 sounddevice。';
  if(/voice_network_error|network|timeout|Failed to fetch/i.test(code))return '連線失敗或逾時，請檢查網路後重新錄音。';
  if(/voice_empty/.test(code))return '未錄到有效語音，請靠近主機麥克風後重試。';
  if(/voice_.*(large|sized|long)/.test(code))return '錄音或文字超出限制，請縮短指令後重新錄音。';
  if(/voice_busy|voice_operation_active|NotReadableError/.test(code))return '麥克風或語音服務正忙碌，請稍後重試。';
  if(/voice_dependency_missing/.test(code))return '語音功能缺少必要套件，請檢查主機安裝。';
  return '語音輸入失敗，請重新錄音或直接輸入文字。';
}
function voiceVisual(session,phase){
  if(voiceSession!==session)return;
  session.phase=phase;const button=$('voice-input'),recording=phase==='recording';
  button.innerHTML=recording
    ?'<svg><use href="#i-mic"/></svg> 麥克風收音中…點擊停止'
    :phase==='starting'?'<svg><use href="#i-mic"/></svg> 正在啟動麥克風…':'<svg><use href="#i-mic"/></svg> 語音轉錄中…';
  button.classList.toggle('is-listening',recording);
  button.disabled=false;
  button.setAttribute('aria-pressed',String(recording));
  button.setAttribute('aria-busy',phase==='transcribing'?'true':'false');
  $('instruction').setAttribute('aria-busy','true');
  syncButtons();
}
function finishVoice(session,message){
  if(voiceSession!==session)return;
  const button=$('voice-input');
  button.innerHTML=session.original||'<svg><use href="#i-mic"/></svg>語音輸入';
  button.classList.toggle('is-listening',false);
  button.disabled=false;
  button.setAttribute('aria-pressed','false');
  button.setAttribute('aria-busy','false');
  $('instruction').readOnly=Boolean(session.wasReadOnly);
  $('instruction').setAttribute('aria-busy','false');
  voiceSession=null;
  syncButtons();
  if(message)error('語音：'+message);
}
async function followVoice(session,job){
  session.job=job;const deadline=Date.now()+90000;
  while(voiceSession===session){
    session.job=job;
    // 及時串流逐字稿直接顯示於文字輸入框
    if(typeof job.text==='string'&&job.text){
      $('instruction').value=job.text;
    }
    if(job.status==='completed'){
      error('');session.ended=true;
      $('instruction').value=job.text;
      $('semantic-source').value='api';
      savePreferences();sourceNote();
      finishVoice(session);
      toast('語音輸入完成，請核對文字後點擊 AI 解析任務。');
      return;
    }
    if(job.status==='failed'){
      session.ended=true;
      throw Object.assign(new Error(job.error?.message||'voice_transcription_failed'),{code:job.error?.code});
    }
    if(Date.now()>deadline)throw new Error('voice_transcription_timeout');
    voiceVisual(session,job.status==='recording'?'recording':'transcribing');
    await new Promise(resolve=>setTimeout(resolve,150));
    job=await api('/api/v2/voice/'+job.voice_id);
  }
}
async function stopVoiceInput(session){
  if(session.stopping||session.ended||voiceSession!==session)return;
  session.stopping=true;
  voiceVisual(session,'transcribing');
  try{
    if(!session.ended&&session.job){
      await api('/api/v2/voice',{action:'stop',voice_id:session.job.voice_id});
    }
  }catch(e){
    await failVoice(session,e);
  }
}
async function failVoice(session,cause){
  if(session.job&&!session.ended){
    session.ended=true;
    try{await api('/api/v2/voice',{action:'stop',voice_id:session.job.voice_id});}catch{}
  }
  finishVoice(session,voiceErrorMessage(cause));
}
async function startVoiceInput(){
  if(voiceSession){
    if(voiceSession.phase==='recording')await stopVoiceInput(voiceSession);
    return;
  }
  if(activeTask()||busy)return;
  error('');
  const session={
    original:$('voice-input').innerHTML,
    phase:'starting',
    wasReadOnly:$('instruction').readOnly
  };
  voiceSession=session;
  $('instruction').readOnly=false;
  voiceVisual(session,'starting');
  try{
    const job=await api('/api/v2/voice',{source:'microphone'});
    session.job=job;
    voiceVisual(session,'recording');
    await followVoice(session,job);
  }catch(e){
    await failVoice(session,e);
  }
}
$('voice-input').title='點擊使用麥克風語音輸入，及時串流逐字稿將顯示於文字框；再次點擊可停止';
$('voice-input').onclick=()=>void startVoiceInput();
function toast(text){$('toast').textContent=text;$('toast').hidden=false;setTimeout(()=>$('toast').hidden=true,4500);}
function openDrawer(){const d=$('engineering-drawer');d.hidden=false;$('engineering-open').setAttribute('aria-expanded','true');$('engineering-close').focus();}
function closeDrawer(){$('engineering-drawer').hidden=true;$('engineering-open').setAttribute('aria-expanded','false');$('engineering-open').focus();}
$('engineering-open').onclick=()=>{openDrawer();guarded(loadEngineering);};
$('engineering-close').onclick=closeDrawer;
function selectEngineeringTab(name,focus=false){
  for(const button of document.querySelectorAll('.drawer-tabs [role=tab]')){
    const selected=button.id==='tab-'+name;button.setAttribute('aria-selected',String(selected));button.tabIndex=selected?0:-1;
    $(button.getAttribute('aria-controls')).hidden=!selected;if(selected&&focus)button.focus();
  }
}
for(const tab of document.querySelectorAll('.drawer-tabs [role=tab]')){
  tab.onclick=()=>selectEngineeringTab(tab.id.slice(4));
  tab.onkeydown=e=>{const tabs=[...document.querySelectorAll('.drawer-tabs [role=tab]')],i=tabs.indexOf(tab);let next;
    if(e.key==='ArrowRight')next=(i+1)%tabs.length;else if(e.key==='ArrowLeft')next=(i+tabs.length-1)%tabs.length;else if(e.key==='Home')next=0;else if(e.key==='End')next=tabs.length-1;else return;
    e.preventDefault();selectEngineeringTab(tabs[next].id.slice(4),true);
  };
}
document.addEventListener('keydown',e=>{
  if(e.key==='Escape'&&tableCapture){e.preventDefault();cancelRoi();$('roi-start').focus();}
  else if(e.key==='Escape'&&!$('engineering-drawer').hidden){e.preventDefault();closeDrawer();}
});
const stationStateLabel={active:'已啟用',needs_revalidation:'待重驗',candidate:'候選',disabled:'已停用',retired:'已取代'};
async function loadEngineering(){
  const data=await api('/api/v2/engineering');engineeringState=data;engineeringRevision=data.revision;
  $('engineering-fields').replaceChildren();$('engineering-extra').replaceChildren();
  for(const [key,value] of Object.entries(data.values)){
    const def=data.fields[key],values=Array.isArray(value)?value:[value],container=$(key.startsWith('robot.')?'engineering-extra':'engineering-fields');
    const makeLabel=()=>{const label=document.createElement('label');label.textContent=def.label;return label;};
    if(def.choices){const label=makeLabel(),select=document.createElement('select');select.name=key;select.setAttribute('form','engineering-form');
      for(const choice of def.choices){const option=document.createElement('option');option.value=choice;option.textContent=({segmented_mode0:'分段關節插補（mode 0）',linear_mode1:'單段直線插補（mode 1）',single_mode0:'單段關節插補（mode 0）',align_reference:'平行參照物長軸',keep_target:'保留物品自身角度',fixed_deg:'固定 RZ 角度',match_initial:'符合原位快照角度',profile_pose:'站點示教角度'})[choice]||choice;select.append(option);}select.value=value;label.append(select);container.append(label);continue;}
    values.forEach((v,i)=>{const label=makeLabel();if(Array.isArray(value))label.textContent='TCP '+(i?'Y':'X')+' 補償（mm）';const input=document.createElement('input');input.type='number';input.required=true;input.name=key;input.dataset.index=String(i);input.value=v;input.min=def.min;input.max=def.max;input.step=def.integer?'1':'any';input.setAttribute('form','engineering-form');label.append(input);container.append(label);});
  }
  const selected=$('station-select').value,newOption=document.createElement('option');newOption.value='';newOption.textContent='＋ 新增具名站點';
  const stations=data.catalog.stations,profiles=stations.map(station=>data.profiles.find(p=>p.profile_id===station.profile_id)).filter(Boolean);
  $('station-select').replaceChildren(newOption,...profiles.map(p=>{const option=document.createElement('option');option.value=p.profile_id;option.textContent=p.display_name+' · '+stationStateLabel[p.effective_status];return option;}));
  $('station-select').value=profiles.some(p=>p.profile_id===selected)?selected:'';
  $('station-list').replaceChildren();
  for(const station of stations){const p=profiles.find(p=>p.profile_id===station.profile_id),row=document.createElement('div');row.className='station-row';row.setAttribute('role','listitem');
    const button=document.createElement('button');button.type='button';button.textContent=station.name+(station.profile_id===data.catalog.default_profile_id?'（預設）':'');button.onclick=()=>{$('station-select').value=station.profile_id;fillStation();};
    const type=document.createElement('span');type.className='station-type';type.textContent=['robot_pose_read','robot_teaching'].includes(p?.placement_source?.type)?'示教站點':p?.placement_source?.type==='visual_manual_selection'?'ROI 框選':'手動站點';
    const badge=document.createElement('span');badge.className='station-badge';badge.dataset.ready=String(station.ready);badge.textContent=stationStateLabel[station.status]||station.status;row.append(button,type,badge);$('station-list').append(row);
  }
  if(!stations.length)$('station-list').textContent='尚無站點。填入名稱，選擇下方示教或 ROI 框選建立。';
  fillStation();$('table-z').value=data.table.z_mm??'';$('table-measurement').textContent=data.table.ready?'桌面 Z '+Number(data.table.z_mm).toFixed(2)+' mm':'尚未完成有效桌面標定';
  $('engineering-events').textContent=JSON.stringify(data.events,null,2);$('asset-diagnostics').textContent=JSON.stringify({fingerprints:data.fingerprints,catalog_revision:data.catalog.revision,conflicts:data.catalog.conflicts},null,2);
  $('ik-diagnostics').textContent=data.ik_events.length?JSON.stringify(data.ik_events,null,2):'尚無 IK／轉移紀錄；執行任務後可在此檢視。';
  $('teaching-status').textContent=data.teaching_active?'手動示教中，請托住手臂；離開後再儲存。':'手動示教未啟用';
  if(data.last_pose)$('pose-values').textContent='TCP：'+data.last_pose.coords_mm_deg.join(', ')+'\n六軸：'+data.last_pose.angles_deg.join(', ');
}
function fillStation(){
  const p=engineeringState?.profiles.find(p=>p.profile_id===$('station-select').value);
  $('station-name').value=p?.display_name||'';$('station-aliases').value=p?.aliases?.join(', ')||'';
  $('station-status').textContent=p?'站點狀態：'+({active:'已啟用',needs_revalidation:'待重驗',candidate:'候選',disabled:'已停用',retired:'已取代'})[p.effective_status]:'';
}
$('station-select').onchange=fillStation;
$('engineering-form').onsubmit=e=>{e.preventDefault();guarded(async()=>{
  const values={};
  for(const input of [...$('engineering-form').elements].filter(n=>n.name)){if(input.name==='grasp.tcp_xy_offset_mm'){values[input.name]??=[];values[input.name][Number(input.dataset.index)]=Number(input.value);}else values[input.name]=input.tagName==='SELECT'?input.value:Number(input.value);}
  directRunId=null;
  const result=await api('/api/v2/engineering/settings',{values,expected_revision:engineeringRevision});
  await bootstrap();await loadEngineering();if(runId)render(await api('/api/v2/runs/'+runId));$('engineering-message').textContent='已儲存參數，並同步 '+result.synced_profile_ids.length+' 個原有效站點；新計畫將使用新參數。';toast('參數與站點指紋已同步。');
});};
$('refresh-engineering').onclick=()=>guarded(loadEngineering);
for(const [id,actionName] of [['gripper-open','open'],['gripper-close','close']])$(id).onclick=()=>guarded(async()=>{if(!window.confirm('即將'+(actionName==='open'?'張開':'閉合')+'實體夾爪，請確認操作區淨空。'))return;await api('/api/v2/engineering/gripper',{action:actionName,confirm:actionName.toUpperCase()+'_GRIPPER'});await bootstrap();toast('夾爪命令已送出；空手狀態需現場確認。');});
$('read-pose').onclick=()=>guarded(async()=>{const p=await api('/api/v2/robot/pose',{});$('pose-values').textContent='TCP：'+p.coords_mm_deg.join(', ')+'\n六軸：'+p.angles_deg.join(', ');});
async function stationAction(actionName){
  const payload={action:actionName,profile_id:$('station-select').value,name:$('station-name').value,aliases:$('station-aliases').value.split(/[,，]/).map(s=>s.trim()).filter(Boolean)};
  if(['activate','revalidate','teach'].includes(actionName)){payload.confirm='ACTIVATE_PLACE_PROFILE';payload.clear_path_acknowledged=$('clear-path').checked;if(!payload.clear_path_acknowledged)throw new Error('請先勾選已確認站點與移動路徑淨空。');}
  if(['disable','delete'].includes(actionName)){if(!window.confirm(actionName==='delete'?'確定刪除此站點？此操作會移除該版本。':'確定停用此站點？'))return;payload.confirm=actionName==='delete'?'DELETE_PLACE_PROFILE':'DISABLE_PLACE_PROFILE';}
  if(actionName!=='teach'&&!payload.profile_id)throw new Error('請先選擇要管理的站點。');
  const result=await api('/api/v2/places/manage',payload);await bootstrap();await loadEngineering();
  const created=result.profile?.profile_id;if(created){$('station-select').value=created;fillStation();}
  toast(actionName==='teach'?'示教站點已儲存並啟用。':'站點資料已更新。');
}
for(const [id,name] of [['teach-station','teach'],['station-metadata','metadata'],['station-activate','activate'],['station-disable','disable'],['station-delete','delete']])$(id).onclick=()=>guarded(()=>stationAction(name));
for(const action of ['enter','exit'])$('teaching-'+action).onclick=()=>guarded(async()=>{
  if(!window.confirm(action==='enter'?'已確認空手且托住機械臂？進入後將釋放關節扭力。':'已托住機械臂並確認可恢復扭力？'))return;
  const result=await api('/api/v2/robot/teaching',{action,confirm:action==='enter'?'ENTER_TEACH_MODE':'EXIT_TEACH_MODE'});$('teaching-status').textContent=result.active?'手動示教中，請托住手臂。':'已恢復扭力，可儲存剛才讀取的位姿。';await bootstrap();
});
$('table-form').onsubmit=e=>{e.preventDefault();guarded(async()=>{await api('/api/v2/table-surface/calibrate',{z_mm:Number($('table-z').value),confirmed:$('table-confirmed').checked,note:'現場手動量測'});await bootstrap();await loadEngineering();toast('桌面基準與站點指紋已更新。');});};
$('table-auto').onclick=()=>guarded(async()=>{if(!$('table-confirmed').checked)throw new Error('請先確認畫面中央量測區域為空的水平桌面。');await api('/api/v2/table-surface/calibrate',{confirmed:true,note:'中央 RGB-D 區域量測'});await bootstrap();await loadEngineering();toast('桌面基準與站點指紋已更新。');});
$('table-capture').onclick=()=>guarded(()=>beginRoi('table'));
$('roi-start').onclick=()=>guarded(()=>beginRoi('station'));
$('roi-cancel').onclick=()=>cancelRoi();
$('roi-apply').onclick=()=>guarded(async()=>{if(!tableCapture)throw new Error('請先擷取框選用影格。');const rect=['x1','y1','x2','y2'].map(k=>{const value=$('roi-'+k).value;return value===''?NaN:Number(value);});tableCapture.rect=rect;paint(lastDisplay,null);await updateRoiPreview();});
$('roi-save').onclick=()=>guarded(()=>saveRoi('station'));
$('table-roi-save').onclick=()=>guarded(()=>saveRoi('table'));
$('refresh-diagnostics').onclick=()=>guarded(loadEngineering);
$('check-devices').onclick=()=>guarded(async()=>{
  await bootstrap();const d=bootState.devices;$('device-detail').textContent='手臂 '+(d.robot.configured_port||'未設定')+'；相機 '+d.camera.count+' 台。偵測到裝置不代表動作已驗證。';
});
$('error-action').onclick=()=>{
  const name=$('error-action').dataset.action;
  if(name==='devices')$('check-devices').onclick();
  else if(name==='recover')$('recover-empty').onclick();
  else if(name==='capture')guarded(refreshPreview);
  else if(name==='pose')$('read-pose').onclick();
  else if(name==='reset')guarded(async()=>{if(activeTask())await action('cancel');directRunId=null;$('instruction').focus();});
  else if(name==='status')$('status').focus();
  else {openDrawer();guarded(loadEngineering);}
};
function paintRoi(context,rect){
  const [a,b,c,d]=rect;if(!rect.every(Number.isFinite))return;
  const scale=Math.max(1,context.canvas.width/(context.canvas.getBoundingClientRect().width||context.canvas.width));
  context.save();context.fillStyle='#2563EB22';context.strokeStyle='#1D4ED8';context.lineWidth=2*scale;context.setLineDash([6*scale,4*scale]);
  context.fillRect(a,b,c-a,d-b);context.strokeRect(a,b,c-a,d-b);context.restore();
}
function cancelRoi(){
  tableCapture=null;roiPointer=null;roiRequest++;document.body.classList.remove('roi-selecting');
  $('roi-toolbar').hidden=true;$('table-roi').hidden=true;$('roi-save').disabled=true;$('table-roi-save').disabled=true;
  $('selection-guide').hidden=true;if(lastDisplay&&sceneImage?.complete)paint(lastDisplay,state?.sequence?.question);
}
async function beginRoi(kind){
  if(kind==='station'&&!$('station-name').value.trim())throw new Error('請先輸入站點名稱。');
  if(activeTask())throw new Error('請先完成或取消目前任務，再擷取標定影格。');
  cancelRoi();const result=await api('/api/v2/places/capture',{source:kind==='table'?'new':$('roi-source').value});
  tableCapture={...result,kind,rect:null,preview:null};lastDisplay={frame_id:result.capture_id,image_url:result.frame_url,width:result.width,height:result.height,rotation_deg:viewRotation??0,instances:[],synthetic:false};
  document.body.classList.add('roi-selecting');$('roi-toolbar').hidden=false;$('roi-guide').textContent=kind==='table'?'拖曳框選空的水平桌面；Esc 取消':'拖曳框選固定落點；Esc 取消';
  $('roi-preview').textContent='請在左側畫面拖曳拉框，或使用座標輸入。';$('table-roi').hidden=kind!=='table';$('frame-source').textContent=kind==='table'?'桌面 RGB-D 標定':'固定站點 RGB-D 框選';
  selectEngineeringTab('stations');drawScene(lastDisplay,null);renderCoordinates(lastDisplay,{});$('scene').focus();
}
function roiPayload(){
  if(!tableCapture?.rect)throw new Error('請先在畫面拖曳框選區域。');
  return {capture_id:tableCapture.capture_id,roi_xyxy:[...tableCapture.rect],rotation_deg:displayData(lastDisplay).rotation_deg,rz_deg:Number($('roi-rz').value)};
}
async function updateRoiPreview(){
  const selection=tableCapture,request=++roiRequest;selection.preview=null;$('roi-save').disabled=true;$('table-roi-save').disabled=true;
  $('roi-preview').textContent='計算深度落點中…';
  try{
    const result=await api('/api/v2/places/roi_capture',{...roiPayload(),save:false});
    if(tableCapture!==selection||request!==roiRequest)return;
    selection.preview=result;const [x,y,z]=result.robot_surface_xyz_mm;
    $('roi-preview').textContent='X '+x.toFixed(2)+' · Y '+y.toFixed(2)+' · Z '+z.toFixed(2)+' mm；有效深度 '+(result.depth_valid_ratio*100).toFixed(1)+'%'+(result.workspace_ok?'':'；超出機械臂工作範圍');
    $('roi-save').disabled=selection.kind!=='station'||!result.workspace_ok;$('table-roi-save').disabled=selection.kind!=='table';
  }catch(e){if(tableCapture===selection&&request===roiRequest){$('roi-preview').textContent='框選量測未完成：'+e.message;error(e.message);}}
}
async function saveRoi(kind){
  if(!tableCapture?.preview||tableCapture.kind!==kind)throw new Error('請先完成有效深度預覽。');
  const payload=roiPayload();
  if(kind==='station'){
    if(!$('clear-path').checked)throw new Error('請先確認站點與移動路徑淨空。');
    await api('/api/v2/places/roi_capture',{...payload,save:true,name:$('station-name').value,aliases:$('station-aliases').value.split(/[,，]/).map(s=>s.trim()).filter(Boolean),profile_id:$('station-select').value||null,clear_path_acknowledged:true});
  }else{
    if(!$('table-confirmed').checked)throw new Error('請先確認選取區域為空的水平桌面。');
    delete payload.rz_deg;await api('/api/v2/table-surface/calibrate',{...payload,confirmed:true,note:'畫布 ROI 框選桌面'});
  }
  cancelRoi();await bootstrap();await loadEngineering();toast(kind==='station'?'ROI 站點已儲存並啟用。':'桌面基準與站點指紋已更新。');
}
function canvasPointer(event){
  const canvas=$('scene'),box=canvas.getBoundingClientRect();
  return [Math.round(Math.max(0,Math.min(canvas.width,(event.clientX-box.left)*canvas.width/box.width))),Math.round(Math.max(0,Math.min(canvas.height,(event.clientY-box.top)*canvas.height/box.height)))];
}
$('scene').addEventListener('pointerdown',event=>{
  if(!tableCapture||busy||event.button!==0||!sceneImage?.complete)return;
  event.preventDefault();roiRequest++;roiPointer={id:event.pointerId,start:canvasPointer(event)};tableCapture.preview=null;tableCapture.rect=[...roiPointer.start,...roiPointer.start];
  $('roi-save').disabled=true;$('table-roi-save').disabled=true;event.currentTarget.setPointerCapture(event.pointerId);paint(lastDisplay,null);
});
$('scene').addEventListener('pointermove',event=>{
  if(!tableCapture||!roiPointer||event.pointerId!==roiPointer.id)return;
  tableCapture.rect=[...roiPointer.start,...canvasPointer(event)];paint(lastDisplay,null);
});
$('scene').addEventListener('pointerup',event=>{
  if(!tableCapture||!roiPointer||event.pointerId!==roiPointer.id)return;
  tableCapture.rect=[...roiPointer.start,...canvasPointer(event)];roiPointer=null;event.currentTarget.releasePointerCapture(event.pointerId);
  ['x1','y1','x2','y2'].forEach((key,i)=>{$('roi-'+key).value=Math.round(tableCapture.rect[i]);});paint(lastDisplay,null);updateRoiPreview();
});
$('scene').addEventListener('pointercancel',()=>{if(tableCapture){roiPointer=null;tableCapture.rect=null;tableCapture.preview=null;roiRequest++;$('roi-save').disabled=true;$('table-roi-save').disabled=true;paint(lastDisplay,null);}});
globalThis.matchMedia?.('(prefers-reduced-motion: reduce)').addEventListener('change',()=>syncButtons());
bootstrap().then(()=>{spotlight(state?.status||'idle');poll();}).catch(e=>error(e.message));

for(const id of ['show-placement-preview','show-placement-envelope'])$(id).onchange=()=>{if(lastDisplay&&sceneImage?.complete)paint(lastDisplay,state?.sequence?.question);};
