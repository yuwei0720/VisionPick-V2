'use strict';
const {chromium}=require('playwright'),assert=require('node:assert/strict'),fs=require('node:fs');
const input=JSON.parse(process.argv[2]);
(async()=>{
  const browser=await chromium.launch({headless:true,channel:'msedge'});
  try{
    const page=await browser.newPage({viewport:{width:1440,height:900}}),errors=[],posts=[],polls=[];
    page.on('pageerror',e=>errors.push(e.message));
    page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});
    page.on('request',r=>{if(r.method()==='POST')posts.push({url:r.url(),body:r.postDataJSON()});if(r.method()==='GET'&&/\/api\/v2\/runs\/[a-f0-9]+$/.test(r.url()))polls.push(r.url());});
    await page.route('**/*',route=>new URL(route.request().url()).origin===new URL(input.url).origin?route.continue():route.abort());
    if(input.scenario.startsWith('voice'))await page.addInitScript(scenario=>{
      window.browserMicrophoneRequests=0;
      Object.defineProperty(navigator,'mediaDevices',{configurable:true,value:{getUserMedia:async()=>{
        window.browserMicrophoneRequests++;throw Error('Browser microphone must never be requested');
      }}});
      if(scenario==='voice-insecure')Object.defineProperty(window,'isSecureContext',{value:false});
    },input.scenario);
    await page.goto(input.url+'/v2/');await page.waitForLoadState('networkidle');
    await page.waitForFunction(()=>typeof bootState==='object');
    if(input.scenario==='preferences'){
      await page.click('#engineering-open');await page.click('#tab-motion');await page.locator('#environment-details summary').click();
      await page.selectOption('#semantic-source','api');await page.selectOption('#mode','real');await page.click('#engineering-close');
      await page.selectOption('#destination-mode','strict');await page.click('#confirmation-mode');await page.selectOption('#rotation','270');
      await page.reload();await page.waitForLoadState('networkidle');
      for(const [id,value] of Object.entries({rotation:'270','confirmation-mode':'direct','destination-mode':'strict',mode:'real','semantic-source':'api'}))assert.equal(await page.locator('#'+id).evaluate(e=>e.value),value);
      assert.equal(await page.locator('#confirmation-mode').getAttribute('aria-checked'),'true');
      await page.click('#confirmation-mode');await page.click('#engineering-open');await page.click('#tab-motion');await page.locator('#environment-details summary').click();await page.selectOption('#mode','offline');await page.click('#engineering-close');
      await page.selectOption('#rotation','180');await page.fill('#instruction','把橡皮擦往右移三十毫米');await page.click('#parse');
      await page.waitForFunction(()=>state?.status==='waiting_confirmation'&&!document.querySelector('#confirm').disabled);
      const until=Date.now()+5000;while(polls.length<3&&Date.now()<until)await page.waitForTimeout(100);
      assert(polls.length>=3);assert.equal(await page.locator('#rotation').inputValue(),'180');
      assert.equal(await page.evaluate(()=>viewRotation),180);
      assert.match(await page.locator('#decision-note').textContent(),/工作台 90°，目前顯示 180°/);
      assert(posts.every(p=>!/settings|engineering/.test(p.url)));
      for(const width of [1280,1440,1920]){
        await page.setViewportSize({width,height:900});
        const size=await page.evaluate(()=>({scroll:document.documentElement.scrollHeight,client:document.documentElement.clientHeight}));
        assert.equal(size.scroll,size.client);
      }
      await page.screenshot({animations:'disabled',path:input.output});
    }else if(input.scenario.startsWith('voice')){
      page.on('dialog',d=>d.accept());
      const before=await page.locator('#instruction').inputValue();
      await page.click('#voice-input');
      await page.waitForFunction(()=>document.querySelector('#voice-input').getAttribute('aria-pressed')==='true');
      assert.equal(await page.locator('#voice-input').isDisabled(),false);
      assert.equal(await page.locator('#parse').isDisabled(),true);
      await page.screenshot({animations:'disabled',path:input.output.replace('.png','-recording.png')});
      assert.equal(await page.locator('#instruction').getAttribute('aria-busy'),'true');
      assert.equal(await page.locator('#instruction').evaluate(e=>e.readOnly),false);
      if(input.scenario!=='voice-network'){
        await page.waitForFunction(()=>document.querySelector('#instruction').value==='把橡皮擦');
        assert.equal(await page.locator('#instruction').getAttribute('aria-busy'),'true');
        await page.screenshot({animations:'disabled',path:input.output.replace('.png','-interim.png')});
      }
      await page.click('#voice-input');
      if(input.scenario==='voice-network'){
        await page.waitForFunction(()=>!document.querySelector('#error').hidden&&!voiceSession);
        assert.match(await page.locator('#error-message').textContent(),/連線失敗或逾時/);
        assert.equal(await page.locator('#instruction').inputValue(),before);
      }else{
        await page.waitForFunction(()=>document.querySelector('#instruction').value==='把橡皮擦放到右盒子'&&!voiceSession);
        assert.equal(await page.locator('#semantic-source').inputValue(),'api');
      }
      assert.equal(await page.locator('#voice-input').isDisabled(),false);
      assert.equal(await page.locator('#voice-input').getAttribute('aria-pressed'),'false');
      assert(posts.every(p=>/\/api\/v2\/voice$/.test(p.url)));
      assert.equal(posts[0].body.source,'microphone');
      assert.equal(posts.length,2);assert.equal(posts[1].body.action,'stop');
      assert.equal(await page.evaluate(()=>window.browserMicrophoneRequests),0);
      assert.equal(await page.locator('#instruction').getAttribute('aria-busy'),'false');
      await page.screenshot({animations:'disabled',path:input.output});
      if(input.scenario==='voice-live'){
        await page.click('#engineering-open');await page.click('#tab-motion');
        await page.waitForSelector('select[name=\"robot.vertical_motion_mode\"]',{state:'attached'});
        assert.deepEqual(await page.locator('select[name=\"robot.vertical_motion_mode\"] option').evaluateAll(nodes=>nodes.map(n=>n.value)),['segmented_mode0','linear_mode1','single_mode0']);
        await page.locator('select[name="robot.vertical_motion_mode"]').scrollIntoViewIfNeeded();
        await page.screenshot({animations:'disabled',path:input.output.replace('.png','-modes.png')});
      }
    }else if(input.scenario==='scene-confirm'){
      let accept=false;const dialogs=[];
      page.on('dialog',async d=>{dialogs.push(d.message());await(accept?d.accept():d.dismiss());});
      await page.click('#parse');assert.equal(posts.length,0);
      accept=true;await page.click('#parse');
      await page.waitForFunction(()=>state?.status==='waiting_answer'&&!state.worker_busy);
      assert.equal(posts[0].body.source,'operator_empty_hand_check');assert.equal(posts[0].body.move_to_start,true);
      const old=await page.evaluate(()=>state.semantic_scene.frame_id);
      await page.click('#refresh-scene');
      await page.waitForFunction(old=>state?.semantic_scene?.frame_id!==old&&!state.worker_busy,old);
      assert.equal(dialogs.length,2);assert.match(dialogs[0],/現場確認夾爪空手/);
      assert(posts.some(p=>/refresh_scene$/.test(p.url)&&p.body.move_to_start===true));
      await page.screenshot({animations:'disabled',path:input.output});
    }
    assert.deepEqual(errors,[]);
    const report={scenario:input.scenario,browser:'Edge headless / Playwright',posts:posts.map(p=>({url:new URL(p.url).pathname,source:p.body.source,action:p.body.action})),polls:polls.length,console_errors:errors.length};
    fs.writeFileSync(input.report,JSON.stringify(report,null,2));console.log(JSON.stringify(report));
  }finally{await browser.close();}
})().catch(e=>{console.error(e.stack);process.exitCode=1;});
