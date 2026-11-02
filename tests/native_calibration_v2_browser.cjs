'use strict';
const {chromium}=require('playwright'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const input=JSON.parse(process.argv[2]),scale=input.scale||1,expectedRaw=[20*scale,10*scale,40*scale-1,30*scale-1];
(async()=>{
  const browser=await chromium.launch({headless:true,channel:'msedge'}),errors=[],requests=[],checks=[],screens=[];let page;
  try{
    page=await browser.newPage({viewport:{width:1440,height:900},reducedMotion:'reduce'});
    page.on('pageerror',e=>errors.push(e.message));
    page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});
    page.on('request',r=>{if(r.method()==='POST')requests.push({path:new URL(r.url()).pathname,data:r.postDataJSON()});});
    await page.route('**/*',r=>new URL(r.request().url()).origin===new URL(input.url).origin?r.continue():r.abort());
    await page.goto(input.url);await page.waitForLoadState('networkidle');
    await page.click('#engineering-open');await page.waitForFunction(()=>!!engineeringState);
    for(const [name,index] of [['stations',0],['geometry',1],['motion',2],['diagnostics',3]]){
      await page.click('#tab-'+name);
      assert.equal(await page.locator('[role=tabpanel]:visible').count(),1);
      assert.equal(await page.locator('#tab-'+name).getAttribute('aria-selected'),'true');
      const layout=await page.evaluate(()=>({scroll:document.documentElement.scrollHeight,client:document.documentElement.clientHeight}));
      assert(layout.scroll<=layout.client);checks.push({tab:name,...layout});
      const filename='tab-'+(index+1)+'-'+name+'.png';await page.screenshot({path:path.join(input.output,filename),animations:'disabled'});screens.push(filename);
    }
    await page.focus('#tab-stations');await page.keyboard.press('ArrowRight');
    assert.equal(await page.locator('#tab-geometry').getAttribute('aria-selected'),'true');
    await page.keyboard.press('End');assert.equal(await page.locator('#tab-diagnostics').getAttribute('aria-selected'),'true');
    await page.keyboard.press('Home');assert.equal(await page.locator('#tab-stations').getAttribute('aria-selected'),'true');
    checks.push('arrow/Home/End tab selection');
    await page.fill('#station-name','瀏覽器示教站');await page.fill('#station-aliases','示教盒');await page.check('#clear-path');
    await page.click('#read-pose');await page.waitForFunction(()=>!busy);
    await page.click('#teach-station');await page.waitForFunction(()=>!busy&&engineeringState?.profiles.some(p=>p.display_name==='瀏覽器示教站'));
    let station=await page.evaluate(()=>engineeringState.profiles.find(p=>p.display_name==='瀏覽器示教站'));
    assert.deepEqual(station.place_pose_mm_deg.slice(3),[-172,-7,34]);assert.equal(station.effective_status,'active');
    checks.push('native taught pose stored active with original orientation');
    for(const rotation of [0,90,180,270]){
      await page.selectOption('#station-select','');await page.fill('#station-name','ROI '+rotation);await page.fill('#station-aliases','');
      await page.locator('#roi-start').scrollIntoViewIfNeeded();await page.click('#roi-start');await page.waitForFunction(()=>!!tableCapture&&!busy&&sceneImage?.complete,{},{timeout:5000});
      await page.selectOption('#rotation',String(rotation));
      const rect=({0:[20,10,40,30],90:[10,40,30,60],180:[40,30,60,50],270:[30,20,50,40]})[rotation].map(v=>v*scale);
      const box=await page.locator('#scene').boundingBox(),size=await page.locator('#scene').evaluate(c=>[c.width,c.height]);
      const drawer=await page.locator('#engineering-drawer').boundingBox();assert(box.x+box.width<=drawer.x,'ROI drawer covers camera');
      await page.mouse.move(box.x+rect[0]*box.width/size[0],box.y+rect[1]*box.height/size[1]);
      await page.mouse.down();await page.mouse.move(box.x+rect[2]*box.width/size[0],box.y+rect[3]*box.height/size[1],{steps:8});await page.mouse.up();
      await page.waitForFunction(()=>!!tableCapture?.preview);
      const preview=await page.evaluate(()=>tableCapture.preview);
      assert.deepEqual(preview.selected_roi_px,expectedRaw);assert(Math.abs(preview.robot_surface_xyz_mm[2]-120)<.001);
      assert.match(await page.locator('#roi-preview').textContent(),/100\.0%/);
      await page.locator('#roi-preview').scrollIntoViewIfNeeded();
      const filename='tab-1-roi-'+rotation+'.png';await page.screenshot({path:path.join(input.output,filename),animations:'disabled'});screens.push(filename);
      await page.click('#roi-save');await page.waitForFunction(()=>!tableCapture&&!busy);
      assert((await page.evaluate(name=>engineeringState.profiles.find(p=>p.display_name===name).effective_status,'ROI '+rotation))==='active');
      checks.push('rotated canvas drag '+rotation+'° -> raw '+JSON.stringify(expectedRaw)+' -> active station');
    }
    await page.selectOption('#station-select','');await page.fill('#station-name','鍵盤 ROI');await page.click('#roi-start');
    await page.waitForFunction(()=>!!tableCapture&&!busy&&sceneImage?.complete,{},{timeout:5000});await page.selectOption('#rotation','0');
    await page.locator('#roi-coordinate-fields').evaluate(n=>n.closest('details').open=true);
    for(const [key,value] of Object.entries({x1:20,y1:10,x2:40,y2:30}))await page.fill('#roi-'+key,String(value*scale));
    await page.click('#roi-apply');await page.waitForFunction(()=>!!tableCapture?.preview&&!busy);
    await page.keyboard.press('Escape');assert.equal(await page.evaluate(()=>tableCapture),null);
    assert.equal(await page.locator('#roi-toolbar').isHidden(),true);checks.push('numeric ROI alternative and Escape cancellation');
    await page.check('#table-confirmed');await page.click('#table-capture');await page.waitForFunction(()=>!!tableCapture&&!busy&&sceneImage?.complete,{},{timeout:5000});
    await page.selectOption('#rotation','0');await page.locator('#roi-coordinate-fields').evaluate(n=>n.closest('details').open=true);
    for(const [key,value] of Object.entries({x1:20,y1:10,x2:40,y2:30}))await page.fill('#roi-'+key,String(value*scale));
    await page.click('#roi-apply');await page.waitForFunction(()=>!!tableCapture?.preview&&!busy);await page.click('#table-roi-save');
    await page.waitForFunction(()=>!busy&&!tableCapture&&bootState?.table.ready);assert(Math.abs(await page.evaluate(()=>bootState.table.z_mm)-120)<.001);
    checks.push('table ROI measurement and fingerprint synchronization');
    for(const width of [1440,1050,1920]){
      await page.setViewportSize({width,height:900});const layout=await page.evaluate(()=>({scroll:document.documentElement.scrollHeight,client:document.documentElement.clientHeight}));assert(layout.scroll<=layout.client);checks.push({width,...layout});
    }
    assert(requests.every(r=>r.path.startsWith('/api/v2/')));
    assert(!requests.some(r=>/\/runs$|\/robot\/home$|\/engineering\/gripper$/.test(r.path)));
    assert.deepEqual(errors,[]);
    fs.writeFileSync(path.join(input.output,'browser-report.json'),JSON.stringify({viewport:'1440x900',controlled_rgbd:true,physical_motion:false,checks,screens,errors,requests},null,2));
    console.log(JSON.stringify({checks:checks.length,screens:screens.length,errors}));
  }catch(e){if(page){await page.screenshot({path:path.join(input.output,'failure.png'),animations:'disabled'});fs.writeFileSync(path.join(input.output,'failure.json'),JSON.stringify({errors,requests,checks,message:await page.locator('#error-message').textContent(),status:await page.locator('#station-status').textContent(),debug:await page.evaluate(()=>({localErrors,stationName:document.querySelector('#station-name').value,hasToolbar:!!document.querySelector('#roi-toolbar'),state,busy,tableCapture}))},null,2));}throw e;}finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
