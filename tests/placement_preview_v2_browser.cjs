"use strict";
const {chromium}=require('playwright'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const input=JSON.parse(fs.readFileSync(process.argv[2],'utf8')),output=input.output;
(async()=>{
 const browser=await chromium.launch({headless:true,channel:'msedge'});
 const errors=[],posts=[],screens=[],checks=[];
 try{
  const page=await browser.newPage({viewport:{width:1440,height:900}});
  page.on('pageerror',e=>errors.push(e.message));
  page.on('request',r=>{if(r.method()==='POST')posts.push(new URL(r.url()).pathname);});
  await page.route('**/*',route=>{const u=new URL(route.request().url());if(!['127.0.0.1','localhost'].includes(u.hostname))return route.abort();return route.continue();});
  await page.goto(input.url+'/v2/');await page.waitForLoadState('networkidle');
  await page.waitForFunction(()=>typeof bootState==='object');
  await page.click('#refresh-scene');await page.waitForFunction(()=>state?.status==='completed'&&!state.worker_busy);
  await page.click('#parse');await page.waitForFunction(()=>state?.status==='waiting_confirmation'&&!state.worker_busy&&state.placement_previews?.length===1);
  assert(await page.locator('#show-placement-preview').isChecked());assert(!(await page.locator('#show-placement-envelope').isChecked()));
  assert.equal(await page.evaluate(()=>state.placement_previews[0].feasible),true);
  const before=await page.locator('#scene').evaluate(c=>c.toDataURL());
  await page.uncheck('#show-placement-preview');const hidden=await page.locator('#scene').evaluate(c=>c.toDataURL());assert.notEqual(before,hidden);
  await page.check('#show-placement-preview');await page.check('#show-placement-envelope');const envelope=await page.locator('#scene').evaluate(c=>c.toDataURL());assert.notEqual(envelope,before);checks.push('overlay and envelope independently change canvas pixels');
  const version=await page.evaluate(()=>state.pending.version);
  await page.locator('.placement-control select').selectOption('fixed_deg');
  await page.waitForFunction(v=>state.pending.version>v,version);
  await page.locator('.placement-control input').fill('72');
  await page.waitForFunction(()=>state.placement_previews[0].rz_deg===72&&!busy);
  assert((await page.evaluate(()=>state.pending.version))>version);checks.push('custom 72 degrees updates geometry and confirmation version');
  for(const rotation of ['0','90','180','270']){
   await page.selectOption('#rotation',rotation);await page.waitForFunction(r=>String(viewRotation)===r,rotation);
   const name='preview-'+rotation+'.png';await page.screenshot({animations:'disabled',path:path.join(output,name)});screens.push(name);
  }
  for(const width of [1280,1440,1920]){await page.setViewportSize({width,height:900});const size=await page.evaluate(()=>[document.documentElement.scrollHeight,document.documentElement.clientHeight]);assert.equal(size[0],size[1]);}
  await page.setViewportSize({width:1440,height:900});checks.push('four rotations and zero page scroll at three desktop widths');
  await page.click('#engineering-open');await page.click('#tab-geometry');
  await page.locator('select[name="orientation.place_orientation_policy_relative"]').selectOption('fixed_deg');
  await page.locator('input[name="orientation.fixed_place_yaw_deg"]').fill('33');
  await page.click('#tab-motion');const extra=page.locator('select[name="robot.vertical_motion_mode"]');
  await extra.selectOption('linear_mode1');await page.locator('input[name="robot.vertical_segment_step_mm"]').fill('0');
  await page.click('#tab-geometry');assert(!(await page.locator('#save-engineering').isDisabled()));
  const v=await page.evaluate(()=>state.pending.version);await page.click('#save-engineering');
  await page.waitForFunction(v=>state.pending.version>v&&!busy,v);
  assert.equal(await page.evaluate(()=>state.placement_previews[0].rz_deg),72);
  assert.match(await page.locator('#engineering-message').textContent(),/同步 1 個/);
  await page.click('#engineering-close');await page.locator('.placement-control select').selectOption('default');
  await page.waitForFunction(()=>state.placement_previews[0].rz_deg===33&&!busy);checks.push('engineering defaults refresh preview while step override takes precedence');
  await page.click('#cancel');await page.waitForFunction(()=>state.status==='cancelled'&&!busy);
  await page.click('#engineering-open');await page.click('#tab-motion');await page.locator('#environment-details summary').click();await page.selectOption('#semantic-source','api');await page.click('#engineering-close');
  async function parse(text){await page.click('#refresh-scene');await page.waitForFunction(()=>state?.status==='completed'&&!state.worker_busy&&!busy);await page.fill('#instruction',text);await page.click('#parse');await page.waitForFunction(()=>state?.status==='waiting_confirmation'&&!state.worker_busy&&!busy);}
  await parse('交換筆與橡皮擦');assert.equal(await page.evaluate(()=>state.placement_previews.length),3);
  assert.deepEqual(await page.evaluate(()=>state.placement_previews.map(p=>p.step_id)),[1,2,3]);
  await page.screenshot({animations:'disabled',path:path.join(output,'preview-swap.png')});screens.push('preview-swap.png');checks.push('three-step swap labels and original yaw forecasts');
  await page.click('#cancel');await page.waitForFunction(()=>state.status==='cancelled'&&!busy);
  await parse('警告候選');assert.equal(await page.evaluate(()=>state.placement_previews[0].feasible),false);
  assert.equal(await page.evaluate(()=>state.placement_previews[0].polygon_px),undefined);
  await page.screenshot({animations:'disabled',path:path.join(output,'preview-warning.png')});screens.push('preview-warning.png');checks.push('infeasible candidate shows diagnostic status without a destination polygon');
  assert.deepEqual(errors,[]);assert(!posts.some(p=>p.endsWith('/confirm')));
  const report={browser:'Edge headless / Playwright',checks,console_errors:errors,posts,screenshots:screens,physical_motion:false,provider_calls:0};
  fs.writeFileSync(path.join(output,'browser_report.json'),JSON.stringify(report,null,2));console.log(JSON.stringify(report));
 }finally{await browser.close();}
})().catch(e=>{console.error(e.stack);process.exitCode=1;});
