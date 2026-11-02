"""Ten-system repairs: controlled HTTP, scenes, feedback and voice; no hardware/API."""
import asyncio
from types import SimpleNamespace
from voice_input.gemini_live_transcriber import transcribe_pcm_live
from copy import deepcopy
import json
import time
from pathlib import Path

import pytest

from test_v2_console_integration import factory, task
from test_v2_q1_q4 import devices
from operator_console.events import atomic_json
import base64
import io
import wave
import threading


def real(h, *, known_empty=True):
    h.runtime.holding_path.parent.mkdir(parents=True,exist_ok=True)
    h.service.hardware_enabled=h.service.motion_enabled=True
    h.service.device_status=lambda:devices()
    h.cfg['calibration'].update(ready_for_motion=True,rmse_mm=1,max_rmse_mm=15)
    if known_empty:
        atomic_json(h.runtime.holding_path,{'holding_uncertain':False,'source':'controller_release_state'})


def create(h, **extra):
    code,s,_=h.http('/api/v2/runs',{'text':'受控取像指令','semantic_source':'api','mode':'real',**extra})
    assert code==201,s
    h.run_id=s['run_id']
    return h.wait('waiting_confirmation','waiting_answer','failed','blocked')


@pytest.mark.parametrize('holding',[None,True])
def test_unknown_or_holding_requires_ack_before_any_arm_capture_or_provider(factory,holding):
    h=factory();real(h,known_empty=False)
    if holding is not None:atomic_json(h.runtime.holding_path,{'holding_uncertain':holding})
    s=create(h)
    assert s['status']=='failed' and s['error']['reason']=='empty_gripper_confirmation_required'
    assert h.order==h.calls==h.backends==[]


@pytest.mark.parametrize('known_empty',[False,True])
def test_parse_returns_to_start_before_capture_for_known_or_acknowledged_empty_hand(factory,known_empty):
    h=factory();real(h,known_empty=known_empty)
    s=create(h,**({} if known_empty else {'source':'operator_empty_hand_check','move_to_start':True}))
    assert s['status']=='waiting_confirmation'
    assert h.order==[('start',1),('capture',1)] and len(h.calls)==1
    assert h.runtime._visible_scene['captured_at_start'] is True
    assert h.runtime._runs[h.run_id]['controller'] is None


@pytest.mark.parametrize('invalidate',['none','not_at_start','movement','maintenance_state','controller_state','expired','config'])
def test_real_cache_reuse_requires_verified_start_and_no_subsequent_motion(factory,invalidate):
    h=factory();real(h);create(h);h.action('cancel')
    cached=h.runtime._visible_scene
    if invalidate=='not_at_start':cached['captured_at_start']=False
    if invalidate=='movement':h.runtime._arm_motion_epoch+=1
    if invalidate=='maintenance_state':h.runtime._write_controller_state({'holding_uncertain':False,'source':'manual_gripper'})
    if invalidate=='controller_state':atomic_json(h.runtime.holding_path,{'holding_uncertain':False,'at_start':False})
    if invalidate=='expired':cached['captured_at_unix_ms']=0
    if invalidate=='config':h.cfg['scene']['workspace_view_rotation_deg']=180
    s=create(h);assert s['status']=='waiting_confirmation'
    assert len([v for v in h.order if v[0]=='capture'])==(1 if invalidate=='none' else 2)
    assert len([v for v in h.order if v[0]=='start'])==(1 if invalidate=='none' else 2)


def test_followup_refresh_returns_to_start_and_updates_scene_without_second_provider(factory):
    h=factory(tasks=[task()|{'target':None}]);real(h)
    s=create(h);assert s['status']=='waiting_answer'
    old=s['semantic_scene']['frame_id']
    atomic_json(h.runtime.holding_path,{'holding_uncertain':True})
    code,_,_=h.action('refresh_scene',{'version':s['pending']['version'],'source':'operator_empty_hand_check','move_to_start':True})
    assert code==200
    s=h.wait('waiting_answer')
    assert h.order==[('start',1),('capture',1),('start',2),('capture',1)]
    assert s['semantic_scene']['frame_id']!=old and len(h.calls)==1
    assert s['error'] is None and h.runtime._visible_scene['captured_at_start'] is True


@pytest.mark.parametrize('count',[1,2,3])
def test_only_final_step_returns_to_start_and_persists_empty_hand(factory,count):
    h=factory(tasks=[task()]*count);real(h)
    create(h);h.confirm();s=h.wait('completed','failed')
    assert s['status']=='completed',s.get('error')
    backend=h.backends[-1]
    assert backend.calls[-1]==('angles',h.cfg['robot']['start_angles_deg'])
    assert len([c for c in backend.calls if c[0]=='angles'])==count+1
    assert len([e for e in s['events'] if e['event']=='empty_hand_returned_to_start'])==1
    persisted=json.loads(h.runtime.holding_path.read_text(encoding='utf-8'))
    assert persisted['holding'] is False and persisted['holding_uncertain'] is False and persisted['at_start'] is True
    assert h.runtime._visible_scene is None


def wav_audio(size=160):
    out=io.BytesIO()
    with wave.open(out,'wb') as f:
        f.setnchannels(1);f.setsampwidth(2);f.setframerate(16000);f.writeframes(b'\0\0'*size)
    return out.getvalue()


def upload(audio=None,mime='audio/wav'):
    return {'source':'upload','audio_base64':base64.b64encode(audio if audio is not None else wav_audio()).decode(),'mime_type':mime}


def wait_voice(h,job):
    until=time.monotonic()+3
    while time.monotonic()<until:
        code,job,_=h.http('/api/v2/voice/'+job['voice_id']);assert code==200
        if job['status'] in {'completed','failed'}:return job
        time.sleep(.01)
    raise AssertionError(job)


@pytest.mark.parametrize('mime,audio',[('audio/wav',wav_audio(40000)),('audio/webm;codecs=opus',b'\x1aE\xdf\xa3'+b'controlled'*10)],ids=['wav-large-json','webm'])
def test_voice_upload_http_returns_only_text_with_frozen_config_and_domain_hints(factory,monkeypatch,mime,audio):
    h=factory();h.cfg['voice']={'file_model':'controlled-model','language_codes':['zh-TW'],'custom_vocabulary':['自訂詞']}
    import voice_input.gemini_transcriber as transcriber
    calls=[];paths=[]
    def transcribe(path,**kwargs):
        paths.append(Path(path));calls.append((Path(path).read_bytes(),kwargs));return '把橡皮擦放到右盒子'
    monkeypatch.setattr(transcriber,'transcribe_audio_file',transcribe)
    code,job,headers=h.http('/api/v2/voice',upload(audio,mime));assert code==202
    result=wait_voice(h,job)
    assert result['status']=='completed' and result['text']=='把橡皮擦放到右盒子'
    assert calls[0][0]==audio and calls[0][1]['model']=='controlled-model'
    assert calls[0][1]['language_codes']==['zh-TW']
    hints=calls[0][1]['custom_vocabulary']
    tools=json.loads((h.root/'tools.json').read_text(encoding='utf-8'))
    entities=json.loads((h.root/'world_entities.json').read_text(encoding='utf-8'))['entities']
    assert len(tools)==7 and len(entities)==17
    assert set(tools)|set(entities)|{'右盒子','收納區','自訂詞'}<=set(hints)
    assert not paths[0].exists() and h.calls==h.order==h.backends==[]
    assert not h.runtime._runs and not h.runtime.holding_path.exists()
    assert "media-src 'self' blob:" in headers['Content-Security-Policy']


def test_host_microphone_start_stop_is_separate_from_transcription_and_never_moves(factory,monkeypatch):
    h=factory();entered=threading.Event()
    def record(cfg,stop,seconds,*args):entered.set();assert stop.wait(2);return '受控主機語音'
    monkeypatch.setattr(h.runtime,'_stream_voice',record)
    monkeypatch.setattr(h.runtime,'_transcribe_voice',lambda *args:'受控主機語音')
    code,job,_=h.http('/api/v2/voice',{'source':'microphone'});assert code==202 and job['status']=='recording'
    assert entered.wait(1)
    assert h.http('/api/v2/voice',{'source':'microphone'})[0]==400
    assert h.http('/api/v2/runs',{'text':'受控語意','mode':'real','semantic_source':'api'})[0]==400
    assert h.http('/api/v2/voice',{'action':'stop','voice_id':'wrong'})[0]==400
    assert h.http('/api/v2/voice',{'action':'stop','voice_id':job['voice_id']})[0]==202
    result=wait_voice(h,job);assert result['text']=='受控主機語音'
    assert h.order==h.calls==h.backends==[]


@pytest.mark.parametrize('exc,code',[
    (ModuleNotFoundError('sounddevice'),'voice_host_microphone_unavailable'),
    (PermissionError('permission denied'),'voice_permission_denied'),
    (ConnectionError('network unavailable'),'voice_network_error'),
    (ValueError('voice_empty_audio'),'voice_empty_audio'),
])
def test_host_voice_failure_returns_specific_diagnosis_and_releases_busy_state(factory,monkeypatch,exc,code):
    h=factory()
    def fail(*args):raise exc
    monkeypatch.setattr(h.runtime,'_stream_voice',fail)
    status,job,_=h.http('/api/v2/voice',{'source':'microphone'});assert status==202
    result=wait_voice(h,job)
    assert result['status']=='failed' and result['error']['code']==code and not h.runtime.owns_resources()


@pytest.mark.parametrize('payload',[
    {},{'source':'disk','path':'private.wav'},upload(b'bad'),
    upload(b'RIFF'+b'bad'*10,'audio/webm'),upload(mime='image/png'),
    {'source':'upload','audio_base64':'%%%','mime_type':'audio/wav'},
    {'source':'microphone','record_seconds':999},
])
def test_invalid_voice_requests_never_record_or_transcribe(factory,monkeypatch,payload):
    h=factory()
    monkeypatch.setattr(h.runtime,'_record_voice',lambda *a:pytest.fail('must not record'))
    monkeypatch.setattr(h.runtime,'_transcribe_voice',lambda *a:pytest.fail('must not transcribe'))
    assert h.http('/api/v2/voice',payload)[0]==400
    assert h.runtime._voice_job is None and not h.runtime.owns_resources()


def test_voice_keeps_session_origin_and_existing_json_body_limits(factory):
    h=factory()
    assert h.http('/api/v2/voice',{'source':'microphone'},headers={'X-VisionPick-Token':'wrong'})[0]==403
    assert h.http('/api/v2/voice',{'source':'microphone'},headers={'Origin':'https://example.org'})[0]==403
    assert h.http('/api/v2/runs',{'text':'x'*70000})[0]==400
    h.create();assert h.http('/api/v2/voice',{'source':'microphone'})[0]==400


@pytest.mark.parametrize('size',[65536,70000,6*1024*1024,6*1024*1024+1])
def test_rejected_body_discard_is_bounded_and_restores_socket_timeout(factory,size):
    h=factory();handler=object.__new__(h.server.RequestHandlerClass)
    handler.headers={'Content-Length':str(size)};handler.rfile=io.BytesIO(b'x'*size)
    timeouts=[]
    handler.connection=SimpleNamespace(gettimeout=lambda:5.,settimeout=timeouts.append)
    handler._discard_rejected_body()
    assert handler.rfile.tell()==min(size,6*1024*1024)
    assert handler.close_connection and handler._request_body_read
    assert timeouts[-1]==5. and all(0<t<=.25 for t in timeouts[:-1])
    handler._discard_rejected_body()
    assert handler.rfile.tell()==min(size,6*1024*1024)  # Never consume twice.


def test_rejected_body_discard_uses_total_deadline_not_per_read_timeout(factory,monkeypatch):
    import operator_console.server as transport
    h=factory();handler=object.__new__(h.server.RequestHandlerClass)
    handler.headers={'Content-Length':'100000'};handler.rfile=io.BytesIO(b'x'*100000)
    timeouts=[];handler.connection=SimpleNamespace(gettimeout=lambda:5.,settimeout=timeouts.append)
    clock=iter([0.,0.,.3]);monkeypatch.setattr(transport,'monotonic',lambda:next(clock))
    handler._discard_rejected_body()
    assert handler.rfile.tell()==65536 and timeouts==[.25,5.]
    assert handler.close_connection


def test_host_capture_uses_configured_mono_pcm_wav_and_closes_stream(factory,monkeypatch):
    import sys
    from types import SimpleNamespace
    h=factory();calls=[]
    class Stream:
        def __init__(self,**options):self.options=options;calls.append({k:v for k,v in options.items() if k!='callback'})
        def __enter__(self):self.options['callback'](b'\x01\x00'*16,16,None,None);return self
        def __exit__(self,*args):calls.append('closed')
    monkeypatch.setitem(sys.modules,'sounddevice',SimpleNamespace(query_devices=lambda **kw:calls.append(kw),RawInputStream=Stream))
    stop=threading.Event();stop.set()
    audio=h.runtime._record_voice({'voice':{'sample_rate_hz':16000,'channels':1}},stop,5)
    with wave.open(io.BytesIO(audio),'rb') as f:
        assert (f.getnchannels(),f.getsampwidth(),f.getframerate(),f.getnframes())==(1,2,16000,16)
    assert calls==[{'kind':'input'},{'samplerate':16000,'channels':1,'dtype':'int16'},'closed']


def test_shutdown_stops_host_capture_without_starting_transcription(factory,monkeypatch):
    h=factory();entered=threading.Event()
    def record(cfg,stop,seconds,*args):entered.set();assert stop.wait(3);return '不應使用的關閉結果'
    monkeypatch.setattr(h.runtime,'_stream_voice',record)
    monkeypatch.setattr(h.runtime,'_transcribe_voice',lambda *a:pytest.fail('shutdown must not start transcription'))
    job=h.runtime.voice({'source':'microphone'});assert entered.wait(1)
    h.runtime.shutdown();result=h.runtime.voice_state(job['voice_id'])
    assert result['status']=='failed' and result['error']['code']=='voice_cancelled'
    assert not h.runtime._voice_thread.is_alive() and not h.runtime.owns_resources()


@pytest.mark.parametrize('scenario',['preferences','voice-live','voice-host','voice-insecure','voice-network','scene-confirm'])
def test_browser_preferences_voice_and_scene_consent(factory,monkeypatch,scenario):
    import os
    import subprocess
    if scenario=='preferences':
        h=factory(tasks=[{'target':'eraser','destination_type':'self_relative','destination_name':None,'reference':None,'relation':'right_of','gap_mm':None,'displacement_mm':30}])
    elif scenario=='scene-confirm':
        h=factory(tasks=[task()|{'target':None}]);real(h,known_empty=False)
    else:
        h=factory()
    config_before=Path(h.service.config).read_bytes()
    def recording(cfg,stop,seconds,update,source,audio_queue):
        assert source=='microphone' and audio_queue is None
        if scenario!='voice-network':update('','把橡皮擦')
        assert stop.wait(5),'Browser must be able to stop live microphone streaming'
        return transcribe()
    def transcribe(*args):
        if scenario=='voice-network':raise ConnectionError('controlled network failure')
        return '把橡皮擦放到右盒子'
    monkeypatch.setattr(h.runtime,'_stream_voice',recording)
    root=Path(__file__).resolve().parents[1]
    out=h.root/'browser';out.mkdir(parents=True,exist_ok=True)
    packages=Path.home()/'.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules'
    args={'url':h.url,'scenario':scenario,'output':str(out/(scenario+'.png')),'report':str(out/(scenario+'.json'))}
    result=subprocess.run(['node',str(Path(__file__).with_name('v2_system_repairs_browser.cjs')),json.dumps(args)],capture_output=True,text=True,encoding='utf-8',timeout=50,env={**os.environ,'NODE_PATH':str(packages)})
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    assert Path(h.service.config).read_bytes()==config_before
    if scenario.startswith('voice'):assert h.calls==h.order==h.backends==[]
    if scenario=='scene-confirm':
        assert len(h.calls)==1 and h.order==[('start',1),('capture',1),('start',2),('capture',1)]


def content(**kwargs):
    for key in ('input_transcription','interim_input_transcription'):
        if key in kwargs: kwargs[key]=SimpleNamespace(text=kwargs[key])
    return SimpleNamespace(server_content=SimpleNamespace(**kwargs))


class LiveSession:
    def __init__(self):
        self.messages=asyncio.Queue();self.sent=[];self.closed=False
    async def __aenter__(self): return self
    async def __aexit__(self,*args): self.closed=True
    async def send_realtime_input(self,**payload):
        self.sent.append(payload)
        if 'audio' in payload:
            await self.messages.put(content(interim_input_transcription='把筆'))
        else:
            await self.messages.put(content(input_transcription='把筆放到右盒子'))
            await self.messages.put(content(turn_complete=True))
    async def receive(self):
        while True:
            message=await self.messages.get();yield message
            if getattr(message.server_content,'turn_complete',False):return


def client_for(session,calls):
    def connect(**kwargs): calls.append(kwargs);return session
    return SimpleNamespace(aio=SimpleNamespace(live=SimpleNamespace(connect=connect)))


def test_live_duplex_sends_pcm_before_end_and_publishes_interim_then_final():
    session=LiveSession();calls=[];updates=[]
    text=transcribe_pcm_live(iter([b'\x01\x00'*1600,b'\x02\x00'*1600]),
        on_update=lambda *x:updates.append(x),client=client_for(session,calls))
    assert text=='把筆放到右盒子'
    assert ('','把筆') in updates and updates[-1]==('把筆放到右盒子','')
    assert [list(p) for p in session.sent]==[['audio'],['audio'],['audio_stream_end']]
    assert all(p['audio']['mime_type']=='audio/pcm;rate=16000' for p in session.sent if 'audio' in p)
    assert calls[0]['model']=='gemini-3.5-transcribe-live'
    assert calls[0]['config']['response_modalities']==['TEXT'] and session.closed


def test_live_receiver_continues_after_an_early_completed_turn():
    session=LiveSession();original=session.send_realtime_input
    async def send(**payload):
        if 'audio' in payload:
            session.sent.append(payload)
            await session.messages.put(content(input_transcription='把筆'))
            await session.messages.put(content(turn_complete=True))
        else:
            session.sent.append(payload)
            await session.messages.put(content(input_transcription='放到右盒子'))
            await session.messages.put(content(turn_complete=True))
    session.send_realtime_input=send
    def chunks():
        yield b'\0\0'*1600
        time.sleep(.04)  # First turn completes while capture remains active.
    assert transcribe_pcm_live(chunks(),client=client_for(session,[]))=='把筆 放到右盒子'


@pytest.mark.parametrize('chunk',[b'',b'x','not bytes'])
def test_invalid_pcm_closes_live_session_without_file_fallback(chunk):
    session=LiveSession()
    with pytest.raises(ValueError,match='voice_invalid_pcm_chunk'):
        transcribe_pcm_live([chunk],client=client_for(session,[]))
    assert session.closed


def test_provider_error_unblocks_capture_and_cancels_stream():
    stop=threading.Event();session=LiveSession()
    async def fail():
        raise ConnectionError('controlled disconnect')
        yield
    session.receive=fail
    def chunks():
        while not stop.wait(.01):pass
        return
        yield
    with pytest.raises(ConnectionError):
        transcribe_pcm_live(chunks(),stop_event=stop,client=client_for(session,[]))
    assert stop.is_set() and session.closed


def controlled_stream(monkeypatch,h,*,fail=False):
    entered=threading.Event();seen=[]
    def stream(cfg,stop,seconds,update,source,audio_queue):
        entered.set()
        if source=='stream':
            seen.append(audio_queue.get(timeout=2))
        update('','把筆')
        assert stop.wait(3)
        if fail:raise ConnectionError('controlled network failure')
        update('把筆放到右盒子','')
        return '把筆放到右盒子'
    monkeypatch.setattr(h.runtime,'_stream_voice',stream)
    return entered,seen


def wait_done(h,job):
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        current=h.runtime.voice_state(job['voice_id'])
        if current['status'] in {'failed','completed'}:return current
        time.sleep(.01)
    raise AssertionError(current)


def test_pcm_http_stream_updates_text_before_stop_and_never_creates_a_robot(factory,monkeypatch):
    h=factory();entered,seen=controlled_stream(monkeypatch,h)
    code,job,_=h.http('/api/v2/voice',{'source':'stream'});assert code==202 and entered.wait(1)
    chunk={'action':'chunk','voice_id':job['voice_id'],'sequence':0,'audio_base64':base64.b64encode(b'\1\0'*1600).decode()}
    assert h.http('/api/v2/voice',chunk)[0]==202
    deadline=time.monotonic()+2
    while h.runtime.voice_state(job['voice_id'])['text']!='把筆' and time.monotonic()<deadline:time.sleep(.01)
    state=h.runtime.voice_state(job['voice_id'])
    assert state['status']=='recording' and state['text']=='把筆' and state['interim_text']=='把筆'
    assert seen==[b'\1\0'*1600]
    assert h.http('/api/v2/voice',chunk)[0]==400  # Duplicate sequence cannot replay audio.
    assert h.http('/api/v2/voice',{'action':'stop','voice_id':job['voice_id']})[0]==202
    assert wait_done(h,job)['text']=='把筆放到右盒子'
    assert h.calls==h.order==h.backends==[] and not h.runtime._runs


@pytest.mark.parametrize('patch',[{'sequence':True},{'sequence':2},{'audio_base64':'%%%'},
    {'audio_base64':'eA=='},{'audio_base64':''},{'audio_base64':base64.b64encode(b'xx'*5000).decode()}])
def test_invalid_stream_chunk_is_rejected_without_advancing_sequence(factory,monkeypatch,patch):
    h=factory();entered=threading.Event()
    def stream(cfg,stop,seconds,update,source,audio_queue):
        entered.set();stop.wait(3);return '受控文字'
    monkeypatch.setattr(h.runtime,'_stream_voice',stream)
    job=h.runtime.voice({'source':'stream'});assert entered.wait(1)
    payload={'action':'chunk','voice_id':job['voice_id'],'sequence':0,'audio_base64':'AAA='}|patch
    assert h.http('/api/v2/voice',payload)[0]==400 and h.runtime._voice_next_chunk==0
    h.runtime.voice({'action':'stop','voice_id':job['voice_id']});wait_done(h,job)


def test_host_stream_uses_live_model_and_no_wav_file(factory,monkeypatch):
    import sys
    import voice_input.gemini_live_transcriber as live
    h=factory();sent=[];cfg={'voice':{'live_model':'controlled-live','sample_rate_hz':16000,'channels':1}}
    class Stream:
        def __init__(self,**kwargs):sent.append(kwargs)
        def __enter__(self):return self
        def __exit__(self,*args):sent.append('closed')
        def read(self,frames):return b'\1\0'*frames,False
    monkeypatch.setitem(sys.modules,'sounddevice',SimpleNamespace(query_devices=lambda **k:None,RawInputStream=Stream))
    def transcribe(chunks,**kwargs):
        audio=list(chunks)
        assert len(audio)==1 and len(audio[0])==3200 and kwargs['model']=='controlled-live'
        kwargs['on_update']('','逐字稿');return '逐字稿'
    monkeypatch.setattr(live,'transcribe_pcm_live',transcribe)
    h.runtime._voice_job={}
    assert h.runtime._stream_voice(cfg,threading.Event(),.1,lambda *x:None,'microphone',None)=='逐字稿'
    assert sent[-1]=='closed'


@pytest.mark.parametrize('reason',['shutdown','deadline'])
def test_pending_provider_response_is_cancelled_and_releases_capture(reason):
    session=LiveSession();stop=threading.Event();started=time.monotonic()
    async def send(**payload):session.sent.append(payload)
    session.send_realtime_input=send
    kwargs={'cancelled':lambda:time.monotonic()-started>.04} if reason=='shutdown' else {'session_timeout_s':.04}
    with pytest.raises((ValueError,TimeoutError)):
        transcribe_pcm_live([b'\0\0'*100],client=client_for(session,[]),stop_event=stop,**kwargs)
    assert stop.is_set() and session.closed and time.monotonic()-started<1


def test_stopping_before_first_pcm_chunk_reports_empty_audio_without_final_wait():
    session=LiveSession();started=time.monotonic()
    with pytest.raises(ValueError,match='voice_empty_audio'):
        transcribe_pcm_live([],client=client_for(session,[]))
    assert session.closed and time.monotonic()-started<1
