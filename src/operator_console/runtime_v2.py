"""V2 consumer: one semantic request, local answers, one confirmed 1–20 step loop.

Native V2 routes own engineering, stations and calibration. Shared hybrid
solvers control transfers; no retired task runtime or slot resolver is loaded.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import json
import math
import threading
import time
import uuid

from .arrival_settings_v2 import ArrivalSettingsStoreV2
from .direct_control_v2 import DirectRobotControllerV2, _six
from .events import atomic_json, redact
from .lease import RobotLease
from .place_catalog_v2 import PlaceCatalogV2
from .task_parser_v2 import parse_task_plan_v2, validate_task_plan_v2_candidate
from .task_pending_v2 import PendingTaskPlanV2
from .task_sequence_v2 import ObservationV2, TaskSequenceV2


RELATIONS = ['left_of', 'right_of', 'in_front_of', 'behind', 'above', 'below', 'near', 'on_top_of', 'at_initial']
DEMO_TEXT = '把橡皮擦放到筆的右邊'


def _identity(value):
    import hashlib
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def _direction_frame(config):
    from integration.scene_grounding import normalize_rotation_deg
    return {'coordinate_frame':'display_workbench_plane',
            'rotation_deg':normalize_rotation_deg(config.get('scene',{}).get('workspace_view_rotation_deg',90)),
            'self_anchor_policy':'step_pre_pick'}


class ConsoleRuntimeV2:
    def __init__(self, service, *, semantic_parser=None, scene_factory=None, controller_factory=None):
        self.service = service
        self.root = service.root
        self.runs_root = service.runs_root / 'v2'
        self.settings = ArrivalSettingsStoreV2(self.root, config_loader=service._base_config)
        self.semantic_parser = semantic_parser
        self.scene_factory = scene_factory
        self.controller_factory = controller_factory
        self._lock = threading.RLock()
        self._runs = {}
        self._active = None
        self._thread = None
        self._closing = False
        self._visible_scene = None
        self._arm_motion_epoch = 0
        self._voice_job = None
        self._voice_thread = None
        self._voice_stop = threading.Event()
        self._devices = {'camera': {'detected': False, 'count': 0}, 'robot': {'detected': False}}
        self.holding_path = self.root / 'operator_data' / 'v2_controller_state.json'
        from .engineering_v2 import EngineeringConsoleV2
        self.engineering = EngineeringConsoleV2(self)

    def _write_controller_state(self, data):
        with self._lock:
            self.holding_path.parent.mkdir(parents=True, exist_ok=True)
            atomic_json(self.holding_path, data)
            run = self._runs.get(data.get('run_id'))
            if run is not None:
                self._save(run)  # Holding transitions must reach both durable files.

    def _holding(self):
        with self._lock:
            if not self.holding_path.exists():
                return False
            if self.holding_path.is_symlink():
                raise ValueError('controller_state_symlink_forbidden')
            data = json.loads(self.holding_path.read_text(encoding='utf-8'))
            if type(data.get('holding_uncertain')) is not bool:
                raise ValueError('invalid_persisted_controller_state')
            return data['holding_uncertain']

    def _empty_hand_confirmed(self):
        # Missing state is unknown, even though _holding() historically returns False.
        return self.holding_path.exists() and not self._holding()

    def _arm_state_marker(self):
        import hashlib
        raw = self.holding_path.read_bytes() if self.holding_path.exists() else b''
        return hashlib.sha256(raw).hexdigest()

    def _accept_observation_consent(self, payload):
        if type(payload.get('move_to_start', False)) is not bool:
            raise ValueError('move_to_start_must_be_boolean')
        if 'source' in payload and payload['source'] != 'operator_empty_hand_check':
            raise ValueError('invalid_empty_hand_confirmation_source')
        if payload.get('source') == 'operator_empty_hand_check':
            self._maintenance_idle()
            if self.owns_resources(): raise ValueError('v2_worker_busy')
            self.holding_path.parent.mkdir(parents=True, exist_ok=True)
            self._write_controller_state({'holding_uncertain':False,'holding':False,'at_start':False,
                                          'source':'operator_empty_hand_check'})
            self._visible_scene = None

    def owns_resources(self):
        with self._lock:
            run = self._runs.get(self._active)
            return bool(self._voice_busy() or self._thread and self._thread.is_alive()
                        or run and (run.get('lease') or run.get('cleanup_error')))

    def _voice_busy(self):
        return self._voice_job is not None and self._voice_job['status'] in {'recording','transcribing'}

    def voice_state(self, voice_id):
        with self._lock:
            if self._voice_job is None or voice_id != self._voice_job['voice_id']:
                raise ValueError('voice_session_not_found')
            return deepcopy(self._voice_job)

    def _voice_vocabulary(self, cfg):
        from voice_input.voice_input import build_custom_vocabulary
        values = build_custom_vocabulary(cfg)
        for name in ('tools.json','world_entities.json'):
            rows = json.loads((self.root/name).read_text(encoding='utf-8'))
            for key, row in rows.get('entities',rows).items():
                values.extend([key, *row.get('aliases',[])])
        for station in self._catalog().prompt_stations():
            values.extend([station['name'], *station['aliases']])
        return list(dict.fromkeys(v.strip() for v in values if isinstance(v,str) and v.strip()))

    def _record_voice(self, cfg, stop, seconds):
        import sounddevice as sd
        import wave
        import io
        voice = cfg.get('voice',{})
        rate = voice.get('sample_rate_hz',16000)
        if (type(rate) is not int or not 8000 <= rate <= 48000
                or type(voice.get('channels',1)) is not int or voice.get('channels',1) != 1):
            raise ValueError('voice_invalid_capture_settings')
        chunks, errors = [], []
        def captured(indata, frames, timing, status):
            if status: errors.append(str(status))
            chunks.append(bytes(indata))
        sd.query_devices(kind='input')  # Precise no-input diagnosis before opening the stream.
        with sd.RawInputStream(samplerate=rate,channels=1,dtype='int16',callback=captured):
            stop.wait(seconds)
        if errors: raise RuntimeError('voice_audio_capture_failed: '+errors[0])
        pcm = b''.join(chunks)
        if not pcm: raise ValueError('voice_empty_audio')
        output = io.BytesIO()
        with wave.open(output,'wb') as wav:
            wav.setnchannels(1); wav.setsampwidth(2); wav.setframerate(rate); wav.writeframes(pcm)
        return output.getvalue()

    def _transcribe_voice(self, audio, mime, cfg):
        import tempfile
        from voice_input.gemini_transcriber import transcribe_audio_file
        suffix = {'audio/wav':'.wav','audio/webm':'.webm','audio/ogg':'.ogg','audio/mp4':'.m4a'}[mime]
        with tempfile.TemporaryDirectory(prefix='visionpick_v2_voice_') as folder:
            path = Path(folder)/('command'+suffix)
            path.write_bytes(audio)
            voice = cfg.get('voice',{})
            return transcribe_audio_file(path,model=voice.get('file_model',voice.get('model','gemini-3.5-transcribe')),
                language_codes=voice.get('language_codes',[]),custom_vocabulary=self._voice_vocabulary(cfg))

    def _stream_voice(self, cfg, stop, seconds, on_update, source, audio_queue):
        from voice_input.gemini_live_transcriber import transcribe_pcm_live
        import queue
        voice = cfg.get('voice',{})
        if voice.get('sample_rate_hz',16000) != 16000 or voice.get('channels',1) != 1:
            raise ValueError('voice_invalid_capture_settings')
        chunk_ms = voice.get('live_chunk_ms',100)
        if type(chunk_ms) is not int or not 50 <= chunk_ms <= 250:
            raise ValueError('voice_invalid_chunk_ms')
        def browser_chunks():
            deadline = time.monotonic()+seconds
            remaining = int(seconds*32000)
            while remaining > 0:
                if time.monotonic() >= deadline:
                    stop.set()
                try:
                    chunk = audio_queue.get(timeout=.1)
                except queue.Empty:
                    if stop.is_set(): break
                    continue
                chunk = chunk[:remaining];remaining -= len(chunk)
                yield chunk
                if stop.is_set() and audio_queue.empty(): break
        def host_chunks():
            import sounddevice as sd
            sd.query_devices(kind='input')
            frames = round(16000*chunk_ms/1000)
            remaining = int(seconds*16000)
            with sd.RawInputStream(samplerate=16000,channels=1,dtype='int16',blocksize=frames) as stream:
                while not stop.is_set() and remaining > 0:
                    count = min(frames,remaining)
                    data, overflow = stream.read(count)
                    # Aligned with 1013: minor PortAudio buffer overflow warning on Windows is non-fatal
                    pass
                    if not data: raise ValueError('voice_empty_audio')
                    remaining -= count
                    chunk_bytes = bytes(data)
                    yield chunk_bytes
        def chunks():
            try:
                yield from (browser_chunks() if source=='stream' else host_chunks())
            finally:
                with self._lock:
                    self._voice_job['status'] = 'transcribing'
        return transcribe_pcm_live(chunks(),model=voice.get('live_model','gemini-3.5-transcribe-live'),
            language_codes=voice.get('language_codes',[]),custom_vocabulary=self._voice_vocabulary(cfg),
            mode=voice.get('live_mode','VERBATIM'),final_timeout_seconds=voice.get('live_final_timeout_s',10.),
            on_update=on_update,stop_event=stop,cancelled=lambda:self._closing,
            session_timeout_s=seconds+float(voice.get('live_final_timeout_s',10.))+10.)

    def voice(self, payload):
        """Voice is text-only: no parser, perception, lease or robot command is invoked."""
        import base64
        import queue
        if not isinstance(payload,dict): raise ValueError('invalid_voice_request')
        with self.service._lock, self._lock:
            if payload.get('action') in {'stop','chunk'}:
                current = self.voice_state(payload.get('voice_id'))
                if payload['action']=='stop':
                    if set(payload) != {'action','voice_id'}: raise ValueError('invalid_voice_stop_request')
                    self._voice_stop.set()
                    return current
                if set(payload) != {'action','voice_id','sequence','audio_base64'}:
                    raise ValueError('invalid_voice_chunk_request')
                if current['source']!='stream' or current['status']!='recording' or self._voice_stop.is_set():
                    raise ValueError('voice_stream_not_recording')
                if type(payload['sequence']) is not int or payload['sequence']!=self._voice_next_chunk:
                    raise ValueError('voice_chunk_out_of_order')
                encoded = payload['audio_base64']
                if not isinstance(encoded,str) or len(encoded)>10668: raise ValueError('voice_invalid_pcm_chunk')
                try: chunk = base64.b64decode(encoded,validate=True)
                except (ValueError,TypeError) as exc: raise ValueError('voice_invalid_audio_base64') from exc
                if not chunk or len(chunk)>8000 or len(chunk)%2: raise ValueError('voice_invalid_pcm_chunk')
                if self._voice_bytes+len(chunk)>int(current['record_seconds']*32000)+8000:
                    raise ValueError('voice_audio_too_large')
                try: self._voice_queue.put_nowait(chunk)
                except queue.Full as exc: raise ValueError('voice_stream_backpressure') from exc
                self._voice_next_chunk += 1
                self._voice_bytes += len(chunk)
                return {'voice_id':current['voice_id'],'sequence':payload['sequence'],'status':'accepted'}
            if self._closing: raise ValueError('v2_runtime_shutting_down')
            self._maintenance_idle()
            if self.owns_resources(): raise ValueError('voice_busy_or_operation_active')
            if self._active and self.state(self._active)['status'] not in {'completed','failed','cancelled','expired','blocked'}:
                raise ValueError('voice_operation_active')
            source = payload.get('source')
            audio = None
            cfg = deepcopy(self.service._base_config())
            seconds = cfg.get('voice',{}).get('record_seconds',5.0)
            if isinstance(seconds,bool) or not isinstance(seconds,(float,int)) or not math.isfinite(seconds) or not 0 < seconds <= 30:
                raise ValueError('voice_record_seconds_must_be_0_to_30')
            if source == 'upload':
                if set(payload) != {'source','audio_base64','mime_type'} or not isinstance(payload['audio_base64'],str):
                    raise ValueError('invalid_voice_upload')
                if len(payload['audio_base64']) > 5592408: raise ValueError('voice_audio_too_large')
                mime = str(payload['mime_type']).split(';')[0].strip().lower()
                if mime == 'audio/x-wav': mime = 'audio/wav'
                if mime not in {'audio/wav','audio/webm','audio/ogg','audio/mp4'}: raise ValueError('voice_unsupported_audio_type')
                try: audio = base64.b64decode(payload['audio_base64'],validate=True)
                except (ValueError,TypeError) as exc: raise ValueError('voice_invalid_audio_base64') from exc
                if not 12 <= len(audio) <= 4194304: raise ValueError('voice_empty_or_oversized_audio')
                valid = (mime=='audio/wav' and audio[:4]==b'RIFF' and audio[8:12]==b'WAVE'
                         or mime=='audio/webm' and audio[:4]==b'\x1aE\xdf\xa3'
                         or mime=='audio/ogg' and audio[:4]==b'OggS'
                         or mime=='audio/mp4' and audio[4:8]==b'ftyp')
                if not valid: raise ValueError('voice_audio_header_mismatch')
            elif source in {'microphone','stream'}:
                if set(payload) != {'source'}: raise ValueError('invalid_voice_microphone_request')
                mime = 'audio/wav'
            else: raise ValueError('voice_source_must_be_upload_microphone_or_stream')
            self._voice_stop = threading.Event()
            self._voice_queue = queue.Queue(maxsize=64) if source=='stream' else None
            self._voice_next_chunk = self._voice_bytes = 0
            self._voice_job = {'voice_id':uuid.uuid4().hex,'source':source,'status':'recording' if audio is None else 'transcribing',
                               'record_seconds':seconds,'text':'','committed_text':'','interim_text':'','error':None}
            initial = deepcopy(self._voice_job)
            def work():
                try:
                    def update(committed, interim):
                        with self._lock:
                            if self._closing: raise ValueError('voice_cancelled')
                            self._voice_job.update(committed_text=committed,interim_text=interim,
                                text=' '.join(x for x in (committed,interim) if x))
                    if source in {'microphone','stream'}:
                        text = self._stream_voice(cfg,self._voice_stop,seconds,update,source,self._voice_queue)
                    else:
                        text = self._transcribe_voice(audio,mime,cfg)
                    if self._closing: raise ValueError('voice_cancelled')
                    text = str(text or '').strip()
                    if not text: raise ValueError('voice_empty_transcript')
                    if len(text)>4000: raise ValueError('voice_transcript_too_long')
                    with self._lock: self._voice_job.update(status='completed',text=text,committed_text=text,interim_text='')
                except Exception as exc:
                    self._voice_stop.set()
                    detail = str(redact(str(exc)))[:400]
                    if isinstance(exc,ImportError) or type(exc).__name__=='PortAudioError': code = 'voice_host_microphone_unavailable' if source=='microphone' else 'voice_dependency_missing'
                    elif any(s in (type(exc).__name__+' '+detail).lower() for s in ['network','connect','timeout','unavailable','503']): code = 'voice_network_error'
                    elif 'permission' in detail.lower() or 'denied' in detail.lower(): code = 'voice_permission_denied'
                    else: code = detail if detail.startswith('voice_') else 'voice_transcription_failed'
                    with self._lock: self._voice_job.update(status='failed',error={'code':code,'message':detail,'type':type(exc).__name__})
            self._voice_thread = threading.Thread(target=work,daemon=True,name='visionpick-v2-voice')
            self._voice_thread.start()
            return initial

    def _maintenance_idle(self):
        if getattr(self, '_teaching_controller', None) is not None:
            raise ValueError('teaching_session_active')
        if getattr(self.service, '_quarantined_leases', []):
            raise ValueError('robot_ownership_quarantined')

    def _catalog(self):
        return PlaceCatalogV2(self.service.place_profiles.status())

    def read_pose(self, payload):
        """Read paired joints/TCP through the V2 controller without motion."""
        if payload:
            raise ValueError('pose_requires_empty_payload')
        with self.service._lock, self._lock:
            if not self.service.hardware_enabled:
                raise RuntimeError('hardware_disabled')
            teaching = getattr(self, '_teaching_controller', None)
            if self.owns_resources() or self._closing:
                raise ValueError('engineering_locked_while_running')
            ctrl, lease = teaching, None
            if teaching is None:
                self._maintenance_idle()
                cfg = self.service._base_config()
                lease = RobotLease(cfg['robot']['port']).acquire()
            try:
                if teaching is None:
                    ctrl = DirectRobotControllerV2(cfg, settings=self.settings.freeze().settings)
                    ctrl.connect()
                return {'coords_mm_deg': _six(ctrl.get_coords(), 'fresh_tcp'),
                        'angles_deg': _six(ctrl._try_get_angles(), 'fresh_joints'),
                        'read_at_unix_ms': time.time()*1000,
                        'motion_command_sent': False, 'gripper_command_sent': False,
                        'teaching_mode_active': teaching is not None,
                        'config_revision': _identity(self.service._base_config()), 'arm_epoch': self._arm_motion_epoch}
            finally:
                if lease is not None:
                    if ctrl is None: lease.release()
                    else: self._close_maintenance(ctrl, lease)

    def home(self, payload):
        if payload != {'confirm': 'RETURN_HOME'}:
            raise ValueError('explicit_return_home_confirmation_required')
        with self.engineering._idle():
            if not self.service.hardware_enabled or not self.service.motion_enabled:
                raise ValueError('real_motion_not_enabled_for_this_server')
            from v0_mvp.motion_safety import validate_motion_unlock
            cfg = deepcopy(self.service._base_config())
            validate_motion_unlock(cfg)
            run_id = uuid.uuid4().hex
            self._path(run_id).mkdir(parents=True, exist_ok=False)
            run = {'id': run_id, 'mode': 'real', 'operation': 'home', 'semantic_source': 'none',
                   'status': 'running', 'config': cfg, 'config_revision': _identity(cfg),
                   'arrival': self.settings.freeze(), 'events': [], 'frames': {}, 'cancelled': False}
            self._runs[run_id], self._active = run, run_id
            self._calibration_frame = None
            self._visible_scene = None
            def move():
                def checkpoint():
                    if run['cancelled']: raise ValueError('operator_cancelled')
                try:
                    self._initialize(run, 1, checkpoint=checkpoint)
                    checkpoint()
                    run['status'] = 'completed'
                finally:
                    self._cleanup(run)
                    if run['cancelled']: run['status'] = 'cancelled'
            self._launch(run, move)
            return self.state(run_id)

    def _close_maintenance(self, ctrl, lease):
        try:
            if getattr(ctrl, 'mc', None) is not None and callable(getattr(ctrl.mc, 'close', None)):
                ctrl.mc.close()
        except Exception:
            self.service._quarantined_leases.append(lease)
            raise
        else:
            lease.release()

    def teaching(self, payload):
        action = payload.get('action')
        if payload not in ({'action': 'enter', 'confirm': 'ENTER_TEACH_MODE'},
                           {'action': 'exit', 'confirm': 'EXIT_TEACH_MODE'}):
            raise ValueError('explicit_teaching_confirmation_required')
        with self.service._lock, self._lock:
            ctrl = getattr(self, '_teaching_controller', None)
            if action == 'exit':
                if ctrl is None:
                    return {'active': False}
                # Retain the connection and lease when torque restoration fails.
                ctrl.exit_free_mode()
                self._teaching_controller = None
                self._close_maintenance(ctrl, self._teaching_lease)
                self._teaching_lease = None
                self.engineering._event('teaching_exited')
                return {'active': False}
            if ctrl is not None:
                return {'active': True}
            with self.engineering._idle():
                if not self.service.hardware_enabled or not self.service.motion_enabled:
                    raise ValueError('real_motion_not_enabled_for_this_server')
                if not self._empty_hand_confirmed():
                    raise ValueError('gripper_holding_uncertain_requires_manual_recovery')
                cfg = self.service._base_config()
                lease = RobotLease(cfg['robot']['port']).acquire()
                ctrl, free_attempted = None, False
                try:
                    ctrl = DirectRobotControllerV2(cfg, settings=self.settings.freeze().settings)
                    ctrl.connect()
                    self._write_controller_state({'holding_uncertain': True, 'reason': 'manual_teaching'})
                    self._arm_motion_epoch += 1
                    self._visible_scene = None
                    self._calibration_frame = None
                    free_attempted = True
                    ctrl.enter_free_mode()
                except Exception:
                    if free_attempted:
                        # Torque release may have reached the controller.
                        self._teaching_controller, self._teaching_lease = ctrl, lease
                    elif ctrl is None:
                        lease.release()
                    else:
                        self._close_maintenance(ctrl, lease)
                    raise
                self._teaching_controller, self._teaching_lease = ctrl, lease
                self.engineering._event('teaching_entered')
                return {'active': True}

    def capture_rgbd(self, payload):
        if set(payload) - {'source'} or payload.get('source', 'new') not in {'new', 'current'}:
            raise ValueError('invalid_capture_request')
        with self.engineering._idle():
            if not self.service.hardware_enabled:
                raise RuntimeError('hardware_disabled')
            if payload.get('source') == 'current':
                current = getattr(self, '_calibration_frame', None)
                if current is None or current['synthetic']:
                    raise ValueError('current_real_rgbd_frame_required')
                if time.monotonic() - current['at'] > 60:
                    raise ValueError('current_rgbd_frame_expired_capture_again')
                if current['config_revision'] != _identity(self.service._base_config()):
                    raise ValueError('capture_configuration_changed')
                result = self.service.place_capture.persist(current['frame'], self.service._base_config())
            else:
                result = self.service.place_capture.capture()
            self.engineering._event('rgbd_captured', capture_id=result['capture_id'])
            return result

    def _resolve_roi(self, payload):
        """Display-edge coordinates -> raw pixel ROI -> frozen RGB-D XYZ."""
        import math
        capture_id = payload.get('capture_id')
        path = self.service.place_capture._path(capture_id) / 'metadata.json'
        if path.is_symlink():
            raise ValueError('capture_symlink_forbidden')
        meta = json.loads(path.read_text(encoding='utf-8'))
        cfg = self.service._base_config()
        if (meta.get('calibration_matrix') != cfg.get('calibration', {}).get('T_robot_from_camera')
                or meta.get('camera_device_id') != cfg.get('camera', {}).get('device_id')
                or meta.get('workspace_mm') != cfg.get('grasp', {}).get('workspace_mm')
                or meta.get('depth_policy') != cfg.get('depth', {})):
            raise ValueError('capture_configuration_changed')
        roi, rotation = payload.get('roi_xyxy'), payload.get('rotation_deg', 0)
        if type(rotation) is not int or rotation not in {0, 90, 180, 270}:
            raise ValueError('rotation_must_be_0_90_180_270')
        if (not isinstance(roi, list) or len(roi) != 4
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in roi)):
            raise ValueError('roi_requires_four_finite_display_coordinates')
        w, h = meta['width'], meta['height']
        dw, dh = (h, w) if rotation in {90, 270} else (w, h)
        a, b, c, d = roi
        if not (0 <= a <= dw and 0 <= c <= dw and 0 <= b <= dh and 0 <= d <= dh):
            raise ValueError('roi_outside_display_frame')
        if abs(a-c) < 2 or abs(b-d) < 2:
            raise ValueError('roi_must_span_at_least_two_pixels')
        def inverse(x, y):
            return ((w-y, x) if rotation == 90 else (w-x, h-y) if rotation == 180
                    else (y, h-x) if rotation == 270 else (x, y))
        points = [inverse(x, y) for x in (a, c) for y in (b, d)]
        xs, ys = zip(*points)
        result = self.service.place_capture.resolve({'capture_id': capture_id,
            'u0': math.floor(min(xs)), 'v0': math.floor(min(ys)),
            'u1': math.ceil(max(xs))-1, 'v1': math.ceil(max(ys))-1,
            'z_offset_mm': payload.get('z_offset_mm', 0)})
        return {**result, 'roi_xyxy': roi, 'rotation_deg': rotation}, meta

    def roi_capture(self, payload):
        allowed = {'capture_id', 'roi_xyxy', 'rotation_deg', 'z_offset_mm', 'rz_deg',
                   'save', 'name', 'aliases', 'profile_id', 'clear_path_acknowledged'}
        if set(payload) - allowed or type(payload.get('save', False)) is not bool:
            raise ValueError('invalid_roi_capture_fields')
        with self.engineering._idle():
            result, meta = self._resolve_roi(payload)
            rz = payload.get('rz_deg', self.service._base_config().get('orientation', {}).get('fixed_place_yaw_deg', -45.))
            if type(rz) not in (int, float) or not math.isfinite(rz) or not -180 <= rz <= 180:
                raise ValueError('rz_out_of_range')
            pose = [*result['place_xyz_mm'], -175., -5., float(rz)]
            result['place_pose_mm_deg'] = pose
            if not payload.get('save', False):
                return result
            if not result['workspace_ok']:
                raise ValueError('roi_outside_robot_workspace')
            if payload.get('clear_path_acknowledged') is not True:
                raise ValueError('clear_path_acknowledgement_required')
            with self.engineering._transaction():
                manager = self.service.place_profiles
                created = manager.create_candidate({'label': payload.get('name'), 'aliases': payload.get('aliases', []),
                    'base_profile_id': payload.get('profile_id') or None,
                    'reference_object': 'table', 'spatial_relation': 'on_top_of', 'planner_mode': 'hybrid',
                    'place_pose_mm_deg': pose, 'approach_clearance_mm': 60, 'retreat_clearance_mm': 80,
                    'placement_source': {'type': 'visual_manual_selection', 'capture_id': meta['capture_id'],
                                         'record': result}})
                profile = manager.activate(created['profile']['profile_id'], confirm='ACTIVATE_PLACE_PROFILE',
                                           clear_path_acknowledged=True)
                self._catalog()._valid_catalog()
                snapshot = self.service.assets.register_current()
                self.engineering._event('roi_station_saved', profile_id=profile['profile_id'], capture_id=meta['capture_id'])
                return {**result, 'profile': profile, 'asset_snapshot_id': snapshot['snapshot_id'], 'catalog': self._catalog().status()}

    def table_calibrate(self, payload):
        allowed = {'capture_id', 'roi_xyxy', 'rotation_deg', 'confirmed', 'z_mm', 'note'}
        if set(payload) - allowed:
            raise ValueError('invalid_table_calibration_fields')
        with self.engineering._idle(), self.engineering._transaction(include_table=True):
            cfg = self.service._base_config()
            manager = self.service.place_profiles
            previous = manager._dependencies()
            eligible = [r['profile_id'] for r in manager.status()['profiles'] if r['effective_status'] == 'active']
            provenance = {}
            if 'z_mm' in payload:
                if 'capture_id' in payload or 'roi_xyxy' in payload:
                    raise ValueError('choose_manual_or_rgbd_table_measurement')
                z = payload['z_mm']
            else:
                selection = dict(payload)
                if 'capture_id' not in selection:
                    capture = self.capture_rgbd({'source': 'new'})
                    selection['capture_id'] = capture['capture_id']
                if 'roi_xyxy' not in selection:
                    path = self.service.place_capture._path(selection['capture_id']) / 'metadata.json'
                    meta = json.loads(path.read_text(encoding='utf-8'))
                    w, h = meta['width'], meta['height']
                    if selection.get('rotation_deg', 0) in {90, 270}: w, h = h, w
                    selection['roi_xyxy'] = [w*.35, h*.35, w*.65, h*.65]
                result, _ = self._resolve_roi(selection)
                if result['depth_mad_m']*1000 > float(cfg.get('scene', {}).get('surface_tolerance_mm', 25)):
                    raise ValueError('selected_table_surface_not_flat')
                z, provenance = result['robot_surface_xyz_mm'][2], result
            table = self.service.table_surface.save(cfg, z_mm=z, confirmed=payload.get('confirmed'),
                method='camera_selection' if provenance else 'manual_measurement', note=payload.get('note', ''), provenance=provenance)
            synced = manager.sync_motion_fingerprint(previous, eligible)
            snapshot = self.service.assets.register_current()
            self._visible_scene = None
            self.engineering._event('table_calibrated', z_mm=z, synced_profile_ids=synced)
            return {**table, 'synced_profile_ids': synced, 'asset_snapshot_id': snapshot['snapshot_id']}

    def _discover_devices(self):
        # Passive enumeration only: never open a serial port or issue motion here.
        discover = getattr(self.service, 'device_status', None)
        if discover is not None:
            self._devices = deepcopy(discover())
        return deepcopy(self._devices)

    def _require_devices(self):
        devices = self._discover_devices()
        if (devices.get('camera', {}).get('detected') is not True
                or devices.get('camera', {}).get('count') == 0
                or devices.get('robot', {}).get('detected') is not True):
            raise ValueError('hardware_not_detected')
        return devices

    def bootstrap(self):
        cfg = self.service._base_config()
        devices = self._discover_devices()
        from object_catalog import class_display_name, graspable_classes
        object_labels = {name:class_display_name(name) for name in graspable_classes()}
        world_path = self.root / 'world_entities.json'
        if world_path.is_file():
            entities = json.loads(world_path.read_text(encoding='utf-8')).get('entities', {})
            for name, item in entities.items():
                aliases = item.get('aliases') or []
                object_labels.setdefault(name, aliases[0] if aliases else name)
        start_ready = False
        start_angles_ready = False
        try:
            angles = _six(cfg['robot'].get('start_angles_deg'), 'start_angles')
            start_angles_ready = all(abs(a) <= 360 for a in angles)
        except (ValueError, RuntimeError, KeyError):
            pass
        try:
            DirectRobotControllerV2(cfg)._pose(cfg['robot'].get('start_pose_mm_deg'))
            start_ready = True
        except (ValueError, RuntimeError, KeyError):
            pass
        return {'active_run': self._active, 'settings': self.settings.read(), 'devices': devices,
                'direction_frame':_direction_frame(cfg),
                'controlled_semantic': self.semantic_parser is not None,
                'voice':{'record_seconds':cfg.get('voice',{}).get('record_seconds',5.0)},
                'catalog': self._catalog().status(), 'table': self.service.table_surface.status(cfg),
                'hardware_enabled': self.service.hardware_enabled,
                'motion_enabled': self.service.motion_enabled, 'holding_uncertain': self._holding(),
                'empty_hand_confirmed': self._empty_hand_confirmed(),
                'initialization_policy': 'each_task_before_capture', 'start_tcp_ready':start_ready,
                'start_angles_ready':start_angles_ready, 'start_ready':start_angles_ready,
                'start_verification':'fresh_joint_feedback',
                'object_labels':object_labels, 'demo_text': DEMO_TEXT}

    def _path(self, run_id):
        if not isinstance(run_id, str) or len(run_id) != 32 or any(c not in '0123456789abcdef' for c in run_id):
            raise ValueError('invalid_v2_run_id')
        path = self.runs_root / run_id
        if path.is_symlink():
            raise ValueError('v2_run_symlink_forbidden')
        return path

    def state(self, run_id):
        with self._lock:
            run = self._runs.get(run_id)
            if run is None:
                saved = json.loads((self._path(run_id) / 'state.json').read_text(encoding='utf-8'))
                return {**saved, 'read_only': True}
            pending = run['pending'].snapshot() if run.get('pending') else None
            sequence = run['sequence'].snapshot() if run.get('sequence') else None
            status = sequence['status'] if sequence else pending['status'] if pending else run['status']
            return redact({'run_id': run_id, 'status': status, 'mode': run['mode'],
                'direction_frame':_direction_frame(run['config']),
                'operation': run.get('operation', 'task'), 'preview': run.get('preview'),
                'final_preview':deepcopy(run.get('final_preview')),
                'completion_capture':deepcopy(run.get('completion_capture')),
                'placement_previews':deepcopy(run.get('placement_previews',[])),
                'semantic_scene': {'visible_objects': run.get('visible_objects'),
                                   'frame_id': run.get('visible_frame_id')},
                'devices': deepcopy(self._devices),
                'holding_uncertain':self._holding(), 'empty_hand_confirmed':self._empty_hand_confirmed(),
                'connections': {
                    'robot': ('simulated' if run['mode'] == 'offline' else 'connected'
                              if getattr(run.get('controller'), 'mc', None) is not None
                              else 'unknown' if run.get('cleanup_error') else 'detected'
                              if self._devices.get('robot', {}).get('detected') is True else 'disconnected'),
                    'camera': ('simulated' if run['mode'] == 'offline' else 'connected'
                               if run.get('camera') is not None and run.get('camera_connected')
                                else 'unknown' if run.get('cleanup_error') else 'detected'
                                if self._devices.get('camera', {}).get('detected') is True
                                and self._devices.get('camera', {}).get('count') != 0 else 'disconnected')},
                'semantic_source': run['semantic_source'], 'pending': pending, 'sequence': sequence,
                'error': run.get('error'), 'parse': run.get('parse'), 'events': deepcopy(run['events']),
                'cleanup_error': run.get('cleanup_error'), 'outcome_verified': False,
                'manual_outcome': run.get('manual_outcome'), 'worker_busy': bool(self._thread and self._thread.is_alive()
                    and self._active == run_id), 'read_only': False})

    def _save(self, run):
        with self._lock:
            atomic_json(self._path(run['id']) / 'state.json', self.state(run['id']))

    def _event(self, run, stage, event, **detail):
        with self._lock:
            run['events'].append(redact({'time_ms': time.time() * 1000, 'stage': stage, 'event': event, **detail}))
            data = detail.get('data') or {}
            error = (detail.get('status') == 'error' or event.endswith(('_failed', '_timeout'))
                     or bool(detail.get('error')) or bool(data.get('error'))
                     or data.get('controller_error') not in (None, 0))
            if run['mode'] == 'real' and (event == 'object_released_at_place'
                    or event == 'gripper_commanded' and data.get('gripper_state') == 1):
                ctrl = run['controller']
                self._write_controller_state({'holding_uncertain':ctrl.gripper_holding_uncertain,
                    'holding':ctrl.gripper_holding_uncertain, 'at_start':ctrl.at_start,
                    'run_id':run['id'], 'source':'controller_gripper_transition'})
                return
            # These are process telemetry, not task/holding transitions. The
            # sequence persists step_started before dispatch, and holding writes
            # persist both files immediately before close dispatch and on release.
            # Unknown event types remain synchronous by default.
            transient = event in {
                'feedback_polled', 'feedback_sampled', 'movement_started', 'movement_completed',
                'standard_ik_current_seed_reused', 'preferred_ik_selected',
                'transfer_selected', 'transfer_completed', 'gripper_opened', 'gripper_closed',
                'start_verified_before_capture', 'initialization_open_reused',
                'pre_grasp_settled', 'empty_hand_returned_to_start',
            }
            if error or not transient:
                self._save(run)

    def _launch(self, run, work):
        if self._thread and self._thread.is_alive():
            raise ValueError('v2_worker_busy')
        def worker():
            try:
                work()
            except Exception as exc:
                if run['cancelled']:
                    run['status'] = 'cancelled'
                else:
                    run['error'] = redact({'type': type(exc).__name__, 'reason': str(exc)})
                    run['status'] = 'failed'
            finally:
                seq = run.get('sequence')
                if (run.get('status') == 'failed' or run.get('cancelled') or
                        seq and seq.snapshot()['status'] in {'completed', 'failed', 'cancelled'}):
                    self._cleanup(run)
                self._save(run)
        self._thread = threading.Thread(target=worker, daemon=True, name='visionpick-v2')
        self._thread.start()

    def create(self, payload):
        allowed = {'text', 'mode', 'semantic_source', 'destination_mode', 'default_station', 'move_to_start', 'source'}
        if not isinstance(payload, dict) or set(payload) - allowed:
            raise ValueError('invalid_v2_request_fields')
        text = payload.get('text')
        if not isinstance(text, str) or not text.strip() or len(text) > 4000:
            raise ValueError('instruction_required_max_4000')
        mode = payload.get('mode', 'offline')
        source = payload.get('semantic_source', 'demo')
        if mode not in {'offline', 'real'} or source not in {'demo', 'api'}:
            raise ValueError('invalid_v2_mode')
        if source == 'demo' and (text != DEMO_TEXT or mode != 'offline'):
            raise ValueError('demo_accepts_only_displayed_fixed_example')
        with self.service._lock, self._lock:
            if self._closing: raise ValueError('v2_runtime_shutting_down')
            if self._voice_busy(): raise ValueError('voice_busy')
            self._maintenance_idle()
            if self._active:
                old = self.state(self._active)
                if old['status'] not in {'completed', 'failed', 'cancelled', 'expired', 'blocked'} or self.owns_resources():
                    raise ValueError('v2_run_active_cancel_before_new_instruction')
            if mode == 'real': self._discover_devices()
            if mode == 'real': self._accept_observation_consent(payload)
            elif 'source' in payload or payload.get('move_to_start'):
                raise ValueError('empty_hand_confirmation_requires_real_mode')
            cfg = deepcopy(self.service._base_config())
            catalog = self._catalog()
            catalog_status = catalog.status()
            frozen = self.settings.freeze()
            run_id = uuid.uuid4().hex
            path = self._path(run_id)
            path.mkdir(parents=True, exist_ok=False)
            visible = self._visible_scene
            if visible is not None and (visible['mode'] != mode or visible['config_revision'] != _identity(cfg)
                    or not 0 <= time.time()*1000-visible['captured_at_unix_ms'] <= cfg.get('scene',{}).get('max_age_ms',30000)):
                visible = None
            run = {'id': run_id, 'mode': mode, 'semantic_source': source, 'status': 'parsing',
                'visible_objects': deepcopy(visible['objects']) if visible is not None else None,
                'visible_frame_id': visible['frame_id'] if visible is not None else None,
                'config': cfg, 'config_revision': _identity(cfg), 'catalog': catalog, 'arrival': frozen,
                'events': [], 'frames': {}, 'cancelled': False, 'text': text,
                'destination_mode': payload.get('destination_mode', 'strict'),
                'default_station': catalog_status['default_station'],
                'default_profile_id':catalog_status['default_profile_id']}
            if visible is not None and visible.get('observation') is not None:
                run['observation'] = visible['observation']
                run['preview'] = visible['observation'].display
            self._runs[run_id] = run
            self._active = run_id
            self._launch(run, lambda: self._parse(run))
            return self.state(run_id)

    def _parse(self, run):
        if run['mode'] == 'real':
            self._auto_observe(run)
        if run['cancelled']: return
        options = {'tools': json.loads((self.root / 'tools.json').read_text(encoding='utf-8')),
            'world_entities': json.loads((self.root / 'world_entities.json').read_text(encoding='utf-8')),
            'stations': run['catalog'].prompt_stations(), 'supported_relations': RELATIONS,
            'visible_objects': run.get('visible_objects')}
        if run['semantic_source'] == 'demo':
            candidate = {'tasks': [{'target': 'eraser', 'destination_type': 'relative',
                'destination_name': None, 'reference': 'pen', 'relation': 'right_of'}]}
            parsed = validate_task_plan_v2_candidate(candidate, **options)
        elif self.semantic_parser is not None:
            parsed = self.semantic_parser(run['text'], **options)
        else:
            from google import genai
            from google.genai import types
            from google.genai._gaos.utils.retries import RetryConfig
            import os
            key = os.getenv('GOOGLE_API_KEY') or os.getenv('GEMINI_API_KEY')
            if not key:
                raise ValueError('api_key_missing')
            client = genai.Client(vertexai=False, api_key=key,
                http_options=types.HttpOptions(base_url='https://generativelanguage.googleapis.com/',
                    timeout=60000, retry_options=types.HttpRetryOptions(attempts=1)))
            client.interactions.sdk_configuration.retry_config = RetryConfig('none', None, False, max_retries=0)
            try:
                parsed = parse_task_plan_v2(run['text'], client=client, **options)
            finally:
                client.close()
        run['parse'] = parsed.to_dict()
        atomic_json(self._path(run['id']) / 'semantic.json', redact(run['parse']))
        if run['cancelled']:
            run['status'] = 'cancelled'
            return
        if parsed.raw_payload is None:
            raise ValueError('semantic_candidate_missing')
        run['pending'] = PendingTaskPlanV2(parsed.raw_payload, original_text=run['text'],
            context_options=options, mode=run['destination_mode'], default_station=run['default_station'],
            settings={'arrival': run['arrival'].to_dict(), 'catalog_revision': run['catalog'].revision,
                'default_profile_id':run['default_profile_id'],
                'config_revision': run['config_revision'], 'initialization': 'each_task_before_capture'},
            instance_selection_available=True, timeout_s=self.service.confirmation_timeout,
            visual_perception_required=run['mode']=='real' and self._devices['camera'].get('detected') is True)
        if run['mode'] == 'offline' and self.scene_factory is None and run.get('observation') is None:
            # Preview-only RGB-D capture never initializes or dispatches a robot.
            observed = self._observe(run,1)
            run['preview'] = observed.display
        self._refresh_placement_previews(run)
        self._save(run)

    def action(self, run_id, action, payload):
        with self.service._lock, self._lock:
            if self._closing and action != 'cancel': raise ValueError('v2_runtime_shutting_down')
            run = self._runs.get(run_id)
            if run is None:
                raise ValueError('v2_run_not_resumable_after_restart')
            if run.get('cleanup_error') and action not in {'cancel','stop'}:
                raise ValueError('ownership_quarantined')
            if action in {'cancel', 'stop'}:
                run['cancelled'] = True
                if run.get('sequence'): run['sequence'].cancel()
                elif run.get('pending'): run['pending'].cancel()
                else: run['status'] = 'cancelled'
                if action == 'stop' and run.get('controller') is not None:
                    # Latch cancellation before attempting the transport stop.
                    try:
                        result = run['controller'].emergency_stop()
                    except Exception as exc:
                        result = {'sent': False, 'reason': str(exc)}
                    self._event(run, 'safety', 'software_stop_result', **result)
                if not self._thread or not self._thread.is_alive(): self._cleanup(run)
            elif action == 'answer':
                run['pending'].answer(**payload)
                self._refresh_placement_previews(run)
            elif action == 'placement_orientation':
                if self.owns_resources() or run.get('sequence'):
                    raise ValueError('placement_settings_locked_while_running')
                if set(payload) != {'version','step_id','policy','rz_deg'}:
                    raise ValueError('invalid_place_orientation_override')
                pending = run['pending'].snapshot()
                if pending['status'] != 'waiting_confirmation':
                    raise ValueError('not_waiting_for_confirmation')
                step = payload['step_id']
                tasks = pending['plan']['tasks']
                if type(step) is not int or not 1 <= step <= len(tasks):
                    raise ValueError('invalid_placement_step')
                from integration.relative_placement import resolve_place_orientation
                task = tasks[step-1]
                scenario = 'swap' if task['relation']=='at_initial' else task['destination_type']
                override = {'policy':payload['policy'],'rz_deg':payload['rz_deg']}
                resolve_place_orientation(run['config'],scenario,0.,reference_yaw=0.,initial_yaw=0.,profile_yaw=0.,override=override)
                settings = deepcopy(pending['settings'])
                settings.setdefault('placement_orientation_overrides',{})[str(step)] = override
                run['pending'].set_settings(settings,version=payload['version'])
                self._refresh_placement_previews(run)
            elif action == 'refresh_scene':
                if self._thread and self._thread.is_alive(): raise ValueError('v2_worker_busy')
                pending = run['pending'].snapshot()
                if (set(payload)-{'version','move_to_start','source'} or 'version' not in payload
                        or type(payload['version']) is not int or payload['version'] != pending['version']):
                    raise ValueError('stale_plan_version')
                if pending['status'] != 'waiting_answer': raise ValueError('not_waiting_for_answer')
                if run['mode'] == 'real': self._accept_observation_consent(payload)
                def refresh_scene():
                    try:
                        if run['mode'] == 'real': self._observe_for_semantics(run, allow_start=payload.get('move_to_start',False))
                        else:
                            observed = self._observe(run,1,semantic_only=True)
                            run['visible_objects'] = self._visible_scene['objects']
                            run['visible_frame_id'] = observed.frame_id
                            run['preview'] = observed.display
                        if run['cancelled']: return
                        run['pending'].update_visible_objects(run['visible_objects'], visual_perception_required=run['mode']=='real')
                        self._refresh_placement_previews(run)
                        run['error'] = None
                    except Exception as exc:
                        if run.get('cleanup_error'): raise
                        run['error'] = redact({'type':type(exc).__name__,'reason':str(exc)})
                        self._event(run,'perception','semantic_scene_refresh_failed',reason=str(exc))
                self._launch(run,refresh_scene)
            elif action == 'confirm':
                self._maintenance_idle()
                if (run['config_revision'] != _identity(self.service._base_config())
                        or run['arrival'].revision != self.settings.freeze().revision
                        or run['catalog'].revision != self._catalog().revision):
                    raise ValueError('configuration_changed_create_new_plan')
                if self._thread and self._thread.is_alive(): raise ValueError('v2_worker_busy')
                self._preflight(run)
                run['pending'].confirm(**payload)
                from integration.relative_placement import evaluate_local_result
                seq = TaskSequenceV2(run['pending'], run_id=run_id, initialization_policy='each_task',
                    observe=lambda step: self._observe(run, step), prepare=lambda *a: self._prepare(run, *a),
                    execute=lambda *a, **k: self._execute(run, *a, **k),
                    step_recovery_enabled=True,
                    persist_boundary=lambda: self._save(run),
                    initialize_before_observe=lambda step, **k: self._initialize(run, step, **k),
                    check_result=lambda expected, observed: evaluate_local_result(expected, observed.instances),
                    observe_completion=lambda **options: self._capture_completion(run, run['sequence'].snapshot(), **options),
                    max_scene_age_ms=run['config'].get('scene', {}).get('max_age_ms', 30000))
                run['sequence'] = seq
                seq.begin(version=payload['version'])
                self._launch(run, lambda: self._advance(run))
            elif action in {'retry_step','skip_step'}:
                if self._thread and self._thread.is_alive(): raise ValueError('v2_worker_busy')
                if set(payload) != {'run_id','step_id','frame_id','plan_version'}:
                    raise ValueError('invalid_step_recovery_fields')
                if run.get('sequence') is None: raise ValueError('not_waiting_for_step_recovery')
                if run['mode'] == 'real':
                    self._require_devices()
                    if self._holding(): raise ValueError('gripper_holding_uncertain_requires_manual_recovery')
                ctrl = run.get('controller')
                if ctrl is None or ctrl.gripper_holding_uncertain:
                    raise ValueError('gripper_holding_uncertain_requires_manual_recovery')
                run['sequence'].recover_step(action,**payload)
                self._launch(run, lambda: self._advance(run))
            elif action in {'select', 'refresh'}:
                if self._thread and self._thread.is_alive(): raise ValueError('v2_worker_busy')
                if action == 'select': run['sequence'].select(**payload)
                else:
                    if payload: raise ValueError('refresh_requires_empty_payload')
                    run['sequence'].refresh()
                self._launch(run, lambda: self._advance(run))
            elif action == 'outcome':
                if self.state(run_id)['status'] != 'completed' or payload.get('outcome') not in {'success', 'failure', 'unverified'}:
                    raise ValueError('completed_run_and_manual_outcome_required')
                run['manual_outcome'] = {'outcome': payload['outcome'], 'source': 'operator', 'time_ms': time.time()*1000}
            else: raise ValueError('unknown_v2_action')
            self._save(run)
            return self.state(run_id)

    def _preflight(self, run, *, semantic_only=False):
        if run['mode'] == 'offline': return
        self._require_devices()
        if not self.service.hardware_enabled or not self.service.motion_enabled:
            raise ValueError('real_motion_not_enabled_for_this_server')
        if run.get('pending') and not semantic_only:
            pending = run['pending'].require_visual_perception()
            if any(i['code']=='unsupported_visual_reference' for i in pending['issues']):
                raise ValueError('unsupported_visual_reference')
        if self._holding(): raise ValueError('gripper_holding_uncertain_requires_manual_recovery')
        from v0_mvp.motion_safety import validate_motion_unlock
        validate_motion_unlock(run['config'])  # Calibration/RMSE; no retired IK/support policy.
        if run['config']['robot'].get('move_to_start_before_pick', True) is not True:
            raise ValueError('v2_requires_start_before_capture')
        ctrl = DirectRobotControllerV2(run['config'], settings=run['arrival'].settings)
        angles = _six(run['config']['robot'].get('start_angles_deg'), 'start_angles')
        if any(abs(a) > 360 for a in angles): raise ValueError('invalid_start_joint_range')
        ctrl._speed('start_angle_speed')
        self.service.table_surface.require(run['config'])
        pending = run['pending'].snapshot() if run.get('pending') else {}
        for task in (pending.get('plan') or {}).get('tasks', []):
            if task['destination_type'] == 'named': run['catalog'].direct_place(task['destination_name'])

    def _observe_for_semantics(self, run, *, allow_start=False):
        def checkpoint():
            if run['cancelled']: raise ValueError('operator_cancelled')
        try:
            checkpoint()
            self._visible_scene = None
            devices = self._discover_devices()
            if (devices['camera'].get('detected') is not True or devices['camera'].get('count') == 0
                    or not self.service.hardware_enabled):
                raise ValueError('hardware_not_detected')
            with_start = bool(allow_start and devices['robot'].get('detected') is True
                              and self.service.motion_enabled and not self._holding())
            if devices['robot'].get('detected') is True and self.service.motion_enabled:
                if not self._empty_hand_confirmed():
                    raise ValueError('empty_gripper_confirmation_required')
                with_start = True
            if with_start:
                self._preflight(run, semantic_only=True)
                result = self._initialize(run, 1, checkpoint=checkpoint)
                if result.get('start_verified') is not True: raise ValueError('start_not_verified_before_observation')
            checkpoint()
            observed = self._observe(run, 1, semantic_only=True)
            checkpoint()
            age = time.time()*1000-observed.captured_at_unix_ms
            if not math.isfinite(age) or not 0 <= age <= run['config'].get('scene', {}).get('max_age_ms', 30000):
                raise ValueError('scene_snapshot_stale')
            run['visible_objects'] = list(dict.fromkeys(i['class_name'] for i in observed.instances))
            run['visible_frame_id'] = observed.frame_id
            run['preview'] = observed.display
            self._event(run, 'perception', 'semantic_scene_observed', start_verified=with_start,
                        visible_objects=run['visible_objects'], frame_id=observed.frame_id)
            return observed
        finally:
            self._cleanup(run)
            if run.get('cleanup_error'): raise ValueError('ownership_quarantined')

    def _auto_observe(self, run):
        devices = self._discover_devices()
        if (not self.service.hardware_enabled or devices['camera'].get('detected') is not True
                or devices['camera'].get('count') == 0):
            run['visible_objects'] = run['visible_frame_id'] = None
            return
        visible = self._visible_scene
        arm_online = devices['robot'].get('detected') is True
        if (visible is not None and visible['mode'] == run['mode']
                and visible['config_revision'] == run['config_revision']
                and (not arm_online or (visible.get('captured_at_start') is True
                     and visible.get('arm_motion_epoch') == self._arm_motion_epoch
                     and visible.get('arm_state_marker') == self._arm_state_marker()
                     and self._empty_hand_confirmed()))
                and 0 <= time.time()*1000-visible['captured_at_unix_ms'] <= run['config'].get('scene', {}).get('max_age_ms',30000)):
            run['visible_objects'] = deepcopy(visible['objects'])
            run['visible_frame_id'] = visible['frame_id']
            if visible.get('observation') is not None:
                run['observation'] = visible['observation']
                run['preview'] = visible['observation'].display
            return
        self._observe_for_semantics(run, allow_start=self._empty_hand_confirmed())

    def _initialize(self, run, step, *, checkpoint):
        self._visible_scene = None
        self._arm_motion_epoch += 1
        self.engineering.last_pose = None
        self._calibration_frame = None
        run['start_verified'] = False
        checkpoint()
        if run['mode'] == 'real':
            if run.get('operation') == 'home':
                if self._discover_devices().get('robot', {}).get('detected') is not True:
                    raise ValueError('robot_not_detected')
            else:
                self._require_devices()
        if run.get('controller') is None:
            if self.controller_factory:
                run['controller'] = self.controller_factory(run['config'], run['arrival'].settings, run['mode'],
                    lambda *a, **k: self._event(run, *a, **k), checkpoint)
            elif run['mode'] == 'offline':
                from .simulation import FakeMyCobot
                cfg = self._demo_case()['config'] | {'robot': deepcopy(run['config']['robot'])}
                cfg['robot'].update(start_pose_mm_deg=[0,0,150,-180,0,0], fallback_wait_s=0)
                backend = FakeMyCobot()
                backend.send_angles = lambda angles, speed: backend.sync_send_angles(angles, speed)
                run['controller'] = DirectRobotControllerV2(cfg, backend=backend, settings=run['arrival'].settings,
                    event_sink=lambda *a, **k: self._event(run, *a, **k), checkpoint=checkpoint)
            else:
                run['lease'] = RobotLease(run['config']['robot']['port']).acquire()
                run['controller'] = DirectRobotControllerV2(run['config'], settings=run['arrival'].settings,
                    event_sink=lambda *a, **k: self._event(run, *a, **k), checkpoint=checkpoint)
                run['controller'].connect()
            run['controller'].gripper_holding_uncertain = self._holding() if run['mode'] == 'real' else False
            evidence = run.pop('open_empty_evidence', None)
            if (run['mode'] == 'real' and evidence is not None
                    and evidence['config_revision'] == run['config_revision']
                    and evidence['arm_state_marker'] == self._arm_state_marker()
                    and self._empty_hand_confirmed() and not run['controller'].gripper_holding_uncertain):
                # Same run only; never restore open state from a persisted file.
                run['controller']._gripper_open_confirmed = True
        run['controller'].checkpoint = checkpoint
        result = run['controller'].initialize_for_pick()
        run['start_verified'] = result.get('start_verified') is True
        if run['mode'] == 'real' and run['start_verified']:
            self._write_controller_state({'holding_uncertain':False,'holding':False,'at_start':True,
                                          'source':'controller_start','run_id':run['id']})
        return {**result,'empty_hand_confirmed':run['controller'].gripper_holding_uncertain is False}

    def _demo_case(self):
        return json.loads((self.root / 'tests' / 'data' / 'relative_place_s3_demo.json').read_text(encoding='utf-8'))

    def _observe(self, run, step, *, semantic_only=False):
        import cv2
        import numpy as np
        cfg = run['config']
        captured = time.time() * 1000
        if self.scene_factory:
            detections, frame, deproject, project, geometry_cfg, surface = self.scene_factory(step, cfg, run['mode'])
        elif run['mode'] == 'offline':
            from .simulation import synthetic_scene
            case = run.setdefault('offline_scene', self._demo_case())
            detections, frame, deproject, project = synthetic_scene(case)
            geometry_cfg, surface = case['config'], case['surface']
        else:
            from v0_mvp.camera import RealSenseCamera
            from v0_mvp.detector import YOLO11OBBDetector
            if run.get('camera') is None:
                c = cfg['camera']; m = cfg['model']
                run['camera'] = RealSenseCamera(c.get('device_id'), c.get('width',640), c.get('height',480),
                    c.get('fps',30), c.get('frame_timeout_ms',5000))
                run['camera'].connect()
                run['camera_connected'] = True
                weights = Path(m['weights'])
                run['detector'] = YOLO11OBBDetector(weights if weights.is_absolute() else self.root/weights,
                    imgsz=m.get('imgsz',640), confidence=m.get('confidence',.35), iou=m.get('iou',.5), device=m.get('device'))
            # Flush buffered frames AFTER verified Start, including every later step.
            run['camera'].warmup(cfg['camera'].get('warmup_frames',15))
            captured = time.time() * 1000
            frame = run['camera'].capture()
            detections, _ = run['detector'].detect(frame.color_bgr)
            intr = frame.intrinsics
            # Keep frame math usable after semantic capture releases its camera.
            camera_deproject = run['camera'].deproject
            deproject = lambda u,v,z: camera_deproject(u,v,z,intr)
            project = lambda p: (p[0]*intr.fx/p[2]+intr.ppx, p[1]*intr.fy/p[2]+intr.ppy)
            table = {}
            if not semantic_only or cfg.get('calibration', {}).get('T_robot_from_camera'):
                try:
                    table = self.service.table_surface.require(cfg)
                except (ValueError, RuntimeError):
                    if not semantic_only: raise
            scene = cfg.get('scene', {})
            surface = {} if not table else {'static_scene':True, 'z_mm':table['z_mm'], 'bounds_mm':{k:cfg['grasp']['workspace_mm'][k] for k in ('x','y')},
                'error_margin_mm':scene.get('error_margin_mm',5), 'gripper_padding_mm':scene.get('gripper_padding_mm',10),
                'surface_tolerance_mm':scene.get('surface_tolerance_mm',25), 'release_tcp_z_offset_mm':scene.get('release_tcp_z_offset_mm',20),
                'max_age_ms':scene.get('max_age_ms',30000)}
            geometry_cfg = cfg
        frame_id = uuid.uuid4().hex
        self._calibration_frame = {'frame': frame, 'synthetic': run['mode'] != 'real',
            'at': time.monotonic(), 'config_revision': _identity(cfg)}
        frame_timestamp = getattr(frame, 'captured_at_unix_ms', None)
        if frame_timestamp is not None:
            captured = frame_timestamp  # Never make a stale/invalid supplied timestamp look fresh.
        from integration.relative_placement import _footprint, footprint_yaw_deg
        from v0_mvp.geometry import as_transform
        from v0_mvp.grasp import build_top_down_plan
        from object_catalog import graspable_classes
        transform = None
        if not semantic_only or (surface and geometry_cfg.get('calibration', {}).get('T_robot_from_camera')):
            try:
                transform = as_transform(geometry_cfg['calibration']['T_robot_from_camera'])
            except (ValueError, RuntimeError):
                if not semantic_only: raise
        instances = []
        for i, det in enumerate(detections):
            eligible, reason = True, ''
            reference_eligible = False
            point = None
            polygon = None
            try:
                if transform is None:
                    raise ValueError('semantic_only_not_geometry_validated')
                polygon, depth = _footprint(det, frame, deproject, transform, geometry_cfg, surface['surface_tolerance_mm'])
                point = transform[:3,:3] @ np.asarray(deproject(*det.center_uv,depth)) + transform[:3,3]
                if not np.isfinite(point).all():
                    raise ValueError('unreliable_object_depth')
                reference_eligible = True
                workspace = geometry_cfg['grasp']['workspace_mm']
                if any(not workspace[k][0] <= point[n]*1000 <= workspace[k][1] for n,k in enumerate(('x','y','z'))):
                    raise ValueError('instance_outside_workspace')
                if det.class_name in graspable_classes():
                    build_top_down_plan(point, 0, geometry_cfg['grasp'],
                        canonical_target=det.class_name, table_z_mm=surface.get('z_mm'))
            except (ValueError, RuntimeError) as exc: eligible, reason = False, str(exc)
            instances.append({'instance_id':f'{frame_id}:{i}', 'class_name':det.class_name, 'eligible':eligible,
                'reference_eligible':reference_eligible, 'reference_rejection_reason':'' if reference_eligible else reason,
                'rejection_reason':reason, 'index':i, 'corners_px':np.asarray(det.corners_px).tolist(),
                'confidence':float(det.confidence),
                'footprint_robot_xy_mm': polygon.tolist() if polygon is not None else None,
                'xyz_mm':(point*1000).tolist() if point is not None and np.isfinite(point).all() else None,
                'top_z_mm':float(point[2]*1000) if point is not None and np.isfinite(point).all() else None,
                'yaw_deg':footprint_yaw_deg(polygon,geometry_cfg) if polygon is not None else None})
        path = self._path(run['id'])
        ok, image = cv2.imencode('.png', frame.color_bgr)
        if not ok: raise ValueError('frame_encode_failed')
        (path / f'{frame_id}.png').write_bytes(image.tobytes())
        np.save(path/f'{frame_id}.depth.npy', frame.depth_m, allow_pickle=False)
        context = {'detections':detections,'frame':frame,'deproject':deproject,'project':project,
            'config':geometry_cfg,'surface':surface,'semantic_only':semantic_only}
        run['frames'][frame_id] = context
        atomic_json(path/f'{frame_id}.json', {'captured_at_unix_ms':captured, 'instances':instances,
            'intrinsics':{k:float(getattr(frame.intrinsics,k)) for k in ('fx','fy','ppx','ppy')},
            'geometry_config':geometry_cfg, 'surface':surface, 'synthetic':run['mode']=='offline'})
        h,w = frame.color_bgr.shape[:2]
        with self._lock:
            self._visible_scene = {'objects': list(dict.fromkeys(d.class_name for d in detections)),
                'frame_id': frame_id, 'captured_at_unix_ms': captured, 'mode': run['mode'],
                'captured_at_start':bool(run.get('start_verified') and getattr(run.get('controller'),'at_start',False)),
                'arm_motion_epoch':self._arm_motion_epoch, 'arm_state_marker':self._arm_state_marker(),
                'config_revision': _identity(cfg)}
        observed = ObservationV2(frame_id,captured,tuple(instances),payload=context,
            display={'frame_id':frame_id,'image_url':f'/api/v2/runs/{run["id"]}/frames/{frame_id}',
                'width':w,'height':h,'rotation_deg':cfg.get('scene',{}).get('workspace_view_rotation_deg',90),
                'synthetic':run['mode']=='offline', 'instances':instances})
        run['observation'] = observed
        run['preview'] = observed.display
        with self._lock:
            self._visible_scene['observation'] = observed
        self._refresh_placement_previews(run)
        return observed

    def preview(self, payload):
        """A separately recorded Start -> capture operation, with no LLM or pick."""
        if (not isinstance(payload, dict) or 'mode' not in payload or set(payload)-{'mode','move_to_start','source'}
                or payload['mode'] not in {'offline','real'} or type(payload.get('move_to_start',False)) is not bool):
            raise ValueError('invalid_preview_request')
        with self.service._lock, self._lock:
            if self._closing: raise ValueError('v2_runtime_shutting_down')
            self._maintenance_idle()
            if self.owns_resources(): raise ValueError('v2_worker_busy')
            if self._active and self.state(self._active)['status'] not in {'completed','failed','cancelled','expired','blocked'}:
                raise ValueError('v2_run_active_cancel_before_new_instruction')
            mode = payload['mode']
            if mode == 'real':
                if not self.service.hardware_enabled:
                    raise ValueError('real_motion_not_enabled_for_this_server')
                devices = self._discover_devices()
                if devices['camera'].get('detected') is not True or devices['camera'].get('count') == 0:
                    raise ValueError('hardware_not_detected')
                self._accept_observation_consent(payload)
            run_id = uuid.uuid4().hex
            self._path(run_id).mkdir(parents=True, exist_ok=False)
            cfg = deepcopy(self.service._base_config())
            run = {'id':run_id, 'mode':mode, 'operation':'preview', 'semantic_source':'none',
                   'status':'running', 'config':cfg, 'config_revision':_identity(cfg),
                   'arrival':self.settings.freeze(), 'events':[], 'frames':{}, 'cancelled':False}
            self._runs[run_id] = run
            self._active = run_id
            def capture():
                def checkpoint():
                    if run['cancelled']: raise ValueError('operator_cancelled')
                try:
                    if mode == 'real':
                        self._observe_for_semantics(run, allow_start=payload.get('move_to_start',False))
                        run['status'] = 'completed'
                        return
                    self._initialize(run, 1, checkpoint=checkpoint)
                    checkpoint()
                    observed = self._observe(run, 1)
                    checkpoint()
                    run['preview'] = observed.display
                    run['status'] = 'completed'
                finally:
                    self._cleanup(run)
                    if run['cancelled']: run['status'] = 'cancelled'
            self._launch(run, capture)
            return self.state(run_id)

    def _placement_context(self, run, observation):
        cfg = deepcopy(run['config'] if run['mode']=='real' else observation.payload['config'])
        orientation = cfg.setdefault('orientation',{})
        for key,value in run['config'].get('orientation',{}).items():
            if key.startswith('place_orientation_policy_') or key == 'fixed_place_yaw_deg':
                orientation[key] = value
        if 'sequence_gap_mm' in run['config'].get('scene',{}):
            cfg.setdefault('scene',{})['sequence_gap_mm'] = run['config']['scene']['sequence_gap_mm']
        surface = deepcopy(observation.payload['surface'])
        if not surface:
            table = self.service.table_surface.require(run['config'])
            scene = run['config'].get('scene',{})
            surface = {'static_scene':True,'z_mm':table['z_mm'],
                'bounds_mm':{k:cfg['grasp']['workspace_mm'][k] for k in ('x','y')},
                'error_margin_mm':scene.get('error_margin_mm',5.),
                'gripper_padding_mm':scene.get('gripper_padding_mm',10.),
                'surface_tolerance_mm':scene.get('surface_tolerance_mm',25.),
                'release_tcp_z_offset_mm':scene.get('release_tcp_z_offset_mm',20.),
                'max_age_ms':scene.get('max_age_ms',30000.)}
        return cfg,surface

    def refresh_waiting_placement_settings(self):
        """Engineering save refreshes an unclaimed plan and invalidates old consent."""
        for run in self._runs.values():
            pending = run.get('pending')
            if pending is None or run.get('sequence') is not None:
                continue
            snap = pending.snapshot()
            if snap['status'] not in {'waiting_confirmation','waiting_answer'}:
                continue
            cfg = deepcopy(self.service._base_config())
            run.update(config=cfg,config_revision=_identity(cfg),catalog=self._catalog(),arrival=self.settings.freeze())
            settings = {**snap['settings'],'config_revision':run['config_revision'],
                'catalog_revision':run['catalog'].revision,'arrival':run['arrival'].to_dict()}
            pending.set_settings(settings,version=snap['version'])
            self._refresh_placement_previews(run)

    def _refresh_placement_previews(self, run, *, selected=None):
        """Read-only model overlay; no solver, controller, capture, or artifact writes.

        Future steps use predicted planar poses, while retaining the measured
        depth image. A depth warning is never suppressed to make a forecast fit.
        """
        import numpy as np
        from dataclasses import replace
        from integration.relative_placement import (_footprint,footprint_yaw_deg,
            placement_preview_geometry,project_robot_points_mm)
        from v0_mvp.geometry import as_transform
        observation = run.get('observation')
        pending = run.get('pending')
        if observation is None or pending is None:
            run['placement_previews'] = []
            return
        snap = pending.snapshot()
        seq = run['sequence'].snapshot() if run.get('sequence') else None
        if (seq and seq['status'] not in {'running','executing','waiting_instance'}) or (not seq and snap['status'] != 'waiting_confirmation'):
            run['placement_previews'] = []
            return
        tasks = (snap.get('plan') or {}).get('tasks',[])
        overrides = snap['settings'].get('placement_orientation_overrides',{})
        result = []
        start = (seq.get('step_id') or 1)-1 if seq else 0
        if observation.payload.get('semantic_only'):
            try:
                as_transform(observation.payload.get('config',{}).get('calibration',{}).get('T_robot_from_camera'))
                self._placement_context(run,observation)
            except (ValueError,RuntimeError,KeyError,TypeError) as exc:
                run['placement_previews'] = [{'step_id':i+1,'target':task['target'],
                    'frame_id':observation.frame_id,'feasible':False,
                    'reason':'semantic_only_not_geometry_validated','geometry_reason':str(exc),'authorizes_motion':False,
                    'requires_fresh_observation':True} for i,task in enumerate(tasks) if i>=start]
                return
        try:
            cfg,surface = self._placement_context(run,observation)
            ctx = observation.payload
            transform = as_transform(cfg['calibration']['T_robot_from_camera'])
            detections = list(ctx['detections'])
            measured = [_footprint(d,ctx['frame'],ctx['deproject'],transform,cfg,surface['surface_tolerance_mm']) for d in detections]
            points = [(transform[:3,:3] @ np.asarray(ctx['deproject'](*d.center_uv,depth))+transform[:3,3])*1000
                for d,(_,depth) in zip(detections,measured)]
            initial = seq.get('initial_pose_snapshot',{}).get('instances') if seq else None
            if not initial:
                initial = [{**item,'footprint_robot_xy_mm':measured[i][0].tolist(),
                    'xyz_mm':points[i].tolist(),'yaw_deg':footprint_yaw_deg(measured[i][0],cfg)}
                    for i,item in enumerate(observation.instances)]
            failed_dependency = False
            for index,task in enumerate(tasks):
                if index < start: continue
                record = {'step_id':index+1,'target':task['target'],'frame_id':observation.frame_id,
                    'feasible':False,'reason':None,'authorizes_motion':False,
                    'predicted_scene':index>start,'requires_fresh_observation':True}
                # A later swap step has no meaningful landing position until
                # the preceding step produced a feasible temporary placement.
                if failed_dependency:
                    record['reason'] = 'previous_step_geometry_failed'
                    result.append(record)
                    continue
                choices = dict(selected or {}) if index==start else {}
                geometry = {}
                try:
                    for role in ('target','reference'):
                        if role=='reference' and task['destination_type']!='relative':continue
                        if role=='reference' and task['relation']=='at_initial':
                            anchors = [i for i in initial if i['class_name']==task['reference']]
                            if len(anchors)!=1 or not anchors[0]['eligible']:raise ValueError('initial_reference_not_unique_or_missing')
                            choices['initial_reference'] = anchors[0]
                            continue
                        if role in choices:continue
                        matches = [i for i in observation.instances if i['class_name']==task[role]]
                        if len(matches)!=1:raise ValueError('ambiguous_object_instances' if matches else 'object_not_detected')
                        # Forecast source positions can differ from this frame's
                        # positions. Validate each step's actual/modelled source,
                        # while a reference only requires measured RGB-D geometry.
                        if role=='target':
                            from v0_mvp.grasp import build_top_down_plan
                            point = points[matches[0]['index']]
                            if any(not cfg['grasp']['workspace_mm'][k][0] <= point[n] <= cfg['grasp']['workspace_mm'][k][1] for n,k in enumerate(('x','y','z'))):
                                raise ValueError('no_eligible_target_instance')
                            build_top_down_plan(point/1000.,0,cfg['grasp'],canonical_target=task['target'],table_z_mm=surface.get('z_mm'))
                        choices[role] = matches[0]['instance_id']
                    ti = next(i for i,item in enumerate(observation.instances) if item['instance_id']==choices['target'])
                    if task['destination_type']=='self_relative':
                        choices['self_anchor'] = {'frame_id':observation.frame_id,'instance_id':choices['target']}
                    # First step always uses measured geometry. Later steps use
                    # the previous modelled footprints, never to authorize motion.
                    virtual_ctx = {**ctx,'detections':detections}
                    if index>start:
                        virtual_ctx['preview_scene'] = {'measurements':measured,'points_mm':points}
                    virtual = replace(observation,payload=virtual_ctx)
                    prepared = self._plan_placement(run,task,virtual,choices,
                        orientation_override=overrides.get(str(index+1)),step_id=index+1)
                    geometry = prepared['geometry']
                    record['feasible'] = True
                except (ValueError,RuntimeError) as exc:
                    record['reason'] = str(exc)
                    geometry = getattr(exc,'details',{})
                # Failed candidates are diagnostic only. Never draw them as
                # planned placements or advance the virtual object position.
                if record['feasible']:
                    record.update(placement_preview_geometry(geometry,cfg,ctx['frame'].intrinsics))
                    if record.get('projection_reason'):
                        record.update(feasible=False,reason=record['projection_reason'])
                if record['feasible'] and 'center_mm' in record:
                    record['source_center_px'] = project_robot_points_mm([points[ti]],cfg,ctx['frame'].intrinsics)[0]
                    old_yaw = footprint_yaw_deg(measured[ti][0],cfg)
                    delta = math.radians(record['rz_deg']-old_yaw)
                    # Project a model of this object at its forecast top height.
                    poly = np.asarray(record['polygon_mm'])[:,:2]
                    old_z = points[ti][2]
                    points[ti] = np.r_[np.asarray(record['center_mm'])[:2],old_z]
                    measured[ti] = (poly,measured[ti][1])
                    detections[ti] = replace(detections[ti],long_axis_angle_deg=detections[ti].long_axis_angle_deg+math.degrees(delta))
                failed_dependency = not record['feasible']
                result.append(record)
        except (ValueError,RuntimeError) as exc:
            result = [{'step_id':i+1,'target':t['target'],'frame_id':observation.frame_id,
                'feasible':False,'reason':str(exc),'authorizes_motion':False} for i,t in enumerate(tasks) if i>=start]
        run['placement_previews'] = result

    def _swap_reserved_regions(self, run, task, observation, cfg, surface, *, step_id=None, orientation_override=None):
        """Reserve original sites and both future landing/gripper areas for a swap.

        This only filters temporary candidates; every step still needs fresh
        geometry. It never reserves the moving target's old site for ordinary near.
        """
        import numpy as np
        from integration.relative_placement import (_footprint, footprint_yaw_deg,
            clearance_envelope_obb, resolve_place_orientation)
        from v0_mvp.geometry import as_transform
        if task.get('destination_type')!='relative' or task.get('relation')!='near':
            return []
        snap = run['pending'].snapshot()
        tasks = (snap.get('plan') or {}).get('tasks',[])
        seq = run['sequence'].snapshot() if run.get('sequence') else None
        index = (step_id or (seq.get('step_id') if seq else 1) or 1)-1
        triple = tasks[index:index+3]
        a,b = task.get('target'),task.get('reference')
        if (task.get('destination_type')!='relative' or task.get('relation')!='near'
                or a==b or len(triple)!=3
                or any(t.get('destination_type')!='relative' for t in triple)
                or [(t['target'],t['reference'],t['relation']) for t in triple]
                   != [(a,b,'near'),(b,a,'at_initial'),(a,b,'at_initial')]):
            return []
        initial = seq.get('initial_pose_snapshot',{}).get('instances') if seq else None
        initial = initial or observation.instances
        anchors = {}
        for name in (a,b):
            matches = [i for i in initial if i['class_name']==name]
            if len(matches)!=1 or not matches[0]['eligible']:
                raise ValueError('initial_reference_not_unique_or_missing')
            anchors[name] = matches[0]
        polygons = {name:np.asarray(item['footprint_robot_xy_mm'],dtype=float) for name,item in anchors.items()}
        # Snapshot both ORIGINAL orientations before A is moved. These angles
        # are subsequently used for B -> original A and A -> original B.
        yaws = {name:float(item['yaw_deg']) if item.get('yaw_deg') is not None
            else footprint_yaw_deg(polygons[name],cfg) for name,item in anchors.items()}
        temp_a_deg, temp_b_deg = yaws[a], yaws[b]
        ctx = observation.payload
        transform = as_transform(cfg['calibration']['T_robot_from_camera'])
        current = {}
        for name in (a,b):
            instance = next(i for i in observation.instances if i['class_name']==name)
            i = instance['index']
            poly = (ctx['preview_scene']['measurements'][i][0] if ctx.get('preview_scene') is not None else
                _footprint(ctx['detections'][i],ctx['frame'],ctx['deproject'],transform,cfg,surface['surface_tolerance_mm'])[0])
            current[name] = footprint_yaw_deg(poly,cfg)
        policy_cfg = cfg
        if 'place_orientation_policy_relative' not in cfg.get('orientation',{}):
            policy_cfg = {**cfg,'orientation':{**cfg.get('orientation',{}),
                'place_orientation_policy_relative':'align_reference' if surface.get('align_to_reference_yaw',False) else 'keep_target'}}
        _,temporary_yaw = resolve_place_orientation(policy_cfg,'relative',current[a],reference_yaw=current[b],override=orientation_override)
        overrides = snap['settings'].get('placement_orientation_overrides',{})
        # Reserve FULL original gripper envelopes, not just object footprints.
        # A temporary landing may not touch either original A/B envelope.
        regions = [{'label':name+'_initial',
            'polygon_mm':clearance_envelope_obb(poly,surface['gripper_padding_mm'],
                surface['error_margin_mm'],cfg['grasp'].get('tcp_xy_offset_mm',[0,0])).tolist(),
            'check_envelope':True} for name,poly in polygons.items()]
        for offset,(target,reference) in enumerate(((b,a),(a,b)),start=1):
            override = (seq['steps'][index+offset].get('orientation_override') if seq else overrides.get(str(index+offset+1)))
            _,yaw = resolve_place_orientation(cfg,'swap',temporary_yaw if target==a else current[b],
                initial_yaw=temp_a_deg if reference==a else temp_b_deg,override=override)
            angle = math.radians(yaw-(temp_a_deg if target==a else temp_b_deg));c,s = math.cos(angle),math.sin(angle)
            poly = polygons[target]
            center = np.asarray(anchors[reference]['xyz_mm'][:2],dtype=float)
            landing = (poly-poly.mean(axis=0)) @ np.array([[c,-s],[s,c]]).T + center
            envelope = clearance_envelope_obb(landing,surface['gripper_padding_mm'],
                surface['error_margin_mm'],cfg['grasp'].get('tcp_xy_offset_mm',[0,0]))
            regions.extend([
                {'label':target+'_future_landing','polygon_mm':landing.tolist(),'check_envelope':True},
                {'label':target+'_future_gripper','polygon_mm':envelope.tolist(),'check_envelope':False}])
        return regions

    def _plan_placement(self, run, task, observation, selected, *, orientation_override=None, step_id=None):
        import cv2
        import numpy as np
        from integration.relative_placement import (_footprint, plan_relative_place,
            clearance_envelope_obb, convex_polygons_intersect, _surface_clear)
        from v0_mvp.geometry import as_transform, camera_axis_to_robot_yaw_deg
        from v0_mvp.grasp import build_top_down_plan
        from v0_mvp.types import GraspPlan
        ctx = observation.payload
        cfg, surface = self._placement_context(run,observation)
        detections = ctx['detections']
        ids = {i['instance_id']:i['index'] for i in observation.instances}
        ti = ids[selected['target']]
        if task['destination_type'] in {'relative','self_relative'}:
            is_self = task['destination_type'] == 'self_relative'
            if is_self:
                if _direction_frame(cfg) != _direction_frame(run['config']):
                    raise ValueError('self_direction_frame_mismatch')
                anchor = selected.get('self_anchor', {})
                if anchor.get('frame_id') != observation.frame_id or anchor.get('instance_id') != selected['target']:
                    raise ValueError('self_anchor_snapshot_mismatch')
            legacy = {'intent':'place','target_object':task['target'],'reference_object':task['reference'],
                'spatial_relation':task['relation'],'needs_clarification':False,'destination_type':task['destination_type']}
            scene = cfg.get('scene', {})
            if run['mode'] == 'real':
                surface.update(require_verified_support_models=False, geometry_policy='pure_rgbd',
                    use_local_surface_z=task['relation']!='on_top_of',
                    align_to_reference_yaw=True,use_safe_transfer_z=True)
            gap = task.get('gap_mm')
            if not is_self and gap is None and task['relation'] not in {'on_top_of','at_initial'}:
                gap = scene.get('sequence_gap_mm',50.0)
            geometry = plan_relative_place(legacy,detections,ctx['frame'],deproject=ctx['deproject'],project=ctx['project'],
                config=cfg,surface=surface,gap_mm=gap,captured_at_unix_ms=observation.captured_at_unix_ms,
                selected_target_index=ti,selected_reference_index=ids.get(selected.get('reference')),
                initial_reference=selected.get('initial_reference'),
                self_displacement_mm=task.get('displacement_mm'),
                orientation_override=orientation_override,
                exact_gap=task.get('gap_mm') is not None,preview_scene=ctx.get('preview_scene'),
                locked_destination=selected.get('locked_destination'),
                reserved_regions=self._swap_reserved_regions(run,task,observation,cfg,surface,
                    step_id=step_id,orientation_override=orientation_override))
            grasp = GraspPlan(**{k:tuple(v) for k,v in geometry['source_grasp_plan'].items()})
            poses = geometry['candidate_tcp_poses_mm_deg']
            place = SimpleNamespace(planner_mode='hybrid',pre_place_pose_mm_deg=poses['pre_place'],
                place_pose_mm_deg=poses['place'],retreat_pose_mm_deg=poses['retreat'],transfer_mode=0,place_mode=1,retreat_mode=1)
        else:
            place = run['catalog'].direct_place(task['destination_name'])
            transform = as_transform(cfg['calibration']['T_robot_from_camera'])
            measured = [_footprint(d,ctx['frame'],ctx['deproject'],transform,cfg,surface['surface_tolerance_mm']) for d in detections]
            if ctx.get('preview_scene') is not None: measured = ctx['preview_scene']['measurements']
            polygon, depth = measured[ti]; det = detections[ti]; u,v = det.center_uv
            point = transform[:3,:3] @ np.asarray(ctx['deproject'](u,v,depth)) + transform[:3,3]
            if ctx.get('preview_scene') is not None: point = np.asarray(ctx['preview_scene']['points_mm'][ti])/1000.
            angle = math.radians(det.long_axis_angle_deg); half = cfg['orientation'].get('axis_sample_half_length_px',20)
            yaw = camera_axis_to_robot_yaw_deg(ctx['deproject'](u-half*math.cos(angle),v-half*math.sin(angle),depth),
                ctx['deproject'](u+half*math.cos(angle),v+half*math.sin(angle),depth),transform,
                gripper_yaw_offset_deg=cfg['orientation'].get('gripper_yaw_offset_deg',0),
                reference_yaw_deg=cfg['orientation'].get('reference_yaw_deg',0))
            if ctx.get('preview_scene') is not None:
                from integration.relative_placement import footprint_yaw_deg
                yaw = footprint_yaw_deg(polygon,cfg)
            grasp = build_top_down_plan(point,yaw,cfg['grasp'],
                canonical_target=task['target'], table_z_mm=surface.get('z_mm'))
            from integration.relative_placement import resolve_place_orientation
            policy, place_yaw = resolve_place_orientation(cfg,'named',float(yaw),
                profile_yaw=float(place.place_pose_mm_deg[5]),override=orientation_override)
            if abs(place_yaw-place.place_pose_mm_deg[5]) > 1e-9:
                for name in ('pre_place_pose_mm_deg','place_pose_mm_deg','retreat_pose_mm_deg'):
                    pose = list(getattr(place,name));pose[5] = place_yaw;setattr(place,name,pose)
                # Old paired joint angles describe the taught RZ, so solve anew.
                place.pre_place_joint_angles_deg = None
            # Rotate the measured object footprint to the taught placement yaw.
            theta = math.radians(place.place_pose_mm_deg[5]-yaw)
            rot = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
            center = np.asarray(place.place_pose_mm_deg[:2])-np.asarray(cfg['grasp'].get('tcp_xy_offset_mm',[0,0]))
            landing = (polygon-polygon.mean(axis=0)) @ rot.T + center
            envelope = clearance_envelope_obb(landing, surface['gripper_padding_mm'],
                surface['error_margin_mm'], cfg['grasp'].get('tcp_xy_offset_mm',[0,0]))
            geometry = {'station_name':task['destination_name'],'all_objects_checked':len(detections),
                'destination_center_robot_xy_mm':center.tolist(),'destination_footprint_robot_xy_mm':landing.tolist(),
                'clearance_envelope_robot_xy_mm':envelope.tolist(),'source_grasp_plan':grasp.as_dict(),
                'place_yaw_deg':place_yaw,'orientation_policy':policy,'grasp_yaw_deg':float(yaw),
                'landing_surface_z_mm':float(surface['z_mm']),
                'ik_validated':False,'path_validated':False,'collision_validated_3d':False}
            taught = place.destination_check_policy == 'operator_taught'
            geometry.update(destination_check_policy=place.destination_check_policy,
                placement_source=place.placement_source,clear_path_acknowledged=place.clear_path_acknowledged,
                source_occlusion_checked=True,destination_collision_checked=not taught,
                destination_surface_checked=not taught,checked_surface_pixels=0,local_surface_z_mm=None)
            from integration.scene_grounding import SceneGroundingError
            def reject_named(reason):
                raise SceneGroundingError(reason,reason,details=geometry)
            for name in ('pre_place_pose_mm_deg','place_pose_mm_deg','retreat_pose_mm_deg'):
                pose = getattr(place,name)
                if any(not cfg['grasp']['workspace_mm'][k][0] <= pose[n] <= cfg['grasp']['workspace_mm'][k][1]
                        for n,k in enumerate(('x','y','z'))):
                    reject_named('named_tcp_outside_workspace')
            if not taught:
                for n,k in enumerate(('x','y')):
                    if not cfg['grasp']['workspace_mm'][k][0] <= center[n] <= cfg['grasp']['workspace_mm'][k][1]:
                        reject_named('named_landing_outside_workspace')
                    if not surface['bounds_mm'][k][0] <= center[n] <= surface['bounds_mm'][k][1]:
                        reject_named('named_landing_outside_surface')
            for i,(other,other_depth) in enumerate(measured):
                if i==ti: continue
                height = float((transform[:3,:3] @ np.asarray(ctx['deproject'](*detections[i].center_uv,other_depth))+transform[:3,3])[2]*1000)
                if cv2.intersectConvexConvex(polygon.astype(np.float32),other.astype(np.float32))[0]>0 and height>point[2]*1000+surface['surface_tolerance_mm']:
                    reject_named('target_occluded_by_stacked_object')
                if not taught and convex_polygons_intersect(envelope,other):
                    reject_named('named_landing_occupied')
            if not taught:
                try:
                    checked, local_z = _surface_clear(envelope,ctx['frame'],ctx['deproject'],ctx['project'],
                        transform,float(surface['z_mm']),surface['surface_tolerance_mm'],cfg['depth'])
                except SceneGroundingError as exc:
                    exc.details.update(geometry)
                    raise
                geometry.update(checked_surface_pixels=checked,local_surface_z_mm=local_z)
        return {'geometry_passed':True,'frame_id':observation.frame_id,'grasp':grasp,'place':place,'geometry':geometry}

    def _prepare(self, run, task, observation, selected):
        sequence = run['sequence'].snapshot() if run.get('sequence') else None
        step = sequence['step_id'] if sequence else 1
        settings = run['pending'].snapshot()['settings'] if run.get('pending') else {}
        override = (sequence['steps'][step-1].get('orientation_override') if sequence else
                    settings.get('placement_orientation_overrides',{}).get(str(step)))
        prepared = self._plan_placement(run,task,observation,selected,orientation_override=override)
        grasp,place,geometry = prepared['grasp'],prepared['place'],prepared['geometry']
        source = next(i for i in observation.instances if i['instance_id'] == selected['target'])
        prepared['result_expectation'] = {
            'target_object':task['target'], 'destination_type':task['destination_type'],
            'spatial_relation':task['relation'],
            'source_center_robot_xy_mm':source['xyz_mm'][:2],
            'destination_center_robot_xy_mm':geometry['destination_center_robot_xy_mm'],
            'unselected_centers_mm':[i['xyz_mm'][:2] for i in observation.instances
                if i['class_name'] == task['target'] and i['instance_id'] != selected['target']],
            'expected_top_z_mm':geometry.get('expected_top_z_mm'),
            'place_yaw_deg':geometry['place_yaw_deg'],
            'landing_surface_z_mm':geometry['landing_surface_z_mm'],
        }
        run['controller']._program(grasp,place,False)  # Endpoint/parameter validation, no dispatch or solver.
        atomic_json(self._path(run['id'])/f'{observation.frame_id}.geometry.json',geometry)
        current_preview = next((p for p in run.get('placement_previews', []) if p.get('step_id') == step), None)
        if (current_preview is None or not current_preview.get('feasible')
                or current_preview.get('frame_id') != observation.frame_id
                or any(p.get('reason') in {'ambiguous_object_instances','object_not_detected'}
                       for p in run.get('placement_previews',[]))):
            self._refresh_placement_previews(run,selected=selected)
        return prepared

    def _execute(self, run, prepared, *, initialize, checkpoint):
        ctrl = run['controller']; ctrl.checkpoint = checkpoint
        with self._lock:
            self._visible_scene = None  # A dispatched move invalidates the semantic scene too.
            self._arm_motion_epoch += 1
            run['start_verified'] = False
        if initialize: raise ValueError('duplicate_post_capture_initialization_rejected')
        try:
            sequence = run['sequence'].snapshot() if run.get('sequence') else None
            final_step = sequence is None or sequence['step_id'] == len(sequence['steps'])
            result = ctrl.execute_pick_and_place(prepared['grasp'],prepared['place'],geometry_passed=True,
                                                 initialize=False,return_to_start=final_step)
            if run['mode'] == 'offline' and self.scene_factory is None:
                # Advance only the built-in synthetic scene; real observations always come from the camera.
                import cv2
                import numpy as np
                from integration.relative_placement import project_robot_points_mm
                observed = run['observation']
                selected = sequence['steps'][sequence['step_id']-1]['selected']['target']
                source = next(i for i in observed.instances if i['instance_id'] == selected)
                geometry = prepared['geometry']
                z = geometry.get('expected_top_z_mm')
                if z is None: z = source['xyz_mm'][2]
                points = np.column_stack((geometry['destination_footprint_robot_xy_mm'], [z]*4))
                cfg, intrinsics = observed.payload['config'], observed.payload['frame'].intrinsics
                pixels = project_robot_points_mm(points, cfg, intrinsics)
                center, size, angle = cv2.minAreaRect(np.asarray(pixels, dtype=np.float32))
                if size[0] < size[1]:
                    size = (size[1], size[0]);angle += 90
                inv = np.linalg.inv(np.asarray(cfg['calibration']['T_robot_from_camera']))
                depth = float((inv @ np.array([*geometry['destination_center_robot_xy_mm'], z, 1000.]))[2]/1000.)
                run['offline_scene']['objects'][source['index']].update(
                    center_uv=list(center), size_px=list(size), angle_deg=angle, depth_m=depth)
            return result
        finally:
            if run['mode']=='real':
                self._write_controller_state({'holding_uncertain':ctrl.gripper_holding_uncertain,
                    'holding':ctrl.gripper_holding_uncertain, 'at_start':ctrl.at_start, 'run_id':run['id'],
                    'source':'controller_release_state'})

    def _advance(self, run):
        while not run['cancelled']:
            state = run['sequence'].advance()
            if state['status']=='completed' and not run['cancelled']:
                self._capture_completion(run, state)
            if run['sequence'].persist_boundary is None:
                self._save(run)
            if state['status']!='running': break

    def _capture_completion(self, run, state, *, force=False):
        """Record the post-return scene once, without dispatching another move."""
        if run['cancelled']: return
        source_frame = state['steps'][-1].get('frame_id')
        previous = run.get('completion_capture') or {}
        if not force and previous.get('source_frame_id') == source_frame and previous.get('status') == 'captured':
            return run['completion_observation']
        run['completion_capture'] = {'status':'capturing','authorizes_motion':False,
                                     'source_frame_id':source_frame}
        self._save(run)
        try:
            ctrl = run.get('controller')
            if ctrl is None or ctrl.gripper_holding_uncertain or not ctrl.at_start:
                raise ValueError('completion_capture_requires_empty_start')
            settle = 0.3 if getattr(ctrl, '_last_arrival_verified', False) else float(run['config']['robot'].get('start_settle_s', 1.5))
            ctrl.sleep(settle)
            if run['cancelled']:
                run['completion_capture']['status'] = 'cancelled'
                run['sequence'].cancel()
                return
            observed = self._observe(run, state['step_id'], semantic_only=True)
            if run['cancelled']:
                run['completion_capture']['status'] = 'cancelled'
                run['sequence'].cancel()
                self._visible_scene = None
                return
            run['final_preview'] = observed.display
            run['completion_observation'] = observed
            # This image documents completion; a new task obtains its own scene.
            self._visible_scene = None
            run['completion_capture'].update(status='captured',frame_id=observed.frame_id,
                captured_at_unix_ms=observed.captured_at_unix_ms)
            self._event(run,'perception','completion_scene_observed',frame_id=observed.frame_id)
            return observed
        except Exception as exc:
            self._visible_scene = None
            run['completion_capture'].update(status='failed',reason=str(exc))
            self._event(run,'perception','completion_scene_capture_failed',reason=str(exc))
            raise

    def _cleanup(self, run):
        errors = []
        ctrl = run.get('controller')
        reusable_open = (run['mode'] == 'real' and ctrl is not None
            and run.get('start_verified') is True and getattr(ctrl, 'at_start', False) is True
            and getattr(ctrl, '_gripper_open_confirmed', False) is True
            and getattr(ctrl, 'gripper_holding_uncertain', True) is False
            and not run.get('cancelled') and run.get('status') != 'failed')
        run.pop('open_empty_evidence', None)
        for key,method in [('camera','disconnect'),('controller','close')]:
            obj = run.get(key)
            if obj:
                try:
                    callback = getattr(obj,method,None)
                    if callback: callback()
                    elif key=='controller' and getattr(obj,'mc',None) is not None:
                        callback = getattr(obj.mc,'close',None)
                        if callback: callback()
                except Exception as exc: errors.append(f'{key}:{type(exc).__name__}')
            run[key] = None
        if run.get('lease') and not errors:
            run['lease'].release(); run['lease'] = None
        if reusable_open and not errors and not self._holding():
            run['open_empty_evidence'] = {'arm_state_marker':self._arm_state_marker(),
                                          'config_revision':run.get('config_revision')}
        if errors: run['cleanup_error'] = errors

    def frame_bytes(self, run_id, frame_id):
        if len(frame_id)!=32 or any(c not in '0123456789abcdef' for c in frame_id):
            raise ValueError('invalid_frame_id')
        path = self._path(run_id)/f'{frame_id}.png'
        if path.is_symlink(): raise ValueError('frame_symlink_forbidden')
        return path.read_bytes()

    def acknowledge_empty(self, payload):
        with self.service._lock, self._lock:
            self._maintenance_idle()
            if self._voice_busy(): raise ValueError('voice_busy')
            if self.owns_resources() or payload != {'confirm':'EMPTY_GRIPPER'}:
                raise ValueError('idle_and_manual_empty_gripper_confirmation_required')
            self.holding_path.parent.mkdir(parents=True,exist_ok=True)
            self._write_controller_state({'holding_uncertain':False,'source':'operator_empty_hand_check'})
            return {'holding_uncertain':False}

    def shutdown(self):
        with self._lock:
            self._closing = True
            self._voice_stop.set()
            run = self._runs.get(self._active)
        if run: self.action(run['id'],'cancel',{})
        if self._thread: self._thread.join(timeout=20)
        if run and (self._thread is None or not self._thread.is_alive()): self._cleanup(run)
        if self._voice_thread: self._voice_thread.join(timeout=2)
        if getattr(self, '_teaching_controller', None) is not None:
            self.teaching({'action': 'exit', 'confirm': 'EXIT_TEACH_MODE'})
