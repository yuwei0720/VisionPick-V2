"""Local engineering controls with a narrow, versioned configuration contract."""
from __future__ import annotations

from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time

import yaml

from .direct_control_v2 import ArrivalSettings, DirectRobotControllerV2, _six
from .events import atomic_json, redact, utc_now
from .lease import RobotLease


# key: (label, minimum, maximum, default, integer)
FIELDS = {
    'grasp.tcp_xy_offset_mm': ('TCP XY 補償（mm）', -50, 50, [-6., -6.], False),
    'grasp.grasp_z_offset_mm': ('抓取 Z 補償（mm）', -30, 30, 1., False),
    'grasp.rx_deg': ('預設姿態 Rx（°）', -180, 180, -175., False),
    'grasp.ry_deg': ('預設姿態 Ry（°）', -180, 180, -5., False),
    'scene.sequence_gap_mm': ('平面放置間距（mm）', 5, 200, 50., False),
    'robot.motion_timeout_s': ('舊版到位逾時（秒；V2 不使用）', .1, 300, 25., False),
    'robot.gripper.wait_s': ('夾爪等待（秒）', 0, 30, 3., False),
    'robot.fallback_wait_s': ('備援等待（秒）', 0, 30, 3., False),
    'robot.start_angle_speed': ('Start 關節速度', 1, 100, 15, True),
    'robot.place_joint_speed': ('放置關節速度', 1, 100, 15, True),
    'robot.coordinate_speed': ('座標速度', 1, 100, 15, True),
    'robot.vertical_segment_step_mm': ('方案 A 垂直步長（mm；B/C 固定單段）', 0, 500, 35., False),
    'orientation.fixed_place_yaw_deg': ('固定放置 RZ（°）', -180, 180, -45., False),
}
ENUM_FIELDS = {
    'robot.vertical_motion_mode': ('垂直升降模式（A/B/C）', ['segmented_mode0','linear_mode1','single_mode0'], 'segmented_mode0'),
    'orientation.place_orientation_policy_relative': ('相對放置朝向', ['align_reference','keep_target','fixed_deg'], 'align_reference'),
    'orientation.place_orientation_policy_self_relative': ('自身位移朝向', ['keep_target','fixed_deg'], 'keep_target'),
    'orientation.place_orientation_policy_swap': ('交換原位朝向', ['match_initial','keep_target','fixed_deg'], 'match_initial'),
    'orientation.place_orientation_policy_named': ('具名站點朝向', ['profile_pose','keep_target'], 'profile_pose'),
}


def _atomic_bytes(path, data):
    path = Path(path)
    if path.is_symlink():
        raise ValueError('engineering_symlink_forbidden')
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.engineering-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _patch_yaml(text, dotted, value):
    """Replace just the selected YAML node, preserving unrelated comments/lines."""
    lines = text.splitlines(keepends=True)
    start, end, indent = 0, len(lines), 0
    for n, key in enumerate(dotted.split('.')):
        match = next((i for i in range(start, end)
                      if re.match(r'^' + ' ' * indent + re.escape(key) + r'\s*:', lines[i])), None)
        if match is None:
            if n != len(dotted.split('.')) - 1:
                lines.insert(end, ' ' * indent + key + ':\n')
                match = end
                end += 1
            else:
                lines.insert(end, ' ' * indent + key + ': ' + json.dumps(value) + '\n')
                return ''.join(lines)
        stop = next((i for i in range(match + 1, end) if lines[i].strip()
                     and not lines[i].lstrip().startswith('#')
                     and not lines[i].lstrip().startswith('- ')
                     and len(lines[i]) - len(lines[i].lstrip()) <= indent), end)
        if n == len(dotted.split('.')) - 1:
            lines[match:stop] = [' ' * indent + key + ': ' + json.dumps(value) + '\n']
            return ''.join(lines)
        start, end, indent = match + 1, stop, indent + 2
    raise ValueError('invalid_configuration_path')


class EngineeringConsoleV2:
    def __init__(self, runtime):
        self.runtime = runtime
        self.service = runtime.service
        self.root = runtime.root
        self.log_path = self.root / 'operator_data' / 'engineering_events_v2.json'
        self.last_pose = None

    @contextmanager
    def _idle(self):
        with self.service._lock, self.runtime._lock:
            if self.runtime._closing or self.runtime.owns_resources():
                raise ValueError('engineering_locked_while_running')
            active = self.runtime._runs.get(self.runtime._active)
            if active and active.get('sequence') and active['sequence'].snapshot()['status'] not in {'completed','failed','cancelled'}:
                raise ValueError('engineering_locked_while_running')
            self.runtime._maintenance_idle()
            if getattr(self.service, '_quarantined_leases', []):
                raise ValueError('robot_ownership_quarantined')
            yield

    def _paths(self):
        return [Path(self.service.config), self.service.place_profiles.path,
                self.service.assets.path, self.log_path]

    @contextmanager
    def _transaction(self, *, include_table=False):
        """Rollback station/table/registry/log as a single local mutation."""
        with self.service.place_profiles._lock, self.service.assets._lock:
            paths = self._paths()[1:]
            if include_table:
                paths.append(self.service.table_surface.path)
            if any(path.is_symlink() for path in paths):
                raise ValueError('engineering_symlink_forbidden')
            before = {p: p.read_bytes() if p.exists() else None for p in paths}
            try:
                yield
            except Exception:
                for path, data in before.items():
                    if data is None:
                        path.unlink(missing_ok=True)
                    else:
                        _atomic_bytes(path, data)
                raise

    def _events(self):
        if self.log_path.is_symlink():
            raise ValueError('engineering_symlink_forbidden')
        return json.loads(self.log_path.read_text(encoding='utf-8')) if self.log_path.exists() else []

    def _event(self, action, **detail):
        events = self._events()
        events.append(redact({'at': utc_now(), 'action': action, **detail}))
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(self.log_path, events)

    def read(self):
        path = Path(self.service.config)
        if path.is_symlink():
            raise ValueError('engineering_symlink_forbidden')
        raw = path.read_bytes()
        cfg = yaml.safe_load(raw)
        values, definitions = {}, {}
        for key, (label, low, high, default, integer) in FIELDS.items():
            value = cfg
            for part in key.split('.'):
                value = value.get(part) if isinstance(value, dict) else None
            values[key] = default if value is None else value
            definitions[key] = {'label': label, 'min': low, 'max': high, 'integer': integer}
        for key, (label, choices, default) in ENUM_FIELDS.items():
            value = cfg
            for part in key.split('.'):
                value = value.get(part) if isinstance(value, dict) else None
            values[key] = default if value is None else value
            definitions[key] = {'label':label, 'choices':choices}
        return {'revision': hashlib.sha256(raw).hexdigest(), 'values': values, 'fields': definitions,
                'profiles': self.service.place_profiles.status()['profiles'],
                'table': self.service.table_surface.status(cfg), 'events': self._events(),
                'last_pose': self.last_pose, 'catalog': self.runtime._catalog().status(),
                'teaching_active': getattr(self.runtime, '_teaching_controller', None) is not None,
                'fingerprints': self.service.place_profiles.status().get('dependencies', {}),
                'ik_events': [e for run in self.runtime._runs.values() for e in run.get('events', [])
                              if any(word in str(e).lower() for word in ('ik', 'transfer'))]}

    def save(self, payload):
        if not isinstance(payload, dict) or set(payload) != {'values', 'expected_revision'}:
            raise ValueError('invalid_engineering_fields')
        values = payload['values']
        if not isinstance(values, dict) or not values or set(values) - (FIELDS.keys() | ENUM_FIELDS.keys()):
            raise ValueError('unknown_engineering_setting')
        for key, value in values.items():
            if key in ENUM_FIELDS:
                if type(value) is not str or value not in ENUM_FIELDS[key][1]:
                    raise ValueError('engineering_value_out_of_range:' + key)
                continue
            _, low, high, _, integer = FIELDS[key]
            items = value if key == 'grasp.tcp_xy_offset_mm' else [value]
            if key == 'grasp.tcp_xy_offset_mm' and (not isinstance(value, list) or len(value) != 2):
                raise ValueError('tcp_xy_requires_two_numbers')
            if any(type(v) not in (int, float) or not math.isfinite(v) or not low <= v <= high
                   or integer and type(v) is not int for v in items):
                raise ValueError('engineering_value_out_of_range:' + key)
        with self._idle(), self.service.place_profiles._lock, self.service.assets._lock:
            before = self.read()
            if payload['expected_revision'] != before['revision']:
                raise ValueError('stale_engineering_revision')
            paths = self._paths()
            if any(path.is_symlink() for path in paths):
                raise ValueError('engineering_symlink_forbidden')
            originals = {path: path.read_bytes() if path.exists() else None for path in paths}
            text = originals[paths[0]].decode('utf-8')
            for key, value in values.items():
                text = _patch_yaml(text, key, value)
            cfg = yaml.safe_load(text)
            ArrivalSettings.from_config(cfg['robot'])
            previous = self.service.place_profiles._dependencies()
            eligible = [r['profile_id'] for r in before['profiles'] if r['effective_status'] == 'active']
            try:
                _atomic_bytes(paths[0], text.encode('utf-8'))
                synced = self.service.place_profiles.sync_motion_fingerprint(previous, eligible)
                snapshot = self.service.assets.register_current()
                self.runtime.refresh_waiting_placement_settings()
                after = self.read()
                if any(r['profile_id'] in eligible and r['effective_status'] != 'active' for r in after['profiles']):
                    raise ValueError('station_sync_failed')
                self._event('settings_saved', changes={k: {'before': before['values'][k], 'after': v}
                            for k, v in values.items()}, synced_profile_ids=synced,
                            asset_snapshot_id=snapshot['snapshot_id'])
                return {**self.read(), 'synced_profile_ids': synced}
            except Exception:
                for path, original in originals.items():
                    if original is None:
                        if path.exists():
                            path.unlink()
                    else:
                        _atomic_bytes(path, original)
                raise

    def pose(self, payload):
        record = self.runtime.read_pose(payload)
        self.last_pose = record
        self._event('pose_read', pose=record)
        return record

    def gripper(self, payload):
        if payload not in ({'action': 'open', 'confirm': 'OPEN_GRIPPER'},
                           {'action': 'close', 'confirm': 'CLOSE_GRIPPER'}):
            raise ValueError('explicit_gripper_confirmation_required')
        with self._idle():
            if not self.service.hardware_enabled or not self.service.motion_enabled:
                raise ValueError('real_motion_not_enabled_for_this_server')
            cfg = self.service._base_config()
            ctrl = DirectRobotControllerV2(cfg, settings=self.runtime.settings.freeze().settings)
            lease = RobotLease(cfg['robot']['port']).acquire()
            try:
                ctrl.connect()
                self.last_pose = None
                self.runtime._arm_motion_epoch += 1
                self.runtime._visible_scene = None
                self.runtime._calibration_frame = None
                self.runtime.holding_path.parent.mkdir(parents=True, exist_ok=True)
                atomic_json(self.runtime.holding_path, {'holding_uncertain': True,
                            'reason': 'manual_gripper_requires_empty_hand_check'})
                (ctrl.open_gripper if payload['action'] == 'open' else ctrl.close_gripper)()
            finally:
                try:
                    if getattr(ctrl, 'mc', None) and callable(getattr(ctrl.mc, 'close', None)):
                        ctrl.mc.close()
                except Exception:
                    # An uncertain transport close must retain exclusive ownership.
                    self.service._quarantined_leases.append(lease)
                    raise
                else:
                    lease.release()
            self._event('manual_gripper', action_name=payload['action'])
            return {'action': payload['action'], 'holding_uncertain': True}

    def station(self, payload):
        allowed = {'action', 'profile_id', 'name', 'aliases', 'confirm', 'clear_path_acknowledged'}
        if not isinstance(payload, dict) or set(payload) - allowed:
            raise ValueError('invalid_station_fields')
        with self._idle(), self._transaction():
            action, pid = payload.get('action'), payload.get('profile_id')
            manager = self.service.place_profiles
            if action == 'metadata':
                result = manager.update_metadata(pid, display_name=payload.get('name'), aliases=payload.get('aliases', []))
            elif action in {'activate', 'revalidate'}:
                result = manager.activate(pid, confirm=payload.get('confirm'),
                    clear_path_acknowledged=payload.get('clear_path_acknowledged'))
            elif action == 'disable':
                result = manager.disable(pid, confirm=payload.get('confirm'))
            elif action == 'delete':
                result = manager.delete(pid, confirm=payload.get('confirm'), force_unbind=True)
            elif action == 'default':
                result = manager.set_default(pid)
            elif action == 'teach':
                if not self.last_pose or not 0 <= time.time()*1000-self.last_pose['read_at_unix_ms'] <= 60000:
                    raise ValueError('fresh_pose_read_required')
                if payload.get('clear_path_acknowledged') is not True:
                    raise ValueError('clear_path_acknowledgement_required')
                from .runtime_v2 import _identity
                if (self.last_pose.get('config_revision', _identity(self.service._base_config())) != _identity(self.service._base_config())
                        or self.last_pose.get('arm_epoch', self.runtime._arm_motion_epoch) != self.runtime._arm_motion_epoch):
                    raise ValueError('pose_changed_read_again')
                pose = _six(self.last_pose['coords_mm_deg'], 'fresh_tcp')
                angles = _six(self.last_pose['angles_deg'], 'fresh_joints')
                created = manager.create_candidate({'label': payload.get('name'), 'aliases': payload.get('aliases', []),
                    'base_profile_id': pid or None, 'reference_object': 'table', 'spatial_relation': 'on_top_of',
                    'planner_mode': 'hybrid', 'place_pose_mm_deg': pose, 'approach_clearance_mm': 60,
                    'retreat_clearance_mm': 80, 'place_joint_angles_deg': angles,
                    'preferred_ik_enabled': True, 'preferred_ik_reference_angles_deg': angles,
                    'placement_source': {'type': 'robot_pose_read', 'record': self.last_pose}})
                result = manager.activate(created['profile']['profile_id'], confirm='ACTIVATE_PLACE_PROFILE',
                                          clear_path_acknowledged=True)
                result = {'profile': result}
                # PLACE feedback is paired only with PLACE, never PRE-PLACE.
            else:
                raise ValueError('unknown_station_action')
            self.runtime._catalog()._valid_catalog()
            snapshot = self.service.assets.register_current()
            self._event('station_' + action, profile_id=pid, result=result, asset_snapshot_id=snapshot['snapshot_id'])
            return result

    def table(self, payload):
        return self.runtime.table_calibrate(payload)
