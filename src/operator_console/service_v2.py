"""V2 infrastructure only: configuration, local assets and passive discovery."""
from pathlib import Path
import os
import threading

import yaml

from .assets import AssetRegistry
from .place_profiles import PlaceProfileManager
from .place_capture import VisualPlaceCaptureManager
from .table_surface import TableSurfaceManager
from .serial_tools import enumerate_ports


class ConsoleServiceV2:
    def __init__(self, root=None, *, config='config/v0.yaml', runs_root=None,
                 hardware_enabled=False, motion_enabled=False, confirmation_timeout=180):
        self.root = Path(root or Path(__file__).resolve().parents[2]).resolve()
        self.config = str(self.root / config)
        self.runs_root = Path(runs_root or self.root / 'runs')
        self.hardware_enabled = bool(hardware_enabled)
        self.motion_enabled = bool(motion_enabled)
        self.confirmation_timeout = confirmation_timeout
        self._lock = threading.RLock()
        self._quarantined_leases = []
        self.assets = AssetRegistry(self.root, config_path=self.config)
        self.place_profiles = PlaceProfileManager(self.root, self.config, asset_registry=self.assets)
        self.place_capture = VisualPlaceCaptureManager(self.root, self.config)
        self.table_surface = TableSurfaceManager(self.root)

    def _base_config(self):
        data = yaml.safe_load(Path(self.config).read_text(encoding='utf-8')) or {}
        if not isinstance(data, dict):
            raise ValueError('config_root_must_be_object')
        return data

    def device_status(self):
        cfg = self._base_config()
        ports = enumerate_ports()
        robot = cfg.get('robot', {})
        match = next((p for p in ports if p.get('device') == robot.get('port')), None)
        camera = {'detected': False, 'count': 0, 'devices': [], 'detail': 'pyrealsense2 unavailable'}
        try:
            import pyrealsense2 as rs
            devices = []
            for dev in rs.context().query_devices():
                def info(kind):
                    return dev.get_info(kind) if dev.supports(kind) else None
                devices.append({'name': info(rs.camera_info.name), 'serial_number': info(rs.camera_info.serial_number)})
            camera = {'detected': bool(devices), 'count': len(devices), 'devices': devices}
        except Exception as exc:
            camera['detail'] = str(exc)
        return {'serial_ports': ports, 'camera': camera,
                'robot': {'detected': bool(match), 'configured_port': robot.get('port'),
                          'baud': robot.get('baud', 115200),
                          'ready_for_motion': bool(match and self.hardware_enabled and self.motion_enabled)},
                'ai': {'provider': 'Gemini', 'api_key_present': bool(os.getenv('GEMINI_API_KEY') or os.getenv('GOOGLE_API_KEY'))}}

    def shutdown(self):
        # Runtime owns and closes robot/camera connections. Quarantined leases
        # remain held until process exit after a transport close failure.
        pass
