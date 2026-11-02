"""Loopback-only HTTP transport. Mutations require same-origin JSON + session token."""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, unquote
import json
import mimetypes
import secrets
import threading
import webbrowser
import base64
import hashlib
from time import monotonic

from .events import redact
from . import VERSION

STATIC = Path(__file__).with_name("static")


def create_server(service, port=8765, *, v2_runtime=None):
    if type(port) is not int or not 0 <= port <= 65535: raise ValueError("Invalid port")
    session_token = secrets.token_urlsafe(32)
    if v2_runtime is None:
        from .runtime_v2 import ConsoleRuntimeV2
        v2_runtime = ConsoleRuntimeV2(service)

    class Handler(BaseHTTPRequestHandler):
        server_version = f"VisionPickConsole/{VERSION}"

        def log_message(self, *args): pass  # No request headers/tokens in access logs.

        def _discard_rejected_body(self):
            # Keep authentication first, but avoid TCP reset from unread POST bytes.
            self.close_connection = True
            if getattr(self, '_request_body_read', False):
                return
            self._request_body_read = True
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if size <= 0:
                    return
                previous_timeout = self.connection.gettimeout()
                # Discarding transport bytes does not accept/parse an oversized
                # request. Bound both total work and elapsed time, including
                # bodies larger than the ordinary 64 KiB application limit.
                remaining, deadline = min(size, 6*1024*1024), monotonic()+.25
                try:
                    while remaining:
                        timeout = deadline-monotonic()
                        if timeout <= 0:
                            break
                        self.connection.settimeout(timeout)
                        chunk = self.rfile.read1(min(remaining,65536))
                        if not chunk:
                            break
                        remaining -= len(chunk)
                finally:
                    self.connection.settimeout(previous_timeout)
            except (OSError, ValueError):
                pass  # Incomplete/invalid bodies still receive the original rejection.

        def _respond(self, status, data, content_type="application/json; charset=utf-8", *, download=None, style_hash=None):
            if status >= 400 and self.command == 'POST':
                self._discard_rejected_body()
            if not isinstance(data, bytes): data = json.dumps(redact(data), ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            style = " 'sha256-" + style_hash + "'" if style_hash else ""
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'" + style + "; img-src 'self' data:; media-src 'self' blob:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
            if download: self.send_header("Content-Disposition", f'attachment; filename="{download}"')
            self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError): pass

        def _guard(self, mutation=False):
            actual_port = self.server.server_address[1]
            hosts = {f"127.0.0.1:{actual_port}", f"localhost:{actual_port}"}
            host = self.headers.get("Host", "")
            if host not in hosts: raise PermissionError("Host must be loopback")
            origin = self.headers.get("Origin")
            if origin and origin not in {"http://" + h for h in hosts}: raise PermissionError("Cross-origin request rejected")
            if mutation:
                token = self.headers.get("X-VisionPick-Token", "") or self.headers.get("X-EdgeAI-Token", "")
                if not secrets.compare_digest(token, session_token): raise PermissionError("Invalid session token")
                if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json": raise ValueError("JSON required")

        def _parts(self): return [unquote(p) for p in urlsplit(self.path).path.split("/") if p]

        def do_GET(self):
            try:
                self._guard()
                parts = self._parts()
                if not parts or parts == ['v2']:
                    page = (STATIC / 'v2' / 'index.html').read_bytes()
                    self._respond(200, page, 'text/html; charset=utf-8')
                elif parts == ['v2','app.js']:
                    self._respond(200,(STATIC/'v2'/'app.js').read_bytes(),'text/javascript; charset=utf-8')
                elif parts == ['v2','dashboard.css']:
                    self._respond(200,(STATIC/'v2'/'dashboard.css').read_bytes(),'text/css; charset=utf-8')
                elif len(parts) == 3 and parts[:2] == ['v2', 'assets'] and parts[2] in {
                        'mascot_capture.gif', 'mascot_moving.gif', 'mascot_parsing.gif',
                        'mascot_inspector_idle.gif', 'mascot_inspector_parsing.gif'}:
                    asset_root = STATIC / 'v2' / 'assets'
                    asset = asset_root / parts[2]
                    if asset_root.is_symlink() or asset.is_symlink():
                        raise ValueError('Symlink asset rejected')
                    if asset.resolve().parent != asset_root.resolve():
                        raise ValueError('Asset outside allowed directory')
                    # Optional artwork: an empty response invokes the SVG fallback
                    # without treating a deliberately absent mascot as a page error.
                    self._respond(200 if asset.is_file() else 204,
                                  asset.read_bytes() if asset.is_file() else b'', 'image/gif')
                elif parts == ['api','v2','engineering']:
                    self._respond(200,v2_runtime.engineering.read())
                elif parts == ['api','v2','bootstrap']:
                    self._respond(200, {'session':session_token, **v2_runtime.bootstrap()})
                elif len(parts)==4 and parts[:3]==['api','v2','voice']:
                    self._respond(200,v2_runtime.voice_state(parts[3]))
                elif parts == ['api','v2','settings']:
                    self._respond(200,v2_runtime.settings.read())
                elif len(parts)==4 and parts[:3]==['api','v2','runs']:
                    self._respond(200,v2_runtime.state(parts[3]))
                elif len(parts)==6 and parts[:3]==['api','v2','runs'] and parts[4]=='frames':
                    self._respond(200,v2_runtime.frame_bytes(parts[3],parts[5]),'image/png')
                elif parts == ['api','v2','places']:
                    self._respond(200, v2_runtime._catalog().status())
                elif parts == ['api','v2','table-surface']:
                    self._respond(200, service.table_surface.status(service._base_config()))
                elif len(parts) == 6 and parts[:4] == ['api','v2','places','captures'] and parts[5] == 'frame':
                    self._respond(200, service.place_capture.frame_path(parts[4]).read_bytes(), 'image/png')
                else: self._respond(404, {"error": "Not found"})
            except PermissionError as exc: self._respond(403, {"error": str(exc)})
            except FileNotFoundError as exc: self._respond(404, {"error": str(exc)})
            except (ValueError, KeyError, OSError) as exc: self._respond(400, {"error": str(exc)})

        def do_POST(self):
            self._request_body_read = False
            try:
                self._guard(mutation=True)
                parts = self._parts()
                if parts[:2] != ['api', 'v2']:
                    self._respond(404, {'error': 'Not found'})
                    return
                size = int(self.headers.get('Content-Length', '0'))
                limit = 6*1024*1024 if parts == ['api','v2','voice'] else 65536
                if not 0 < size <= limit: raise ValueError('Invalid request size')
                def invalid_constant(value): raise ValueError('Non-finite JSON number rejected')
                raw = self.rfile.read(size)
                self._request_body_read = True
                payload = json.loads(raw, parse_constant=invalid_constant)
                if not isinstance(payload, dict): raise ValueError('Object required')
                routes = {
                    ('voice',): (202, v2_runtime.voice),
                    ('preview',): (201, v2_runtime.preview),
                    ('runs',): (201, v2_runtime.create),
                    ('robot','home'): (202, v2_runtime.home),
                    ('robot','pose'): (200, v2_runtime.engineering.pose),
                    ('robot','teaching'): (200, v2_runtime.teaching),
                    ('places','capture'): (201, v2_runtime.capture_rgbd),
                    ('places','roi_capture'): (200, v2_runtime.roi_capture),
                    ('places','manage'): (200, v2_runtime.engineering.station),
                    ('table-surface','calibrate'): (200, v2_runtime.table_calibrate),
                    ('engineering','settings'): (200, v2_runtime.engineering.save),
                    ('engineering','pose'): (200, v2_runtime.engineering.pose),
                    ('engineering','gripper'): (200, v2_runtime.engineering.gripper),
                    ('engineering','station'): (200, v2_runtime.engineering.station),
                    ('engineering','table'): (200, v2_runtime.engineering.table),
                    ('recover-empty',): (200, v2_runtime.acknowledge_empty),
                }
                key = tuple(parts[2:])
                if key in routes:
                    status, callback = routes[key]
                    self._respond(status, callback(payload))
                elif key == ('settings',):
                    if set(payload) != {'overrides','expected_revision'}: raise ValueError('Invalid setting fields')
                    self._respond(200, v2_runtime.settings.save(**payload))
                elif len(parts) == 5 and parts[:3] == ['api','v2','runs']:
                    if parts[4] in {'retry_step','skip_step'} and set(payload) != {'run_id','step_id','frame_id','plan_version'}:
                        raise ValueError('invalid_step_recovery_fields')
                    self._respond(200, v2_runtime.action(parts[3], parts[4], payload))
                else:
                    self._respond(404, {'error': 'Not found'})
            except PermissionError as exc: self._respond(403, {'error': str(exc)})
            except RuntimeError as exc: self._respond(409, {'error': str(exc)})
            except (ValueError, KeyError, TypeError, OSError) as exc: self._respond(400, {'error': str(exc)})

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    server.v2_runtime = v2_runtime
    return server


def serve(service, *, port=8765, open_browser=True):
    server = create_server(service, port)
    url = f'http://127.0.0.1:{server.server_address[1]}'
    print(f'VisionPick V2: {url} (hardware={service.hardware_enabled}; motion={service.motion_enabled})')
    if open_browser:
        threading.Timer(.3, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            server.v2_runtime.shutdown()
            service.shutdown()
        finally:
            server.server_close()
