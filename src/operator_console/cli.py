"""Launch the native V2 local console; no retired task pipeline."""
from __future__ import annotations

import argparse
from .service_v2 import ConsoleServiceV2
from .server import serve


def main(argv=None):
    parser = argparse.ArgumentParser(description='VisionPick V2 本機工作台')
    subs = parser.add_subparsers(dest='command', required=True)
    ui = subs.add_parser('ui', help='啟動 V2 工作台')
    ui.add_argument('--config', default='config/v0.yaml')
    ui.add_argument('--runs-dir')
    ui.add_argument('--port', type=int, default=8765)
    ui.add_argument('--no-browser', action='store_true')
    backend = ui.add_mutually_exclusive_group()
    backend.add_argument('--hardware', action='store_true')
    backend.add_argument('--demo', action='store_true')
    ui.add_argument('--enable-motion', action='store_true')
    # Existing launchers use these environment hints; all work uses V2 routes.
    ui.add_argument('--mode', choices=['full', 'offline', 'realsense_only'])
    ui.add_argument('--llm', choices=['api', 'mock'])
    args = parser.parse_args(argv)
    service = ConsoleServiceV2(config=args.config, runs_root=args.runs_dir,
                              hardware_enabled=args.hardware, motion_enabled=args.hardware and args.enable_motion)
    serve(service, port=args.port, open_browser=not args.no_browser)
    return 0
