#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ -x ".venv-console/bin/python" ]]; then VP_PY=".venv-console/bin/python"; else VP_PY="python3"; fi
cat <<'EOF'
VisionPick 智慧抓取操作台
  1) 離線示範（不使用設備）
  2) 相機測試（手臂不會移動）
  3) 完整硬體（允許手臂移動）
EOF
read -r -p "選擇模式 [1-3]：" choice
case "$choice" in
  1) exec "$VP_PY" run_v0.py ui --demo --mode offline --llm mock --no-browser ;;
  2) exec "$VP_PY" run_v0.py ui --hardware --mode realsense_only --llm api --no-browser ;;
  3) echo "危險時請使用手臂實體急停。"; exec "$VP_PY" run_v0.py ui --hardware --mode full --llm api --enable-motion --no-browser ;;
  *) echo "請選擇1、2或3。" >&2; exit 2 ;;
esac
