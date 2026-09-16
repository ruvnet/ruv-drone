#!/usr/bin/env bash
# Local desktop launcher. No network binding beyond the Rust loopback listener.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if ! curl --silent --fail --max-time 2 http://127.0.0.1:8777/health >/dev/null; then
  cargo build --release --locked --offline
  mkdir -p runs
  nohup ./target/release/ruv-navigation-lab serve >runs/server.log 2>&1 &
  server_pid=$!
  printf '%s\n' "$server_pid" >runs/server.pid
  ready=false
  for attempt in {1..40}; do
    if curl --silent --fail --max-time 1 http://127.0.0.1:8777/health >/dev/null; then ready=true; break; fi
    kill -0 "$server_pid" 2>/dev/null || { cat runs/server.log >&2; exit 1; }
    sleep 0.1
  done
  "$ready" || { printf 'Server did not become ready; see runs/server.log\n' >&2; exit 1; }
fi
if [[ "${1:-}" != "--server-only" ]]; then xdg-open http://127.0.0.1:8777; fi
