#!/usr/bin/env bash
# embedder.sh -- EmbeddingGemma 2 on the Jetson GPU, for keyframe_memory.py.
#
#   phase4/tools/embedder.sh start | stop | status
#
# Runs on the HOST, not in the rover container (the image is never modified):
# llama.cpp b11753's own CUDA build, an embeddings-only server on 127.0.0.1:8097.
# The container is host-network, so keyframe_memory.py reaches it there.
#
# THE LIGHT SETTING, measured with the whole stack up (2026-10-29, FRAME_MEMORY.md):
#   70 image tokens + the Q8 vision projector: 0.31 s an image non-stop, /odom
#   19.0-19.4 Hz (stack alone 18.7-19.9), vo 31 Hz, ~1.6 GB RAM.
#   140 tokens + the F16 projector: GPU 99 %, /odom 13.7 Hz (gate wants 15), vo
#   25.8 Hz, and available RAM fell to ~0.2 GB. Do not raise TOKENS without
#   re-running that measurement.
#
# THE GUARD. RAM is the tight resource on this 8 GB board, not GPU time. A
# guard loop runs beside the server and stops it (never anything else) when
# MemAvailable falls under GUARD_MB -- the robot's nodes win, frame memory goes
# quiet and says so (keyframe_memory reports "embedder": false). It also stops
# the server when the rover container is gone (./rover stop / fleet.sh stop),
# so 1.6 GB is never held for a robot that is not running.
#
# Files (models/ is gitignored), recreate with:
#   cd ~/rover/models/embed
#   for f in llama-b11753-bin-ubuntu-cuda-13.4-arm64.tar.gz cudart-llama-b11753-bin-ubuntu-cuda-13.4-arm64.tar.gz; do
#     curl -LO https://github.com/ggml-org/llama.cpp/releases/download/b11753/$f && tar xzf $f; done
#   for f in embeddinggemma-2-Q8_0.gguf mmproj-Q8_0.gguf; do
#     curl -LO https://huggingface.co/unsloth/embeddinggemma-2-GGUF/resolve/main/$f; done
#   reference/  any ~40 unrelated photos (keyframe_memory's "nothing here" level)
set -uo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
D="$REPO/models/embed"
BIN="$D/llama-b11753"
PORT="${EMBED_PORT:-8097}"
TOKENS="${EMBED_TOKENS:-70}"
GUARD_MB="${EMBED_GUARD_MB:-600}"
CONTAINER="${NAME:-rover}"
LOG="$REPO/logs/embedder.log"
PIDF=/tmp/rover_embedder.pid          # the guard's pid; the server is its child

server_pid(){ pgrep -f "^$BIN/llama-server .*--port $PORT" | head -1; }   # anchored: an ssh command that merely MENTIONS it must not match
guard_pid(){ [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null && cat "$PIDF"; }
avail_mb(){ awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo; }

guard(){  # runs detached: start the server, then watch RAM and the container
  export LD_LIBRARY_PATH="$BIN:$D/cudart-llama-b11753-bin-ubuntu-cuda-13.4-arm64"
  "$BIN/llama-server" -m "$D/embeddinggemma-2-Q8_0.gguf" --mmproj "$D/mmproj-Q8_0.gguf" \
    --embedding --pooling mean --image-min-tokens "$TOKENS" --image-max-tokens "$TOKENS" \
    -c 512 -ub 512 -b 512 -ngl 99 -np 1 --host 127.0.0.1 --port "$PORT" >> "$LOG" 2>&1 &
  local spid=$!
  echo "$(date -Is) embedder: server pid $spid, $TOKENS image tokens, guard ${GUARD_MB} MB" >> "$LOG"
  while kill -0 "$spid" 2>/dev/null; do
    local a; a=$(avail_mb)
    if [ "$a" -lt "$GUARD_MB" ]; then
      echo "$(date -Is) embedder: GUARD -- available RAM ${a} MB < ${GUARD_MB} MB, stopping the model server (the robot keeps running)" >> "$LOG"
      kill "$spid"; break
    fi
    if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
      echo "$(date -Is) embedder: container '$CONTAINER' is gone, stopping the model server" >> "$LOG"
      kill "$spid"; break
    fi
    sleep 1
  done
  wait "$spid" 2>/dev/null
  echo "$(date -Is) embedder: server stopped" >> "$LOG"
  rm -f "$PIDF"
}

case "${1:-status}" in
  start)
    if [ -n "$(server_pid)" ]; then echo "  embedder already running (pid $(server_pid))"; exit 0; fi
    [ -x "$BIN/llama-server" ] || { echo "  no llama-server in $BIN -- see the header of $0"; exit 1; }
    a=$(avail_mb)
    # Loaded, the server shows ~1.6 GB RSS but MemAvailable drops only ~0.5 GB
    # (the weights are a memory-mapped file): 2.27 GB -> 1.74 GB with the stack up.
    if [ "$a" -lt $((GUARD_MB + 900)) ]; then
      echo "  NOT starting: available RAM ${a} MB; loading costs ~0.5-0.9 GB and the guard keeps ${GUARD_MB} MB"; exit 1
    fi
    mkdir -p "$(dirname "$LOG")"
    nohup bash "$0" _guard >/dev/null 2>&1 &
    echo $! > "$PIDF"
    for i in $(seq 1 60); do curl -sf "localhost:$PORT/health" >/dev/null && break; sleep 1; done
    if curl -sf "localhost:$PORT/health" >/dev/null; then
      echo "  embedder up on 127.0.0.1:$PORT ($TOKENS image tokens), available RAM now $(avail_mb) MB"
    else
      echo "  embedder did NOT come up -- tail $LOG"; exit 1
    fi
    ;;
  _guard) guard ;;
  stop)
    g=$(guard_pid); s=$(server_pid)
    [ -n "$s" ] && kill "$s"; [ -n "$g" ] && kill "$g" 2>/dev/null
    rm -f "$PIDF"
    [ -n "$s$g" ] && echo "  embedder stopped" || echo "  embedder was not running"
    ;;
  status)
    s=$(server_pid)
    if [ -n "$s" ] && curl -sf "localhost:$PORT/health" >/dev/null; then
      echo "  embedder up (pid $s, $(ps -o rss= -p "$s" | awk '{printf "%.0f MB", $1/1024}')), available RAM $(avail_mb) MB"
    else
      echo "  embedder down"; tail -2 "$LOG" 2>/dev/null | sed 's/^/    /'; exit 1
    fi
    ;;
  *) echo "usage: $0 start|stop|status"; exit 2 ;;
esac
