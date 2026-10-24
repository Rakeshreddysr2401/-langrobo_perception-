#!/usr/bin/env bash
# build_engine.sh — YOLO .pt -> TensorRT engine for THIS Jetson's TensorRT (./rover engine).
#
#   phase4/tools/build_engine.sh [models/yolov8n.pt] [height width]     (default 384 640)
#   -> models/<name>_<w>x<h>_trt<version>.engine
#
# trtexec runs on the host: JetPack 7.2's TensorRT 10.16, the same version orin-nav:2.0
# ships, so the engine loads in the image (yolo_backend.py). An engine only loads in the
# TensorRT that built it, so the version is in the file name and in the header, and a
# JetPack update means: run this again.
#
# Steps, each with its own error:
#   1. ONNX export in a throwaway orin-nav:2.0 container (CPU only). Ultralytics,
#      torch (CPU) and onnx stay OUT of the image (2.0 has no torch at all): pip
#      installs them ONCE into ~/.cache/rover-onnx-export-2 (~6 GB, needs internet
#      the first time), and only the container that exports sees them.
#   2. trtexec on the host, FP16 (the same build as Ultralytics' half=True).
#   3. Wrap: Ultralytics' header layout (int32 length + JSON: names, imgsz,
#      stride, task, batch, half) plus "trt": the exact TensorRT version.
# Nothing here moves the robot; step 2 loads the GPU for ~1-7 min.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE=${IMAGE:-orin-nav:2.0}
PT=${1:-$REPO/models/yolov8n.pt}
H=${2:-384}; W=${3:-640}
TRTEXEC=/usr/src/tensorrt/bin/trtexec
DEPS="$HOME/.cache/rover-onnx-export-2"   # ultralytics + torch (CPU) + onnx, for orin-nav:2.x

die(){ echo "  ✗ $*" >&2; exit 1; }
[ -f "$PT" ] || die "no model at $PT"
[ -x "$TRTEXEC" ] || die "no $TRTEXEC: the host TensorRT (JetPack's libnvinfer-bin) is missing"
TRTV=$(python3 -c 'import tensorrt; print(tensorrt.__version__)' 2>/dev/null) \
  || die "the host has no TensorRT Python bindings (python3-libnvinfer)"
TAG=$(echo "$TRTV" | cut -d. -f1-3)
STEM=$(basename "$PT" .pt)
OUT="$REPO/models/${STEM}_${W}x${H}_trt${TAG}.engine"
WORK=$(mktemp -d); trap 'rm -rf "$WORK"' EXIT
cp "$PT" "$WORK/model.pt"

echo "[engine] $STEM -> ${W}x${H} FP16 for TensorRT $TRTV"
echo "  1/3 ONNX export (container, CPU)"
if [ ! -d "$DEPS/ultralytics" ]; then
  echo "      first run: fetching ultralytics, torch (CPU) and onnx into $DEPS (~6 GB: PyPI torch brings CUDA libs)"
  mkdir -p "$DEPS"
  docker run --rm --entrypoint bash -v "$DEPS":/deps "$IMAGE" -c '
    pip install --break-system-packages -q --root-user-action=ignore --target /deps ultralytics torch torchvision onnx onnxslim 2>&1 \
      | grep -v -E "requires|incompatible|dependency resolver" ;
    [ -d /deps/ultralytics ] && [ -d /deps/onnx ] &&
    cd /deps && rm -rf numpy numpy.libs numpy-* packaging packaging-* &&
    chmod -R a+rwX /deps' || die "could not install ultralytics/torch/onnx (internet?)"
fi
docker run --rm --entrypoint bash -e YOLO_OFFLINE=1 -e YOLO_AUTOINSTALL=False \
  -v "$DEPS":/deps:ro -v "$WORK":/work "$IMAGE" -c "
    cd /work && PYTHONPATH=/deps python3 - <<'PY'
import json
from ultralytics import YOLO
m = YOLO('model.pt')
m.export(format='onnx', imgsz=[$H, $W], device='cpu', batch=1, dynamic=False, simplify=True)
json.dump({'names': {int(k): v for k, v in m.names.items()}, 'task': m.task}, open('meta.json', 'w'))
PY
    chmod a+rw /work/*" > "$WORK/export.log" 2>&1 \
  || { tail -20 "$WORK/export.log"; die "ONNX export failed (log above)"; }
[ -f "$WORK/model.onnx" ] || { tail -20 "$WORK/export.log"; die "no ONNX written"; }

echo "  2/3 trtexec (host GPU, FP16)"
"$TRTEXEC" --onnx="$WORK/model.onnx" --saveEngine="$WORK/model.engine" --fp16 \
  --memPoolSize=workspace:2048 > "$WORK/trtexec.log" 2>&1 \
  || { grep -E "\[E\]|Error" "$WORK/trtexec.log" | tail -10; die "trtexec failed (errors above)"; }
grep -E "GPU Compute Time" "$WORK/trtexec.log" | sed 's/^.*\] /      /'

echo "  3/3 header"
python3 - "$WORK" "$OUT" "$TRTV" "$H" "$W" "$STEM" <<'PY'
import json, struct, sys, time
work, out, trtv, h, w, stem = sys.argv[1:]
meta = json.load(open(f"{work}/meta.json"))
meta.update(imgsz=[int(h), int(w)], stride=32, batch=1, half=True, trt=trtv,
            source=f"{stem}.pt", built=time.strftime("%Y-%m-%d %H:%M"))
head = json.dumps(meta).encode()
with open(out, "wb") as f:
    f.write(struct.pack("<i", len(head))); f.write(head); f.write(open(f"{work}/model.engine", "rb").read())
PY
echo "  ✓ $OUT ($(du -h "$OUT" | cut -f1))"
echo "    ./rover detect picks it up (or ./rover up)."
