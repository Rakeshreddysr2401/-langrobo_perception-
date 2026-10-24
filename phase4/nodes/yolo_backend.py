"""yolo_backend.py — run YOLO for detections_3d: TensorRT when it can, PyTorch when it must.

    det = load_detector("/models/yolov8n_640x384_trt10.16.2.engine",
                        fallback="/models/yolov8n.pt", log=node.get_logger())
    det.detect(bgr) -> [(label, conf, (x1, y1, x2, y2)), ...]   # input-image pixels, best first
    det.backend     -> "tensorrt 10.16.2.10 ..." | "pytorch ..."  (said in the log)

WHY (2026-10-21, CLOSED_LOOP_PLAN.md stage 1). The detector ran YOLOv8n through
PyTorch on the GPU: 35 ms and 33 ms of CPU per frame (the CPU is busy for the
whole inference -- PyTorch's per-layer launch overhead, not GPU work) and
1.4 GB of RAM for the PyTorch/CUDA context. Measured on 300 frames of the
2026-10-18 house survey, same 640 input:

    PyTorch .pt          35.4 ms  CPU 33.5 ms  1405 MB  (today)
    Ultralytics+engine   12.2 ms  CPU  9.4 ms  1455 MB  (torch still loaded)
    this TensorRT runner 11.6 ms  CPU 11.3 ms   644 MB  <- the default

Same objects: 91 of the .pt's 94, the 3 missed and 1 extra all at exactly the
confidence cut (FP16 rounding); matched confidences differ by 0.004 on
average. A 448x256 engine (the camera's own size) was 7 ms but found only 5 of
18 of the .pt's objects -- the model wants the 640 upscale. Keep 640.

TensorRT here is the HOST's (JetPack 7.2, 10.16): the orin-nav:1.2 image's own
10.13 predates Orin and refuses SM 87 ("Target GPU SM 87 is not supported").
./rover mounts the host's three runtime libraries and its Python bindings at
/hosttrt and points ONLY this process at them (the image is never modified).
An engine is tied to the exact TensorRT version that built it, so the version
is in its file name and its header; after a JetPack update the engine is
refused here, the .pt is used instead, and the log says to run
`./rover engine`. A detector always comes up.

The PyTorch path is unchanged from before, imported only when used, so the
TensorRT path never pays for torch.

Thresholds match what the .pt path really did: Ultralytics' own defaults,
conf 0.25 (a floor under detections_3d's person_low_conf 0.2 -- weak people
between 0.20 and 0.25 never reached the tracker on either path), class-aware
NMS at IoU 0.7, at most 300 boxes.
"""

import ctypes
import json
import struct

import cv2
import numpy as np

CONF_FLOOR = 0.25   # Ultralytics' predict default -- what the .pt path returned
NMS_IOU = 0.7       # Ultralytics' predict default
MAX_DET = 300


def read_engine_file(path):
    """(metadata dict or {}, raw engine bytes). The header is Ultralytics' own
    layout (int32 length + JSON), so either loader can read the same file."""
    raw = open(path, "rb").read()
    n = struct.unpack("<i", raw[:4])[0]
    if 0 < n < 1_000_000:
        try:
            return json.loads(raw[4:4 + n].decode()), raw[4 + n:]
        except (UnicodeDecodeError, ValueError):
            pass
    return {}, raw


class TrtYolo:
    """YOLOv8-style detect head ((1, 4 + classes, N) output) on TensorRT, no torch."""

    def __init__(self, path):
        # orin-nav:1.2: the image's TensorRT 10.13 refuses this Orin (SM 87), so ./rover detect
        # points PYTHONPATH at the host's (/hosttrt). orin-nav:2.0 ships 10.16, the host's own
        # version: its TensorRT is used directly.
        import tensorrt as trt
        meta, raw = read_engine_file(path)
        if trt.__version__.startswith("10.13.") and not trt.__file__.startswith("/hosttrt/"):
            raise RuntimeError(f"this process has the image's TensorRT {trt.__version__}, which "
                               f"cannot run on this Orin; the host's is not mounted at /hosttrt "
                               f"(a container from an older ./rover: ./rover stop, then ./rover up)")
        built_with = meta.get("trt")
        if built_with and built_with != trt.__version__:
            raise RuntimeError(f"{path} was built with TensorRT {built_with}, the host now has "
                               f"{trt.__version__} (a JetPack update): rebuild it with ./rover engine")
        self._cudart = ctypes.CDLL("libcudart.so.13")
        self._log = trt.Logger(trt.Logger.WARNING)
        self.engine = trt.Runtime(self._log).deserialize_cuda_engine(raw)
        if self.engine is None:
            raise RuntimeError(f"TensorRT {trt.__version__} could not load {path}")
        self.ctx = self.engine.create_execution_context()
        io = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        inp = [t for t in io if self.engine.get_tensor_mode(t) == trt.TensorIOMode.INPUT]
        out = [t for t in io if self.engine.get_tensor_mode(t) == trt.TensorIOMode.OUTPUT]
        if len(inp) != 1 or len(out) != 1:
            raise RuntimeError(f"{path}: expected one input and one output (a detect model), "
                               f"got {len(inp)} and {len(out)}")
        self._in_name, self._out_name = inp[0], out[0]
        in_shape = tuple(self.engine.get_tensor_shape(self._in_name))     # (1, 3, H, W)
        out_shape = tuple(self.engine.get_tensor_shape(self._out_name))   # (1, 4 + C, N)
        self.h, self.w = int(in_shape[2]), int(in_shape[3])
        self._host_in = np.empty(in_shape, trt.nptype(self.engine.get_tensor_dtype(self._in_name)))
        self._host_out = np.empty(out_shape, trt.nptype(self.engine.get_tensor_dtype(self._out_name)))
        names = meta.get("names") or {i: str(i) for i in range(out_shape[1] - 4)}
        self.names = {int(k): str(v).lower() for k, v in names.items()}
        if len(self.names) != out_shape[1] - 4:
            raise RuntimeError(f"{path}: {len(self.names)} names for {out_shape[1] - 4} classes")
        self._d_in, self._d_out, self._stream = ctypes.c_void_p(), ctypes.c_void_p(), ctypes.c_void_p()
        self._ck(self._cudart.cudaMalloc(ctypes.byref(self._d_in), ctypes.c_size_t(self._host_in.nbytes)), "cudaMalloc")
        self._ck(self._cudart.cudaMalloc(ctypes.byref(self._d_out), ctypes.c_size_t(self._host_out.nbytes)), "cudaMalloc")
        self._ck(self._cudart.cudaStreamCreate(ctypes.byref(self._stream)), "cudaStreamCreate")
        self.ctx.set_tensor_address(self._in_name, self._d_in.value)
        self.ctx.set_tensor_address(self._out_name, self._d_out.value)
        self.backend = f"tensorrt {trt.__version__} {self.w}x{self.h} ({path})"

    @staticmethod
    def _ck(err, what):
        if err != 0:
            raise RuntimeError(f"{what} failed (CUDA error {err})")

    def _letterbox(self, img):
        """Ultralytics' letterbox: scale to fit, centre, pad with grey 114."""
        ih, iw = img.shape[:2]
        r = min(self.h / ih, self.w / iw)
        nh, nw = round(ih * r), round(iw * r)
        top, left = (self.h - nh) // 2, (self.w - nw) // 2
        canvas = np.full((self.h, self.w, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = (
            img if (nh, nw) == (ih, iw) else cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR))
        return canvas, r, left, top

    def detect(self, bgr):
        canvas, r, left, top = self._letterbox(bgr)
        # one C++ call: 0..1, BGR -> RGB, HWC -> NCHW float32
        self._host_in[...] = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, swapRB=True)
        rt, s = self._cudart, self._stream
        self._ck(rt.cudaMemcpyAsync(self._d_in, self._host_in.ctypes.data_as(ctypes.c_void_p),
                                    ctypes.c_size_t(self._host_in.nbytes), 1, s), "copy in")
        if not self.ctx.execute_async_v3(s.value):
            raise RuntimeError("TensorRT inference failed")
        self._ck(rt.cudaMemcpyAsync(self._host_out.ctypes.data_as(ctypes.c_void_p), self._d_out,
                                    ctypes.c_size_t(self._host_out.nbytes), 2, s), "copy out")
        self._ck(rt.cudaStreamSynchronize(s), "sync")
        return decode(self._host_out[0], r, left, top, bgr.shape[1], bgr.shape[0], self.names)


def decode(out, r, left, top, iw, ih, names, conf_floor=CONF_FLOOR, iou=NMS_IOU, max_det=MAX_DET):
    """(4 + C, N) YOLOv8 head -> [(label, conf, (x1, y1, x2, y2))] in the original image."""
    p = out.astype(np.float32).T                       # (N, 4 + C): cx, cy, w, h, scores
    scores = p[:, 4:]
    cls = scores.argmax(1)
    conf = scores[np.arange(len(cls)), cls]
    keep = conf >= conf_floor
    if not keep.any():
        return []
    b, cls, conf = p[keep, :4], cls[keep], conf[keep]
    xyxy = np.empty_like(b)
    xyxy[:, 0] = (b[:, 0] - b[:, 2] / 2 - left) / r
    xyxy[:, 1] = (b[:, 1] - b[:, 3] / 2 - top) / r
    xyxy[:, 2] = (b[:, 0] + b[:, 2] / 2 - left) / r
    xyxy[:, 3] = (b[:, 1] + b[:, 3] / 2 - top) / r
    xyxy[:, [0, 2]] = xyxy[:, [0, 2]].clip(0, iw)
    xyxy[:, [1, 3]] = xyxy[:, [1, 3]].clip(0, ih)
    # Class-aware NMS as Ultralytics does it: shift each class's boxes far apart,
    # then one plain NMS (this image's OpenCV has no NMSBoxesBatched).
    off = cls[:, None].astype(np.float32) * 4096.0
    xywh = np.c_[xyxy[:, :2] + off, xyxy[:, 2:] - xyxy[:, :2]]
    idx = np.array(cv2.dnn.NMSBoxes(xywh.tolist(), conf.tolist(), conf_floor, iou), dtype=int).reshape(-1)
    idx = idx[np.argsort(-conf[idx])][:max_det]
    return [(names[int(cls[i])], float(conf[i]), tuple(float(v) for v in xyxy[i])) for i in idx]


class TorchYolo:
    """The original path: Ultralytics + PyTorch, imported only here."""

    def __init__(self, path, device="cuda"):
        import torch
        torch.backends.cudnn.enabled = False  # cuDNN version mismatch on Jetson (same as target_node)
        from ultralytics import YOLO
        self._model, self._device = YOLO(path), device
        self.backend = f"pytorch {torch.__version__} on {device} ({path})"

    def detect(self, bgr):
        found = []
        for r in self._model(bgr, device=self._device, verbose=False,
                             conf=CONF_FLOOR, iou=NMS_IOU, max_det=MAX_DET):
            for b in r.boxes:
                found.append((self._model.names[int(b.cls[0])].lower(), float(b.conf[0]),
                              tuple(float(v) for v in b.xyxy[0])))
        return found


def load_detector(model, fallback="", device="cuda", log=None):
    """TensorRT for a .engine, PyTorch for anything else; on a .engine that will
    not load, say why and use `fallback` -- the detector always comes up."""
    if str(model).endswith(".engine"):
        try:
            return TrtYolo(model)
        except Exception as e:  # noqa: BLE001 -- any failure means: fall back, loudly
            if not fallback:
                raise
            if log:
                log.error(f"TensorRT engine not usable: {e}. Falling back to {fallback} "
                          f"(~2x the CPU, +0.8 GB RAM).")
            model = fallback
    return TorchYolo(model, device)
