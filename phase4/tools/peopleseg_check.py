#!/usr/bin/env python3
"""peopleseg_check.py -- B5 (ISAAC_ROS_5_UPGRADE.md): does NVIDIA's PeopleSemSegNet ShuffleSeg find
the people our YOLO finds, and what does it cost on this Orin? MOVES NOTHING, reads a bag.

    python3 peopleseg_check.py <bag dir> [--every 15] [--max 400]
    (in orin-nav:2.1, with /models = ~/rover/models mounted)

For every Nth compressed colour frame: YOLO (our TensorRT engine) -> person boxes; ShuffleSeg
(model.plan from isaac_ros_peoplesemseg_models_install) -> a person mask at 960x544. Reports:
  recall   share of YOLO person boxes (conf >= 0.5) with >= 20 % of the box marked person
  extra    share of mask pixels outside every YOLO person box (frames with no person: false marks)
  time     per-frame GPU+copy time of the segmentation, and YOLO's, on the same frames
Preprocessing as NVIDIA's unet launch for this model: RGB, 960x544 stretch, (x/255 - 0.5)/0.5.
"""
import argparse
import ctypes
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, '/opt/rover4/nodes')
from yolo_backend import load_detector  # noqa: E402

SEG = '/models/isaac/isaac_ros_assets/models/peoplesemsegnet/optimized_deployable_shuffleseg_unet_amr_v1.0/1/model.plan'
W, H = 960, 544


class Seg:
    def __init__(self, path):
        import tensorrt as trt
        self.cu = ctypes.CDLL('libcudart.so.13')
        self.eng = trt.Runtime(trt.Logger(trt.Logger.WARNING)).deserialize_cuda_engine(open(path, 'rb').read())
        self.ctx = self.eng.create_execution_context()
        self.hin = np.empty((1, H, W, 3), np.float32)
        # the dtype from the ENGINE: it is INT64 although config.pbtxt says INT32 (a half-size
        # buffer = CUDA error 700, 2026-10-25)
        self.hout = np.empty((1, H, W, 1), trt.nptype(self.eng.get_tensor_dtype('argmax_1')))
        self.din, self.dout, self.st = ctypes.c_void_p(), ctypes.c_void_p(), ctypes.c_void_p()
        self.cu.cudaMalloc(ctypes.byref(self.din), ctypes.c_size_t(self.hin.nbytes))
        self.cu.cudaMalloc(ctypes.byref(self.dout), ctypes.c_size_t(self.hout.nbytes))
        self.cu.cudaStreamCreate(ctypes.byref(self.st))
        self.ctx.set_input_shape('input_2', (1, H, W, 3))   # built with a shape profile
        self.ctx.set_tensor_address('input_2', self.din.value)
        self.ctx.set_tensor_address('argmax_1', self.dout.value)

    def mask(self, bgr):
        rgb = cv2.cvtColor(cv2.resize(bgr, (W, H)), cv2.COLOR_BGR2RGB)
        self.hin[0] = (rgb.astype(np.float32) / 255.0 - 0.5) / 0.5
        self.cu.cudaMemcpyAsync(self.din, self.hin.ctypes.data_as(ctypes.c_void_p), ctypes.c_size_t(self.hin.nbytes), 1, self.st)
        self.ctx.execute_async_v3(self.st.value)
        self.cu.cudaMemcpyAsync(self.hout.ctypes.data_as(ctypes.c_void_p), self.dout, ctypes.c_size_t(self.hout.nbytes), 2, self.st)
        self.cu.cudaStreamSynchronize(self.st)
        return self.hout[0, :, :, 0] == 1


def frames(bag, every, cap):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import CompressedImage
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('cdr', 'cdr'))
    i = n = 0
    while r.has_next() and n < cap:
        t, d, _ = r.read_next()
        if t != '/camera/color/image_raw/compressed':
            continue
        i += 1
        if i % every:
            continue
        m = deserialize_message(d, CompressedImage)
        img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            n += 1
            yield img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--every', type=int, default=15)
    ap.add_argument('--max', type=int, default=400)
    ap.add_argument('--save', default='', help='write overlays of frames with people here')
    a = ap.parse_args()
    yolo = load_detector('/models/yolov8n_640x384_trt10.16.2.engine')
    seg = Seg(SEG)
    boxes = hit = 0
    empty_px = empty_frames = with_people = 0
    ts, ty, n = [], [], 0
    for img in frames(a.bag, a.every, a.max):
        n += 1
        t0 = time.perf_counter()
        dets = [d for d in yolo.detect(img) if d[0] == 'person' and d[1] >= 0.5]
        t1 = time.perf_counter()
        m = seg.mask(img)
        t2 = time.perf_counter()
        ty.append(t1 - t0)
        ts.append(t2 - t1)
        sx, sy = W / img.shape[1], H / img.shape[0]
        inside = np.zeros_like(m)
        for _, conf, (x1, y1, x2, y2) in dets:
            u1, v1, u2, v2 = int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy)
            inside[v1:v2, u1:u2] = True
            boxes += 1
            if m[v1:v2, u1:u2].mean() >= 0.20:
                hit += 1
        if dets:
            with_people += 1
            if a.save and with_people <= 6:
                ov = cv2.resize(img, (W, H)).copy()
                ov[m] = (0.4 * ov[m] + [150, 150, 0]).astype(np.uint8)   # cyan: clothes are often red
                for _, _, (x1, y1, x2, y2) in dets:
                    cv2.rectangle(ov, (int(x1 * sx), int(y1 * sy)), (int(x2 * sx), int(y2 * sy)), (0, 255, 0), 2)
                cv2.imwrite(f'{a.save}/peopleseg_{with_people}.jpg', ov)
        else:
            empty_frames += 1
            empty_px += m.mean()
    ms = lambda v: f'{1000 * np.median(v[3:]):.1f} ms median, {1000 * np.percentile(v[3:], 95):.1f} p95'  # noqa: E731
    print(f'frames {n} ({with_people} with a YOLO person, {empty_frames} without)')
    print(f'recall: {hit}/{boxes} YOLO person boxes are >= 20 % marked person' + (f' = {100 * hit / boxes:.0f} %' if boxes else ''))
    print(f'false marks: frames without a person have {100 * empty_px / max(1, empty_frames):.2f} % of pixels marked person (mean)')
    print(f'time: segmentation {ms(ts)}; YOLO {ms(ty)} (same frames, CPU wall incl. copies)')


if __name__ == '__main__':
    main()
