#!/usr/bin/env python3
"""D555 depth image -> strided PointCloud2, as nav2 collision_monitor's obstacle source.

WHY THIS EXISTS (2026-08-10)
----------------------------
collision_monitor needs a pointcloud/scan source. It was pointed at nvblox's
`back_projected_depth/...`, but nvblox treats that as a DEBUG VISUALISATION topic:
measured here it ran at ~0.5 Hz with gaps up to 7.3 s. The monitor's contract is
fail-safe — when its source goes stale it logs

    "Robot to stop due to invalid source"

and HOLDS THE ROBOT AT ZERO. So the rover was being pinned in place by its own
safety layer while nav2 planned happily and every goal died on "Failed to make
progress". Raising source_timeout only swaps that for navigating on 7-second-old
obstacle data, which is worse.

This node publishes a real, fast source straight off the 21 Hz depth stream:
stride the depth image, deproject with the camera intrinsics, emit XYZ in the
depth optical frame. collision_monitor transforms to base_link and applies its own
min_height/max_height band, so no filtering beyond range validity is done here.

Cost is deliberately tiny (the Orin is perception-bound): one strided numpy
deprojection per published frame. At stride 8 on 896x504 that is ~7k points.

    depth_to_cloud.py --ros-args -p stride:=8 -p rate:=10.0
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthToCloud(Node):
    def __init__(self):
        super().__init__("depth_to_cloud")
        self.declare_parameter("depth_topic", "/camera/camera0/depth/image_rect_raw")
        self.declare_parameter("info_topic", "/camera/camera0/depth/camera_info")
        self.declare_parameter("output_topic", "/perception/depth_points")
        # Every Nth pixel in both axes. Obstacle detection needs coverage, not
        # resolution — collision_monitor's min_points is only 6.
        self.declare_parameter("stride", 8)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("min_range_m", 0.20)
        self.declare_parameter("max_range_m", 4.0)

        p = lambda n: self.get_parameter(n).value
        self._stride = max(1, int(p("stride")))
        self._min_r = float(p("min_range_m"))
        self._max_r = float(p("max_range_m"))
        self._depth = None
        self._info = None
        self._uv = None          # cached pixel grid (depends on shape + stride)
        self._uv_shape = None

        self.create_subscription(Image, p("depth_topic"),
                                 lambda m: setattr(self, "_depth", m),
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p("info_topic"),
                                 lambda m: setattr(self, "_info", m),
                                 qos_profile_sensor_data)
        self._pub = self.create_publisher(PointCloud2, p("output_topic"),
                                          qos_profile_sensor_data)
        self.create_timer(1.0 / float(p("rate")), self._tick)
        self._warned = False
        self.get_logger().info(
            f"depth_to_cloud: {p('depth_topic')} -> {p('output_topic')} "
            f"@ {p('rate')} Hz, stride {self._stride}")

    def _decode(self, m):
        buf = np.frombuffer(bytes(m.data), dtype=np.uint8)
        if m.encoding == "16UC1":
            d = buf.view(np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
            return d.astype(np.float32) / 1000.0        # mm -> m
        if m.encoding == "32FC1":
            return buf.view(np.float32).reshape(m.height, m.step // 4)[:, :m.width]
        return None

    def _tick(self):
        m, ci = self._depth, self._info
        if m is None or ci is None:
            return
        dep = self._decode(m)
        if dep is None:
            if not self._warned:
                self.get_logger().warning(f"unsupported depth encoding {m.encoding}")
                self._warned = True
            return

        s = self._stride
        sub = dep[::s, ::s]
        if self._uv_shape != sub.shape:
            vv, uu = np.mgrid[0:dep.shape[0]:s, 0:dep.shape[1]:s]
            self._uv = (uu.astype(np.float32), vv.astype(np.float32))
            self._uv_shape = sub.shape
        uu, vv = self._uv

        valid = (sub > self._min_r) & (sub < self._max_r)
        if not valid.any():
            # Publish an EMPTY cloud rather than nothing: an empty frame is fresh
            # data meaning "nothing in range", which keeps collision_monitor's
            # source valid. Silence would trip its stale-source hard stop.
            self._publish(m, np.empty((0, 3), dtype=np.float32))
            return

        z = sub[valid]
        u = uu[valid]
        v = vv[valid]
        fx, fy, cx, cy = ci.k[0], ci.k[4], ci.k[2], ci.k[5]
        pts = np.empty((z.size, 3), dtype=np.float32)
        pts[:, 0] = (u - cx) / fx * z      # optical frame: x right
        pts[:, 1] = (v - cy) / fy * z      #                y down
        pts[:, 2] = z                      #                z forward
        self._publish(m, pts)

    def _publish(self, src, pts):
        msg = PointCloud2()
        msg.header.stamp = src.header.stamp
        msg.header.frame_id = src.header.frame_id
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * pts.shape[0]
        msg.is_dense = True
        msg.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(DepthToCloud())


if __name__ == "__main__":
    main()
