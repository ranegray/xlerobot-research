"""Detect a colored object in the RealSense color stream and publish its 3D pose.

Pipeline:
    /camera/color/image_raw        ─┐
    /camera/depth/image_rect_raw    ┼─> ApproximateTimeSynchronizer
    /camera/depth/camera_info      ─┘   ├─> HSV color blob detection
                                            ├─> back-project largest blob
                                            │   centroid via depth + intrinsics
                                            ├─> TF lookup camera_depth_optical_frame
                                            │   -> output_frame
                                            └─> publish:
                                                /detected_object/pose      (PoseStamped)
                                                /detected_object/marker    (Marker)
                                                /detected_object/image     (annotated)

Pose orientation is identity (single-blob detection has no orientation info).
The PoseStamped header.frame_id is the configured output_frame so consumers
can ignore TF entirely.

Parameters:
    target_color: red | green | blue (default: red)
    min_area_pixels: int (default: 200)
    output_frame: str (default: world)
    publish_marker: bool (default: true)
    publish_debug_image: bool (default: true)
    depth_window_px: int (default: 3)
    min_depth_valid_pixels: int (default: 8)
    min_depth_valid_fraction: float (default: 0.35)
    min_depth_m: float (default: 0.05)
    max_depth_m: float (default: 1.0)
    max_depth_std_m: float (default: 0.02)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_pose
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)
from visualization_msgs.msg import Marker

from xle_perception.color_detector import ColorDetector


CAMERA_OPTICAL_FRAME = "camera_depth_optical_frame"
# Unaligned depth has been more reliable on the Jetson than aligned_depth.
# Using it means the point lives in the depth optical frame and color/depth
# pixels are only approximately matched.


@dataclass(frozen=True)
class DepthSample:
    depth_m: float
    valid_pixels: int
    total_pixels: int
    valid_fraction: float
    std_m: float


class ColorObjectDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("color_object_detector_node")

        self.declare_parameter("target_color", "red")
        self.declare_parameter("min_area_pixels", 200)
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("publish_marker", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("depth_window_px", 3)
        self.declare_parameter("min_depth_valid_pixels", 8)
        self.declare_parameter("min_depth_valid_fraction", 0.35)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 1.0)
        self.declare_parameter("max_depth_std_m", 0.02)

        self._target_color = str(self.get_parameter("target_color").value)
        self._output_frame = str(self.get_parameter("output_frame").value)
        self._publish_marker = bool(self.get_parameter("publish_marker").value)
        self._publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self._depth_window_px = int(self.get_parameter("depth_window_px").value)
        self._min_depth_valid_pixels = int(self.get_parameter("min_depth_valid_pixels").value)
        self._min_depth_valid_fraction = float(
            self.get_parameter("min_depth_valid_fraction").value
        )
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._max_depth_std_m = float(self.get_parameter("max_depth_std_m").value)
        min_area = int(self.get_parameter("min_area_pixels").value)

        self._detector = ColorDetector.from_color(self._target_color, min_area=min_area)
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Match RealSense sensor QoS.
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._color_sub = Subscriber(self, Image, "/camera/color/image_raw", qos_profile=sensor_qos)
        self._depth_sub = Subscriber(
            self, Image, "/camera/depth/image_rect_raw", qos_profile=sensor_qos
        )
        self._caminfo_sub = Subscriber(
            self, CameraInfo, "/camera/depth/camera_info", qos_profile=sensor_qos
        )

        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub, self._caminfo_sub],
            queue_size=10,
            slop=0.05,
        )
        self._sync.registerCallback(self._on_synced)

        self._pose_pub = self.create_publisher(PoseStamped, "/detected_object/pose", 10)
        if self._publish_marker:
            self._marker_pub = self.create_publisher(Marker, "/detected_object/marker", 10)
        if self._publish_debug_image:
            self._debug_image_pub = self.create_publisher(Image, "/detected_object/image", 10)

        self.get_logger().info(
            f"color_object_detector ready: target={self._target_color!r} "
            f"min_area={min_area} output_frame={self._output_frame!r} "
            f"depth_window={self._depth_window_px}px "
            f"min_valid={self._min_depth_valid_pixels}/"
            f"{self._min_depth_valid_fraction:.2f} "
            f"depth_range=[{self._min_depth_m:.2f}, {self._max_depth_m:.2f}]m "
            f"max_depth_std={self._max_depth_std_m:.3f}m"
        )

    def _on_synced(self, color_msg: Image, depth_msg: Image, caminfo_msg: CameraInfo) -> None:
        try:
            self._process_synced(color_msg, depth_msg, caminfo_msg)
        except Exception as exc:  # noqa: BLE001
            # Keep one bad frame from killing the node.
            self.get_logger().error(f"unhandled error in detection callback: {exc!r}")

    def _process_synced(self, color_msg: Image, depth_msg: Image, caminfo_msg: CameraInfo) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"cv_bridge conversion failed: {exc}")
            return

        detections = self._detector.detect(bgr)
        if not detections:
            if self._publish_debug_image:
                self._publish_debug(bgr, None)
            return

        det = detections[0]  # largest
        cx_px, cy_px = det.centroid

        # Reject weak depth samples at blob edges/background.
        depth_sample, depth_reject_reason = self._sample_depth(
            depth, cx_px, cy_px, window=self._depth_window_px
        )
        if depth_sample is None:
            self.get_logger().info(
                f"rejected depth at centroid ({cx_px}, {cy_px}): "
                f"{depth_reject_reason}; skipping."
            )
            if self._publish_debug_image:
                self._publish_debug(bgr, det, depth_m=None)
            return

        depth_m = depth_sample.depth_m

        # Back-project into camera_depth_optical_frame.
        fx = caminfo_msg.k[0]
        fy = caminfo_msg.k[4]
        cx_int = caminfo_msg.k[2]
        cy_int = caminfo_msg.k[5]
        x = (cx_px - cx_int) * depth_m / fx
        y = (cy_px - cy_int) * depth_m / fy
        z = depth_m

        pose_in_camera = PoseStamped()
        pose_in_camera.header.stamp = color_msg.header.stamp
        pose_in_camera.header.frame_id = CAMERA_OPTICAL_FRAME
        pose_in_camera.pose.position.x = x
        pose_in_camera.pose.position.y = y
        pose_in_camera.pose.position.z = z
        pose_in_camera.pose.orientation.w = 1.0  # identity; no orientation from blob

        pose_out = self._transform_pose(pose_in_camera, self._output_frame)
        if pose_out is None:
            return

        self._pose_pub.publish(pose_out)
        if self._publish_marker:
            self._publish_marker_msg(pose_out, det)
        if self._publish_debug_image:
            self._publish_debug(bgr, det, depth_m=depth_m, pose_out=pose_out)

    def _sample_depth(
        self, depth_image: np.ndarray, cx: int, cy: int, window: int = 3
    ) -> Tuple[Optional[DepthSample], str]:
        """Median depth in a (2*window+1)^2 window plus local sanity gates."""
        h, w = depth_image.shape[:2]
        x0, x1 = max(0, cx - window), min(w, cx + window + 1)
        y0, y1 = max(0, cy - window), min(h, cy + window + 1)
        patch = depth_image[y0:y1, x0:x1]
        if patch.size == 0:
            return None, "empty depth patch"

        finite = patch[np.isfinite(patch)]
        valid = finite[finite > 0]
        if valid.size == 0:
            return None, "no positive finite depth pixels"

        if np.issubdtype(depth_image.dtype, np.integer):
            valid_m = valid.astype(np.float32) * 0.001
        else:
            valid_m = valid.astype(np.float32)

        valid_fraction = float(valid_m.size / patch.size)
        if valid_m.size < self._min_depth_valid_pixels:
            return (
                None,
                f"only {valid_m.size} valid pixels; "
                f"need {self._min_depth_valid_pixels}",
            )
        if valid_fraction < self._min_depth_valid_fraction:
            return (
                None,
                f"valid fraction {valid_fraction:.2f}; "
                f"need {self._min_depth_valid_fraction:.2f}",
            )

        depth_m = float(np.median(valid_m))
        if depth_m < self._min_depth_m or depth_m > self._max_depth_m:
            return (
                None,
                f"median depth {depth_m:.3f}m outside "
                f"[{self._min_depth_m:.3f}, {self._max_depth_m:.3f}]m",
            )

        std_m = float(np.std(valid_m))
        if std_m > self._max_depth_std_m:
            return (
                None,
                f"depth std {std_m:.3f}m exceeds "
                f"max_depth_std_m {self._max_depth_std_m:.3f}m",
            )

        return (
            DepthSample(
                depth_m=depth_m,
                valid_pixels=int(valid_m.size),
                total_pixels=int(patch.size),
                valid_fraction=valid_fraction,
                std_m=std_m,
            ),
            "ok",
        )

    def _transform_pose(
        self, pose: PoseStamped, target_frame: str
    ) -> Optional[PoseStamped]:
        # Use latest TF; exact image stamps often land a few ms past /tf.
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warning(
                f"TF lookup failed {pose.header.frame_id} -> {target_frame}: {exc}"
            )
            return None
        out = do_transform_pose(pose.pose, tf)
        out_stamped = PoseStamped()
        out_stamped.header.stamp = pose.header.stamp
        out_stamped.header.frame_id = target_frame
        out_stamped.pose = out
        return out_stamped

    def _publish_marker_msg(self, pose: PoseStamped, det) -> None:
        m = Marker()
        m.header = pose.header
        m.ns = "detected_object"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.r = 1.0 if self._target_color == "red" else 0.2
        m.color.g = 1.0 if self._target_color == "green" else 0.2
        m.color.b = 1.0 if self._target_color == "blue" else 0.2
        m.color.a = 0.8
        m.lifetime.sec = 1
        self._marker_pub.publish(m)

    def _publish_debug(
        self,
        bgr: np.ndarray,
        det,
        depth_m: Optional[float] = None,
        pose_out: Optional[PoseStamped] = None,
    ) -> None:
        annotated = bgr.copy()
        if det is not None:
            x, y, w, h = det.bbox
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(annotated, det.centroid, 4, (0, 0, 255), -1)
            label_lines = [
                f"area={int(det.area)}",
                f"depth={depth_m:.3f}m" if depth_m is not None else "depth=?",
            ]
            if pose_out is not None:
                label_lines.append(
                    f"({pose_out.header.frame_id}): "
                    f"({pose_out.pose.position.x:+.2f}, "
                    f"{pose_out.pose.position.y:+.2f}, "
                    f"{pose_out.pose.position.z:+.2f})"
                )
            for i, line in enumerate(label_lines):
                cv2.putText(
                    annotated,
                    line,
                    (x, max(0, y - 10 - 18 * i)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
        out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self._debug_image_pub.publish(out_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorObjectDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
