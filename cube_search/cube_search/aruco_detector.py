#!/usr/bin/python3
from collections import deque
from time import monotonic

import cv2 as cv
import numpy as np
import rclpy

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener

from cube_search_interfaces.msg import (
    MarkerDetection,
    MarkerDetectionArray,
)


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("marker_length", 0.07)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("show_image", True)
        self.declare_parameter("tf_wait_timeout", 1.0)
        self.declare_parameter("optical_frame", "camera_rgb_optical_frame")

        self.marker_length = float(
            self.get_parameter("marker_length").value
        )
        self.target_frame = self.get_parameter("target_frame").value
        self.show_image = self.get_parameter("show_image").value
        self.tf_wait_timeout = float(
            self.get_parameter("tf_wait_timeout").value
        )
        self.optical_frame = self.get_parameter("optical_frame").value

        self.calibration = None
        self.pending = deque()
        self.max_pending_batches = 60

        self.bridge = CvBridge()

        self.aruco_dict = cv.aruco.Dictionary_get(
            cv.aruco.DICT_ARUCO_ORIGINAL
        )
        self.detector_parameters = cv.aruco.DetectorParameters_create()
        self.detector_parameters.cornerRefinementMethod = (
            cv.aruco.CORNER_REFINE_SUBPIX
        )

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detections_pub = self.create_publisher(
            MarkerDetectionArray,
            "/aruco/detections",
            10,
        )

        self.info_subscription = self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.info_callback,
            qos_profile_sensor_data,
        )

        self.image_subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.camera_callback,
            qos_profile_sensor_data,
        )

        # Retry batches whose timestamped TF was not available immediately.
        self.tf_timer = self.create_timer(0.05, self.publish_pending)

        if self.show_image:
            cv.namedWindow("camera_image")

        self.get_logger().info(
            f"Marker side: {self.marker_length:.3f} m; "
            f"publishing detections in {self.target_frame}"
        )

    def info_callback(self, msg: CameraInfo):
        self.calibration = msg
        
        
    def make_marker_pose(self, rvec, tvec, header):
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = self.optical_frame

        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])

        # Rodrigues rotation vector -> quaternion.
        # np.sinc handles the zero-angle case without division by zero.
        vector = np.asarray(rvec, dtype=float).reshape(3)
        angle = np.linalg.norm(vector)

        xyz = vector * (0.5 * np.sinc(angle / (2.0 * np.pi)))

        pose.pose.orientation.x = float(xyz[0])
        pose.pose.orientation.y = float(xyz[1])
        pose.pose.orientation.z = float(xyz[2])
        pose.pose.orientation.w = float(np.cos(angle / 2.0))

        return pose
    
    def camera_callback(self, msg: Image):
        info = self.calibration

        if info is None:
            self.get_logger().info(
                "Waiting for camera calibration.",
                throttle_duration_sec=5.0,
            )
            return


        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as error:
            self.get_logger().warning(
                str(error),
                throttle_duration_sec=5.0,
            )
            return

        K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        D = np.asarray(info.d, dtype=np.float64)

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        corners, ids, _ = cv.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_parameters,
        )

        batch = []

        if ids is not None:
            cv.aruco.drawDetectedMarkers(image, corners, ids)

            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                K,
                D,
            )

            for marker_id, rvec, tvec in zip(
                ids.flatten(), rvecs, tvecs
            ):
                tvec = tvec.reshape(3)

                if (
                    not np.isfinite(rvec).all()
                    or not np.isfinite(tvec).all()
                    or tvec[2] <= 0
                ):
                    continue

                pose = self.make_marker_pose(
                    rvec,
                    tvec,
                    msg.header,
                )

                batch.append((int(marker_id), pose))

                if self.show_image:
                    cv.drawFrameAxes(
                        image,
                        K,
                        D,
                        rvec,
                        tvec,
                        self.marker_length * 0.5,
                    )

        if batch:
            if len(self.pending) >= self.max_pending_batches:
                self.pending.popleft()
                self.get_logger().warning(
                    "TF queue full; dropped oldest detection batch.",
                    throttle_duration_sec=5.0,
                )

            self.pending.append((monotonic(), batch))
            self.publish_pending()

        if self.show_image:
            cv.imshow("camera_image", image)
            cv.waitKey(1)
            
    def publish_pending(self):
        for _ in range(len(self.pending)):
            received, batch = self.pending.popleft()
            header = batch[0][1].header

            try:
                # Nonblocking lookup at the original image timestamp.
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    header.frame_id,
                    Time.from_msg(header.stamp),
                )

            except TransformException as error:
                if monotonic() - received < self.tf_wait_timeout:
                    self.pending.append((received, batch))
                else:
                    self.get_logger().warning(
                        f"Dropped detections waiting for TF: {error}",
                        throttle_duration_sec=5.0,
                    )
                continue

            output = MarkerDetectionArray()
            output.header.frame_id = self.target_frame
            output.header.stamp = header.stamp

            for marker_id, camera_pose in batch:
                map_pose = do_transform_pose_stamped(
                    camera_pose,
                    transform,
                )

                detection = MarkerDetection()
                detection.id = marker_id
                detection.pose = map_pose.pose

                output.detections.append(detection)

            self.detections_pub.publish(output)
        
def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ArucoDetectorNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            if node.show_image:
                cv.destroyAllWindows()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()