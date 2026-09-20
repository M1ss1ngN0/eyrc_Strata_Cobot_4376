#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1A of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          4376
# Theme:            Strata Cobot
# Author List:      Saksham Kumar Aggarwal
# Filename:         task1A.py
# Functions:        detect_ores, main
# Global variables: ore_types, color_topic, depth_topic, camera_info_topic,
#                   base_frame, TEAM_ID, DETECTION_IMAGE


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import CameraInfo, Image


##################### TASK CONSTANTS #######################

# Two ores of each type are spawned - six in all - told apart by an id of 1 or 2.
ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

# The RealSense topics. The depth image is ALIGNED to the colour image.
color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

# The parent frame every ore transform is published against.
base_frame = 'base_link'

# TEAM_ID: Numeric eYRC team identifier used in the required detection image filename.
TEAM_ID = '4376'
DETECTION_IMAGE = f'SC#{TEAM_ID}_task1A_detection.png'


##################### FUNCTION DEFINITIONS #######################

def detect_ores(image):
    '''
    Purpose:
    ---
    Detects ores by colour and returns their centre pixels and corresponding
    ore types. Valid ore contours are also drawn on the supplied image.

    Input Arguments:
    ---
    `image` :  [ numpy.ndarray ]
        BGR colour image from the camera.

    Returns:
    ---
    `center_ore_list` :  [ list ]
        List of centre pixel coordinates for the detected ores.

    `ore_type_list` :  [ list ]
        List of ore type names corresponding to the detected centres.

    Example call:
    ---
    `center_ore_list, ore_type_list = detect_ores(image)`
    '''

    center_ore_list = []
    ore_type_list = []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Thresholds validated against the Task 1A camera scene.
    colour_ranges = {
        'azurite_ore': (
            np.array([100, 80, 50]),
            np.array([130, 255, 255]),
            200
        ),
        'malachite_ore': (
            np.array([60, 150, 80]),
            np.array([90, 255, 255]),
            150
        ),
        'vanadinite_ore': (
            np.array([5, 220, 200]),
            np.array([15, 255, 255]),
            150
        )
    }

    for ore_type in ore_types:
        lower, upper, min_area = colour_ranges[ore_type]

        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        valid_contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= min_area
        ]

        # Exactly two ores of each type are expected. If unexpected extra
        # regions survive thresholding, retain the two largest candidates.
        valid_contours.sort(key=cv2.contourArea, reverse=True)
        valid_contours = valid_contours[:2]

        for contour in valid_contours:
            M = cv2.moments(contour)

            if M["m00"] == 0:
                continue

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            center_ore_list.append((cX, cY))
            ore_type_list.append(ore_type)

            # Draw the detected ore boundary during development/submission capture.
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

    return center_ore_list, ore_type_list


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    Purpose:
    ---
    Detects the six ores in the camera image, estimates their 3D positions,
    transforms those positions into the `base_link` frame, and continuously
    broadcasts one TF transform for each ore.

    Subscribes to:
    ---
    `/camera/camera/color/image_raw` : [ sensor_msgs/msg/Image ]
        Raw colour image used for ore detection.

    `/camera/camera/aligned_depth_to_color/image_raw` : [ sensor_msgs/msg/Image ]
        Depth image aligned with the colour image.

    `/camera/camera/color/camera_info` : [ sensor_msgs/msg/CameraInfo ]
        Camera intrinsic parameters used for 3D back-projection.

    Publishes to:
    ---
    `/tf` : [ tf2_msgs/msg/TFMessage ]
        Continuous transforms from `base_link` to each ore frame.
    '''

    def __init__(self):
        '''
        Purpose:
        ---
        Creates the Task 1A ROS 2 node, subscribes to the required camera
        topics, initializes the TF listener and broadcaster, and starts the
        periodic image-processing callback.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        `node = ore_tf()`
        '''

        super().__init__('ore_tf_publisher')                                            # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # Time interval between image-processing callbacks, in seconds.
        self.bridge = CvBridge()                                                        # CvBridge object used for ROS image message conversion.
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # TF buffer used to query the camera-to-base transform.
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # TF broadcaster used to publish ore transforms.
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # Periodic callback used to process the latest camera data.

        self.cv_image = None                                                            # Latest BGR colour image received from the colour callback.
        self.depth_image = None                                                         # Latest aligned depth image in metres.
        self.cam_info = None                                                            # Latest CameraInfo message containing the camera intrinsics.

        # Camera optical frame and intrinsics are populated from ROS topics.
        self.camera_frame = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ore_tracks: Persistent pixel-space identities for the two ores of each type.
        self.ore_tracks = {
            ore_type: [None, None]
            for ore_type in ore_types
        }




    def depthimagecb(self, data):
        '''
        Purpose:
        ---
        Receives the aligned depth image, converts it to a floating-point
        representation in metres, and stores it for processing.

        Input Arguments:
        ---
        `data` :  [ sensor_msgs/msg/Image ]
            Aligned depth image received from the camera.

        Returns:
        ---
        None

        Example call:
        ---
        `self.depthimagecb(data)`
        '''

        try:
            depth = self.bridge.imgmsg_to_cv2(
                data,
                desired_encoding='passthrough'
            )
        except CvBridgeError as e:
            print(f"Depth conversion failed: {e}")
            return

        # Task 1A publishes 32FC1 in metres. Handle 16UC1 defensively too.
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32, copy=False)

        self.depth_image = depth


    def colorimagecb(self, data):
        '''
        Purpose:
        ---
        Receives the raw colour image, converts it to BGR format, and stores
        the camera optical frame identifier.

        Input Arguments:
        ---
        `data` :  [ sensor_msgs/msg/Image ]
            Raw colour image received from the camera.

        Returns:
        ---
        None

        Example call:
        ---
        `self.colorimagecb(data)`
        '''

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(
                data,
                desired_encoding='bgr8'
            )
            self.camera_frame = data.header.frame_id
        except CvBridgeError as e:
            print(f"Colour image conversion failed: {e}")


    def caminfocb(self, data):
        '''
        Purpose:
        ---
        Receives CameraInfo and stores the camera intrinsic parameters
        required for pixel-to-3D back-projection.

        Input Arguments:
        ---
        `data` :  [ sensor_msgs/msg/CameraInfo ]
            Camera calibration information received from the camera.

        Returns:
        ---
        None

        Example call:
        ---
        `self.caminfocb(data)`
        '''

        self.cam_info = data

        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = float(data.k[0])
        self.fy = float(data.k[4])
        self.cx = float(data.k[2])
        self.cy = float(data.k[5])


    def _assign_stable_ids(self, centers, ore_type):
        '''
        Purpose:
        ---
        Assigns stable IDs to the two detections of a given ore type by
        matching current centre pixels to the nearest previously tracked
        centres in pixel space.

        Input Arguments:
        ---
        `centers` :  [ list ]
            Current detected centre pixel coordinates for one ore type.

        `ore_type` :  [ str ]
            Ore type whose tracked IDs are being updated.

        Returns:
        ---
        `id_assignments` :  [ list ]
            List of `(ore_id, centre)` pairs with stable IDs 1 or 2.

        Example call:
        ---
        `id_assignments = self._assign_stable_ids(centers, ore_type)`
        '''

        tracks = self.ore_tracks[ore_type]

        if tracks[0] is None and tracks[1] is None:
            ordered = sorted(centers, key=lambda p: (p[1], p[0]))[:2]
            for i, center in enumerate(ordered):
                tracks[i] = center
            return [(i + 1, center) for i, center in enumerate(ordered)]

        if len(centers) == 0:
            return []

        if len(centers) == 1:
            center = centers[0]
            available = [
                (i, track) for i, track in enumerate(tracks)
                if track is not None
            ]

            if available:
                best_i = min(
                    available,
                    key=lambda item:
                        (item[1][0] - center[0]) ** 2 +
                        (item[1][1] - center[1]) ** 2
                )[0]
            else:
                best_i = 0 if tracks[0] is None else 1

            tracks[best_i] = center
            return [(best_i + 1, center)]

        centers = centers[:2]

        if tracks[0] is None:
            tracks[0] = centers[0]
        if tracks[1] is None:
            tracks[1] = centers[1]

        # Compare the two possible assignments.
        a, b = centers
        cost_direct = (
            (tracks[0][0] - a[0]) ** 2 +
            (tracks[0][1] - a[1]) ** 2 +
            (tracks[1][0] - b[0]) ** 2 +
            (tracks[1][1] - b[1]) ** 2
        )
        cost_swapped = (
            (tracks[0][0] - b[0]) ** 2 +
            (tracks[0][1] - b[1]) ** 2 +
            (tracks[1][0] - a[0]) ** 2 +
            (tracks[1][1] - a[1]) ** 2
        )

        if cost_swapped < cost_direct:
            tracks[0], tracks[1] = b, a
        else:
            tracks[0], tracks[1] = a, b

        return [(1, tracks[0]), (2, tracks[1])]

    def process_image(self):
        '''
        Purpose:
        ---
        Detects ores in the current camera frame, estimates their 3D
        positions using aligned depth and camera intrinsics, transforms the
        points into `base_link`, and continuously broadcasts all valid ore
        transforms.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        `self.process_image()`
        '''

        if (
            self.cv_image is None or
            self.depth_image is None or
            self.cam_info is None or
            self.camera_frame is None
        ):
            return

        if (
            self.fx is None or
            self.fy is None or
            self.cx is None or
            self.cy is None
        ):
            return

        annotated_frame = self.cv_image.copy()

        center_ore_list, ore_type_list = detect_ores(annotated_frame)

        detections_by_type = {
            ore_type: []
            for ore_type in ore_types
        }

        for center, ore_type in zip(center_ore_list, ore_type_list):
            detections_by_type[ore_type].append(center)

        for ore_type in ore_types:

            id_assignments = self._assign_stable_ids(
                detections_by_type[ore_type],
                ore_type
            )

            for ore_id, (cX, cY) in id_assignments:

                if (
                    cX < 4 or
                    cY < 4 or
                    cX >= self.depth_image.shape[1] - 4 or
                    cY >= self.depth_image.shape[0] - 4
                ):
                    continue

                patch = self.depth_image[
                    cY - 4:cY + 5,
                    cX - 4:cX + 5
                ]

                valid_depth = patch[
                    np.isfinite(patch) &
                    (patch > 0.0)
                ]

                if valid_depth.size == 0:
                    continue

                z = float(np.median(valid_depth))

                # Back-project pixel + z into the camera optical frame.
                x = (float(cX) - self.cx) * z / self.fx
                y = (float(cY) - self.cy) * z / self.fy

                point_camera = PointStamped()
                point_camera.header.stamp = self.get_clock().now().to_msg()
                point_camera.header.frame_id = self.camera_frame
                point_camera.point.x = float(x)
                point_camera.point.y = float(y)
                point_camera.point.z = float(z)

                try:
                    tf = self.tf_buffer.lookup_transform(
                        base_frame,
                        self.camera_frame,
                        rclpy.time.Time()
                    )

                    point_base = do_transform_point(
                        point_camera,
                        tf
                    )

                except Exception as e:
                    print(
                        f"TF lookup failed for {ore_type}_{ore_id}: {e}"
                    )
                    continue

                frame_name = f"{ore_type}_{ore_id}"

                # Publish every ore on every processing cycle.
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = base_frame
                t.child_frame_id = frame_name

                t.transform.translation.x = float(point_base.point.x)
                t.transform.translation.y = float(point_base.point.y)
                t.transform.translation.z = float(point_base.point.z)

                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                self.br.sendTransform(t)

                # The submitted image must use exactly the same names as the TF frames.
                cv2.putText(
                    annotated_frame,
                    frame_name,
                    (cX + 8, cY - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA
                )
                cv2.putText(
                    annotated_frame,
                    frame_name,
                    (cX + 8, cY - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA
                )

        # Required Task 1A detection image.
        cv2.imwrite(
            DETECTION_IMAGE,
            annotated_frame
        )

        cv2.imshow(
            "Task 1A Detection",
            annotated_frame
        )
        cv2.waitKey(1)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Purpose:
    ---
    Initializes ROS 2, creates the Task 1A ore TF node, and spins the node
    until execution is stopped.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    `main()`
    '''

    rclpy.init(args=sys.argv)

    node = ore_tf()

    node.get_logger().info(
        'Task 1A ore TF node started'
    )

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()
