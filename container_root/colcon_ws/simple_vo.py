#!/usr/bin/env python3
# ROS 2 monocular visual odometry node converted from the provided OpenCV script
# - Subscribes to /camera/image_raw and /camera/camera_info
# - Optionally undistorts using CameraInfo
# - Publishes nav_msgs/Odometry on /vo/odom, geometry_msgs/PoseStamped on /vo/pose, nav_msgs/Path on /vo/path
# - Broadcasts TF map->camera_frame
# - Publishes a debug image with tracked features on /vo/debug_image

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header

from cv_bridge import CvBridge
import tf_transformations as tft
from tf2_ros import TransformBroadcaster

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2


def rotmat_to_quat(R: np.ndarray):
    # OpenCV returns proper rotation matrix; convert to quaternion [x,y,z,w]
    # tf transformations expects a homogeneous matrix or axis-angle; use from_matrix
    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = R
    q = tft.quaternion_from_matrix(m)
    return q  # x, y, z, w


class PinholeCamera:
    def __init__(self, width, height, fx, fy, cx, cy, k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
        self.width = int(width)
        self.height = int(height)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.distortion = (abs(k1) > 1e-7) or (abs(k2) > 1e-7) or (abs(p1) > 1e-7) or (abs(p2) > 1e-7) or (abs(k3) > 1e-7)
        self.d = [float(k1), float(k2), float(p1), float(p2), float(k3)]


lk_params = dict(winSize=(21, 21),
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


def feature_tracking(image_ref, image_cur, px_ref):
    # Ensure input dtype/shapes
    if px_ref is None or len(px_ref) == 0:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    px_ref = px_ref.reshape(-1, 1, 2).astype(np.float32)
    kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)
    if kp2 is None or st is None:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)
    st = st.reshape(-1).astype(bool)
    kp1 = px_ref.reshape(-1, 2)[st]
    kp2 = kp2.reshape(-1, 2)[st]
    return kp1.astype(np.float32), kp2.astype(np.float32)


class VisualOdometry:
    def __init__(self, cam: PinholeCamera, min_features=1500, fast_threshold=25, gt_lines=None):
        self.frame_stage = STAGE_FIRST_FRAME
        self.cam = cam
        self.new_frame = None
        self.last_frame = None
        self.cur_R = None
        self.cur_t = None
        self.px_ref = None
        self.px_cur = None
        self.focal = cam.fx
        self.pp = (cam.cx, cam.cy)
        self.detector = cv2.FastFeatureDetector_create(threshold=int(fast_threshold), nonmaxSuppression=True)
        self.min_features = int(min_features)
        self.gt_lines = gt_lines  # list[str] of KITTI-style poses, optional

    def get_absolute_scale(self, frame_id):
        if self.gt_lines is None:
            return 1.0
        if frame_id <= 0 or frame_id >= len(self.gt_lines):
            return 1.0
        try:
            ss_prev = self.gt_lines[frame_id - 1].strip().split()
            ss = self.gt_lines[frame_id].strip().split()
            x_prev, y_prev, z_prev = float(ss_prev[3]), float(ss_prev[7]), float(ss_prev[11])
            x, y, z = float(ss[3]), float(ss[7]), float(ss[11])
            return math.sqrt((x - x_prev) ** 2 + (y - y_prev) ** 2 + (z - z_prev) ** 2)
        except Exception:
            return 1.0

    def process_first_frame(self):
        kp = self.detector.detect(self.new_frame)
        self.px_ref = np.array([k.pt for k in kp], dtype=np.float32)
        self.frame_stage = STAGE_SECOND_FRAME

    def process_second_frame(self):
        self.px_ref, self.px_cur = feature_tracking(self.last_frame, self.new_frame, self.px_ref)
        if self.px_cur.shape[0] < 8:
            # not enough points; re-detect and try again next frame
            kp = self.detector.detect(self.new_frame)
            self.px_ref = np.array([k.pt for k in kp], dtype=np.float32)
            return
        E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp,
                                        method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return
        _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp=self.pp)
        self.frame_stage = STAGE_DEFAULT_FRAME
        self.px_ref = self.px_cur

    def process_frame(self, frame_id):
        self.px_ref, self.px_cur = feature_tracking(self.last_frame, self.new_frame, self.px_ref)
        if self.px_cur.shape[0] < 8:
            # re-detect
            kp = self.detector.detect(self.new_frame)
            self.px_cur = np.array([k.pt for k in kp], dtype=np.float32)
            self.px_ref = self.px_cur
            return False
        E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp,
                                        method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return False
        _, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp=self.pp)
        if self.cur_R is None:
            self.cur_R = R
        if self.cur_t is None:
            self.cur_t = np.zeros((3, 1), dtype=np.float64)
        absolute_scale = self.get_absolute_scale(frame_id)
        if absolute_scale > 0.01:
            self.cur_t = self.cur_t + absolute_scale * (self.cur_R @ t)
            self.cur_R = R @ self.cur_R
        if self.px_ref.shape[0] < self.min_features:
            kp = self.detector.detect(self.new_frame)
            self.px_cur = np.array([k.pt for k in kp], dtype=np.float32)
        self.px_ref = self.px_cur
        return True

    def update(self, img_gray: np.ndarray, frame_id: int):
        assert img_gray.ndim == 2, "Image must be grayscale"
        assert img_gray.shape[0] == self.cam.height and img_gray.shape[1] == self.cam.width, \
            "Image size does not match camera intrinsics"
        self.new_frame = img_gray
        if self.frame_stage == STAGE_DEFAULT_FRAME:
            return self.process_frame(frame_id)
        elif self.frame_stage == STAGE_SECOND_FRAME:
            self.process_second_frame()
            return False
        elif self.frame_stage == STAGE_FIRST_FRAME:
            self.process_first_frame()
            return False
        else:
            return False


class VONode(Node):
    def __init__(self):
        super().__init__('mono_vo_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('use_undistort', True)
        self.declare_parameter('min_features', 1500)
        self.declare_parameter('fast_threshold', 25)
        self.declare_parameter('gt_annotations', '')  # optional KITTI-style poses
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'camera')
        self.declare_parameter('publish_path', True)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.use_undistort = self.get_parameter('use_undistort').get_parameter_value().bool_value
        min_features = self.get_parameter('min_features').get_parameter_value().integer_value
        fast_threshold = self.get_parameter('fast_threshold').get_parameter_value().integer_value
        gt_path = self.get_parameter('gt_annotations').get_parameter_value().string_value
        self.map_frame = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.publish_path = self.get_parameter('publish_path').get_parameter_value().bool_value

        # Load GT lines optionally
        gt_lines = None
        if gt_path:
            try:
                with open(gt_path, 'r') as f:
                    gt_lines = f.readlines()
                    self.get_logger().info(f"Loaded ground-truth poses: {len(gt_lines)} lines")
            except Exception as e:
                self.get_logger().warn(f"Could not read gt_annotations file: {e}")

        # QoS suitable for camera streams
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.bridge = CvBridge()
        self.have_cam_info = False
        self.cam = None
        self.vo = None
        self.frame_id_counter = 0

        self.map1 = None
        self.map2 = None

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'vo/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'vo/pose', 10)
        self.path_pub = self.create_publisher(Path, 'vo/path', 10) if self.publish_path else None
        self.debug_img_pub = self.create_publisher(Image, 'vo/debug_image', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.img_sub = self.create_subscription(Image, image_topic, self.on_image, qos)
        self.caminfo_sub = self.create_subscription(CameraInfo, camera_info_topic, self.on_caminfo, qos)

        self.get_logger().info(f"Subscribing to: {image_topic} and {camera_info_topic}")

        # VO will be initialized after first CameraInfo
        self._pending_vo_init = {'min_features': min_features, 'fast_threshold': fast_threshold, 'gt_lines': gt_lines}

    def on_caminfo(self, msg: CameraInfo):
        if self.have_cam_info:
            return
        if msg.k[0] == 0.0 or msg.k[4] == 0.0:
            self.get_logger().warn("CameraInfo has zero intrinsics; waiting for a valid message...")
            return
        w = int(msg.width)
        h = int(msg.height)
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        # D can be size 5 or more in ROS; take k1,k2,t1,t2,k3
        d = list(msg.d) + [0.0] * max(0, 5 - len(msg.d))
        k1, k2, p1, p2, k3 = d[:5]
        self.cam = PinholeCamera(w, h, fx, fy, cx, cy, k1, k2, p1, p2, k3)

        if self.use_undistort and self.cam.distortion:
            K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
            D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
            self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, K, (w, h), cv2.CV_32FC1)
            self.get_logger().info("Undistortion maps created.")
        else:
            self.map1, self.map2 = None, None

        # Initialize VO
        self.vo = VisualOdometry(self.cam,
                                 min_features=self._pending_vo_init['min_features'],
                                 fast_threshold=self._pending_vo_init['fast_threshold'],
                                 gt_lines=self._pending_vo_init['gt_lines'])
        self.have_cam_info = True
        self.get_logger().info(f"Camera model set: {w}x{h}, fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}, dist={self.cam.d}")

    def on_image(self, msg: Image):
        if not self.have_cam_info:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        # Convert to grayscale
        if cv_img.ndim == 3:
            img_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY) if cv_img.shape[2] == 3 else cv2.cvtColor(cv_img, cv2.COLOR_BGRA2GRAY)
        else:
            img_gray = cv_img

        # Optional undistortion to improve E estimation
        if self.map1 is not None and self.map2 is not None:
            img_gray = cv2.remap(img_gray, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        # Ensure size matches intrinsics
        if img_gray.shape[1] != self.cam.width or img_gray.shape[0] != self.cam.height:
            self.get_logger().warn_once("Image size does not match CameraInfo; resizing (this may affect accuracy).")
            img_gray = cv2.resize(img_gray, (self.cam.width, self.cam.height), interpolation=cv2.INTER_LINEAR)

        # Update VO
        updated = self.vo.update(img_gray, self.frame_id_counter)
        self.frame_id_counter += 1

        # Prepare and publish outputs when pose available
        if self.vo.cur_R is not None and self.vo.cur_t is not None:
            stamp = msg.header.stamp
            # Odometry
            odom = Odometry()
            odom.header = Header(stamp=stamp, frame_id=self.map_frame)
            odom.child_frame_id = self.child_frame
            odom.pose.pose.position.x = float(self.vo.cur_t[0])
            odom.pose.pose.position.y = float(self.vo.cur_t[1])
            odom.pose.pose.position.z = float(self.vo.cur_t[2])
            qx, qy, qz, qw = rotmat_to_quat(self.vo.cur_R)
            odom.pose.pose.orientation.x = float(qx)
            odom.pose.pose.orientation.y = float(qy)
            odom.pose.pose.orientation.z = float(qz)
            odom.pose.pose.orientation.w = float(qw)
            self.odom_pub.publish(odom)

            # PoseStamped
            ps = PoseStamped()
            ps.header = Header(stamp=stamp, frame_id=self.map_frame)
            ps.pose = odom.pose.pose
            self.pose_pub.publish(ps)

            # Path
            if self.publish_path:
                self.path_msg.header.stamp = stamp
                self.path_msg.poses.append(ps)
                # Keep path from growing unbounded
                if len(self.path_msg.poses) > 5000:
                    self.path_msg.poses = self.path_msg.poses[-5000:]
                self.path_pub.publish(self.path_msg)

            # TF
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.child_frame
            tf.transform.translation.x = odom.pose.pose.position.x
            tf.transform.translation.y = odom.pose.pose.position.y
            tf.transform.translation.z = odom.pose.pose.position.z
            tf.transform.rotation.x = odom.pose.pose.orientation.x
            tf.transform.rotation.y = odom.pose.pose.orientation.y
            tf.transform.rotation.z = odom.pose.pose.orientation.z
            tf.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(tf)

        # Debug image with tracks
        try:
            dbg = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
            if self.vo.px_ref is not None and self.vo.px_cur is not None:
                for p1, p2 in zip(self.vo.px_ref.astype(int), self.vo.px_cur.astype(int)):
                    cv2.circle(dbg, tuple(p2), 2, (0, 255, 0), -1)
            self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))
        except Exception:
            pass


def main():
    rclpy.init()
    node = VONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
