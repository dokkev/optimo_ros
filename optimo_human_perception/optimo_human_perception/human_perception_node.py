import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header
import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

try:
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions
    from mediapipe.tasks.python.vision import (
        PoseLandmarker, PoseLandmarkerOptions, RunningMode,
        PoseLandmarksConnections,
    )
    from mediapipe.tasks.python.vision import drawing_utils as mp_drawing
    HAS_MEDIAPIPE = True
except ImportError:
    HAS_MEDIAPIPE = False


# MediaPipe Pose landmark names (33 landmarks)
POSE_LANDMARK_NAMES = [
    'nose', 'left_eye_inner', 'left_eye', 'left_eye_outer',
    'right_eye_inner', 'right_eye', 'right_eye_outer',
    'left_ear', 'right_ear', 'mouth_left', 'mouth_right',
    'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
    'left_wrist', 'right_wrist', 'left_pinky', 'right_pinky',
    'left_index', 'right_index', 'left_thumb', 'right_thumb',
    'left_hip', 'right_hip', 'left_knee', 'right_knee',
    'left_ankle', 'right_ankle', 'left_heel', 'right_heel',
    'left_foot_index', 'right_foot_index',
]

# Default model path — try installed share dir first, then source dir
def _find_model():
    # Installed location
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('optimo_human_perception')
        p = os.path.join(share, 'models', 'pose_landmarker_lite.task')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    # Source location fallback
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'models', 'pose_landmarker_lite.task')

DEFAULT_MODEL_PATH = _find_model()


class HumanPerceptionNode(Node):
    def __init__(self):
        super().__init__('human_perception')

        # Parameters
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)

        min_det = self.get_parameter('min_detection_confidence').value
        min_track = self.get_parameter('min_tracking_confidence').value
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        model_path = self.get_parameter('model_path').value

        # Publishers
        self.pose_pub = self.create_publisher(JointState, '/optimo/human_pose', 10)
        self.annotated_pub = self.create_publisher(Image, '~/annotated_image', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)

        # State
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        self.latest_depth = None
        self.frame_ts_ms = 0
        self.frame_count = 0
        self.process_every_n = 3  # skip frames to reduce latency

        # MediaPipe setup (new Tasks API)
        self.landmarker = None
        self.latest_result = None

        if HAS_MEDIAPIPE:
            if not os.path.exists(model_path):
                self.get_logger().error(
                    f'Model file not found: {model_path}\n'
                    'Download it: wget -O models/pose_landmarker_lite.task '
                    '"https://storage.googleapis.com/mediapipe-models/pose_landmarker/'
                    'pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"')
            else:
                options = PoseLandmarkerOptions(
                    base_options=BaseOptions(model_asset_path=model_path),
                    running_mode=RunningMode.VIDEO,
                    min_pose_detection_confidence=min_det,
                    min_tracking_confidence=min_track,
                    num_poses=1,
                )
                self.landmarker = PoseLandmarker.create_from_options(options)
                self.get_logger().info(
                    f'PoseLandmarker initialized (det={min_det}, track={min_track})')
        else:
            self.get_logger().error('MediaPipe not installed! pip3 install mediapipe')

        if not HAS_CV_BRIDGE:
            self.get_logger().error(
                'cv_bridge not available! sudo apt install ros-humble-cv-bridge')

        self.get_logger().info(
            f'Human perception node started. Subscribing to: {image_topic}')

    def depth_callback(self, msg):
        if not self.bridge:
            return
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}', throttle_duration_sec=5.0)

    def image_callback(self, msg):
        if not self.bridge or not self.landmarker:
            return

        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}', throttle_duration_sec=5.0)
            return

        # Convert to MediaPipe Image
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        # Monotonically increasing timestamp required by VIDEO mode
        self.frame_ts_ms += 33  # ~30fps
        result = self.landmarker.detect_for_video(mp_image, self.frame_ts_ms)

        stamp = self.get_clock().now().to_msg()

        if result.pose_landmarks and len(result.pose_landmarks) > 0:
            landmarks = result.pose_landmarks[0]  # first person
            self._publish_pose(landmarks, stamp, cv_image.shape)
            self._publish_annotated(cv_image, result, stamp)
        else:
            # No person detected
            pose_msg = JointState()
            pose_msg.header = Header(stamp=stamp, frame_id='camera_color_optical_frame')
            pose_msg.name = []
            pose_msg.position = []
            self.pose_pub.publish(pose_msg)

    def _publish_pose(self, landmarks, stamp, image_shape):
        """Publish skeleton as JointState with x,y,z positions interleaved."""
        h, w = image_shape[:2]
        pose_msg = JointState()
        pose_msg.header = Header(stamp=stamp, frame_id='camera_color_optical_frame')

        names = []
        positions = []  # x, y, z interleaved per landmark
        velocities = []  # visibility scores

        for i, lm in enumerate(landmarks):
            name = POSE_LANDMARK_NAMES[i] if i < len(POSE_LANDMARK_NAMES) else f'landmark_{i}'
            names.append(name)

            # Pixel coordinates (normalized 0-1 -> pixel)
            px = lm.x * w
            py = lm.y * h

            # Try to get depth at this pixel
            z = 0.0
            if self.latest_depth is not None:
                ix = int(lm.x * self.latest_depth.shape[1])
                iy = int(lm.y * self.latest_depth.shape[0])
                if 0 <= iy < self.latest_depth.shape[0] and 0 <= ix < self.latest_depth.shape[1]:
                    depth_val = self.latest_depth[iy, ix]
                    if depth_val > 0:
                        z = float(depth_val) / 1000.0  # mm -> meters

            positions.extend([px, py, z])
            velocities.append(float(lm.visibility))

        pose_msg.name = names
        pose_msg.position = positions  # [x0,y0,z0, x1,y1,z1, ...]
        pose_msg.velocity = velocities  # visibility per landmark

        self.pose_pub.publish(pose_msg)

    def _publish_annotated(self, cv_image, result, stamp):
        """Publish image with skeleton overlay drawn manually."""
        annotated = cv_image.copy()
        h, w = annotated.shape[:2]

        if result.pose_landmarks and len(result.pose_landmarks) > 0:
            landmarks = result.pose_landmarks[0]

            # Draw landmarks as circles
            points = {}
            for i, lm in enumerate(landmarks):
                px = int(lm.x * w)
                py = int(lm.y * h)
                points[i] = (px, py)
                cv2.circle(annotated, (px, py), 4, (0, 255, 0), -1)

            # Draw connections
            for conn in PoseLandmarksConnections.POSE_LANDMARKS:
                start_idx = conn.start
                end_idx = conn.end
                if start_idx in points and end_idx in points:
                    cv2.line(annotated, points[start_idx], points[end_idx],
                             (0, 255, 255), 2)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'camera_color_optical_frame'
            self.annotated_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Annotated image publish failed: {e}',
                                  throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = HumanPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
