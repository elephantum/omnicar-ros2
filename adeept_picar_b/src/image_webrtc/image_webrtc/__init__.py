import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

FPS = 15
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


class ImageWebRTCNode(Node):
    def __init__(self):
        super().__init__("image_webrtc_node")

        self.camera_sub = self.create_subscription(
            Image,
            "camera/image_raw",
            self.camera_callback,
            3,
        )
        self.br = CvBridge()

        # # gst-launch-1.0 libcamerasrc !
        # # video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert !
        # # x264enc tune=zerolatency ! rtph264pay config-interval=1 pt=96 !
        # # udpsink host=192.168.0.110 port=5004
        # self.gst_stream = cv2.VideoWriter(
        #     "appsrc ! "
        #     f"video/x-raw,width={FRAME_WIDTH},height={FRAME_HEIGHT},framerate={FPS}/1 ! "
        #     "videoconvert ! "
        #     # "x264enc tune=zerolatency bitrate=800 ! "
        #     "queue ! "
        #     # 'v4l2h264enc extra-controls="foo,video_bitrate=700000" ! video/x-h264,level=(string)3 ! '
        #     'v4l2h264enc extra-controls="controls,h264_entropy_mode=0,h264_profile=4,h264_level=3,video_bitrate=700000;" ! video/x-h264,level=(string)3 ! '
        #     "rtph264pay config-interval=1 pt=96 ! "
        #     "udpsink host=192.168.0.110 port=5004",
        #     cv2.CAP_GSTREAMER,
        #     0,
        #     FPS,
        #     (FRAME_WIDTH, FRAME_HEIGHT),
        #     True,
        # )

    def camera_callback(self, msg: Image) -> None:
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")

        self.get_logger().info(f"Got image: {frame.shape}, {frame.dtype}")

        # self.gst_stream.write(frame)


def main(args=None):
    rclpy.init(args=args)

    node = ImageWebRTCNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
