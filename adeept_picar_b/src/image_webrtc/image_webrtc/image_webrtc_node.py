import asyncio
import threading
import time
from fractions import Fraction

import av
import cv2
import rclpy
from aiortc.mediastreams import VideoStreamTrack
from av.frame import Frame
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import webrtc_server

FPS = 15
# FRAME_WIDTH = 480
# FRAME_HEIGHT = 320


class PlayerStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self._queue = asyncio.Queue(maxsize=15)

    def enqueue(self, frame: Frame) -> None:
        try:
            self._queue.put_nowait(frame)
        except asyncio.queues.QueueFull:
            dropped_frame = self._queue.get_nowait()
            self._queue.put_nowait(frame)
            # print(f"Frame {dropped_frame.pts} dropped")
            pass

    async def recv(self) -> Frame:
        frame = await self._queue.get()

        # print(f"Frame {frame.pts} consumed")

        return frame


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
        #     'v4l2h264enc extra-controls="foo,video_bitrate=700000" ! video/x-h264,level=(string)3 ! '
        #     "rtph264pay config-interval=1 pt=96 ! "
        #     "udpsink host=192.168.0.110 port=5004",
        #     cv2.CAP_GSTREAMER,
        #     0,
        #     FPS,
        #     (FRAME_WIDTH, FRAME_HEIGHT),
        #     True,
        # )

        self.video_stream = PlayerStreamTrack()
        self.time_base = Fraction(1, FPS)
        self.start_time = time.time()

        self.web_rtc_runner = webrtc_server.make_server(
            webrtc_server.WebRTCConfig(
                host="0.0.0.0",
                port=8080,
                play_without_decoding=False,
                video_codec="video/H264",
                # video_codec="video/webm",
            ),
            None,
            input_video=self.video_stream,
        )

        self.web_rtc_server = threading.Thread(
            target=webrtc_server.run_server,
            args=(self.web_rtc_runner,),
            daemon=True,
        )
        self.web_rtc_server.start()

    def camera_callback(self, msg: Image) -> None:
        frame = self.br.imgmsg_to_cv2(msg, "rgb8")

        # self.get_logger().info(f"Got image: {frame.shape}, {frame.dtype}")

        av_frame = av.VideoFrame.from_ndarray(frame, "rgb24")
        av_frame.time_base = self.time_base

        msg_ts = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1000000000.0)
        av_frame.pts = int((msg_ts - self.start_time) * FPS)

        self.video_stream.enqueue(av_frame)

        # self.gst_stream.write(frame)


def main(args=None):
    import logging

    # logging.basicConfig(level=logging.DEBUG)

    rclpy.init(args=args)

    node = ImageWebRTCNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
