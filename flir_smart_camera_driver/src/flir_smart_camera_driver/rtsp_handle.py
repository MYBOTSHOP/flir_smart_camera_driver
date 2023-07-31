#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .utils import CameraUtils
from imutils.video import VideoStream
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class RTSPCLIENT(CameraUtils, Node):
    """ Client for RTSP streams from flir camera """

    def __init__(self):
        super().__init__()
        self.streams = list()
        self.cv_bridge = CvBridge()
        self.frame = None
        self.signal = True

    def init_rtsp_handle(self):
        """ Initialize RTSP stream handles

        Args: None
        Returns: None
        Raises: None
        """
        if self.is_rgb_required:
            for v_stream in self.visual_streams_format:
                self.streams.append({'name': '{}/rgb'.format('stream'),
                                     'rtsp_address': 'rtsp://{}/{}{}{}'.format(self.device_address, v_stream, '/ch1', '?overlay=off')})
        for stream in self.ir_streams_format:
            self.streams.append({'name': '{}/ir'.format('stream'),
                                 'rtsp_address': 'rtsp://{}/{}{}{}'.format(self.device_address, stream, '/', '?overlay=off')})

    def image_stream(self, stream):
        """ Stream camera feed defined in stream handles

        Args: None
        Returns: None
        Raises: None
        """

        self.get_logger().info("{} started".format(stream['name']))
        publisher = self.create_publisher(Image, stream['name'], 10)
        rtsp_stream = VideoStream(stream['rtsp_address']).start()
        while rclpy.ok() and self.signal:
            frame = rtsp_stream.read()
            if frame is None:
                self.get_logger().error("Frame is none")
            if 'ir' in stream['name']:
                self.frame = frame
            ros_frame = self.cv_bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            ros_frame.header.stamp = self.get_clock().now().to_msg()
            ros_frame.header.frame_id = 'camera_frame'
            publisher.publish(ros_frame)
            self.rate.sleep()
        rtsp_stream.stop()
        self.get_logger().debug("{} cleaned successfully".format(stream['name']))

    def rtsp_cleanup(self):
        self.signal = False

def main(args=None):
    rclpy.init(args=args)
    rtsp_client = RTSPCLIENT()
    rtsp_client.init_rtsp_handle()
    # Add your additional code here...
    try:
        rclpy.spin(rtsp_client)
    finally:
        rtsp_client.rtsp_cleanup()
        rtsp_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
