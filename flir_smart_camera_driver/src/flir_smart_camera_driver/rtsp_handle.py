#!/usr/bin/env python3
import rospy
from . utils import CameraUtils
from imutils.video import VideoStream
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class RTSPCLIENT(CameraUtils):
    """ Client for RTSP streams from flir camera """

    def __init__(self):
        super(RTSPCLIENT, self).__init__()
        self.streams = list()
        self.cv_bridge = CvBridge()
        self.frame = None
        self.signal = True

    def initRTSPHandle(self):
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

    def imageStream(self, stream):
        """ Stream camera feed defined in stream handles

        Args: None
        Returns: None
        Raises: None
        """

        rospy.loginfo("{} started".format(stream['name']))
        publisher = rospy.Publisher(stream['name'], Image, queue_size=10)
        rtsp_stream = VideoStream(stream['rtsp_address']).start()
        while not rospy.is_shutdown() and self.signal:
            frame = rtsp_stream.read()
            if frame is None:
                # continue
                rospy.logerr("Frame is none")
            if 'ir' in stream['name']:
                self.frame = frame                
            ros_frame = self.cv_bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            ros_frame.header.stamp = rospy.Time.now()
            ros_frame.header.frame_id = 'camera_frame'
            publisher.publish(ros_frame)
            self.rate.sleep()
        rtsp_stream.stop()
        rospy.logdebug("{} cleaned successfuly".format(stream['name']))
   
    def rtspCleanup(self):
        self.signal = False
