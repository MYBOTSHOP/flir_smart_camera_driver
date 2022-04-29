#!/usr/bin/env python3
import rospy
import cv2
import os
import rospkg
import json
from . utils import CameraUtils
from imutils.video import VideoStream
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from requests import get
from datetime import datetime


class RTSPCLIENT(CameraUtils):
    """ Client for RTSP streams from flir camera """

    def __init__(self):
        super(RTSPCLIENT, self).__init__()
        self.streams = list()
        self.cv_bridge = CvBridge()
        self.frame = None
        self.signal = True

        # Human detection variables initalization
        ref_path = os.path.join(rospkg.RosPack().get_path('flir_smart_camera_driver'), 'config/pose/', 'mpi/')
        protoFile = ref_path + "pose_deploy_linevec_faster_4_stages.prototxt"
        weightsFile = ref_path + "pose_iter_160000.caffemodel"
        self.nPoints = 15
        self.POSE_PAIRS = [[0, 1], [1, 2], [2, 3], [3, 4], [1, 5], [5, 6], [6, 7], [1, 14], [14, 8], [8, 9], [9, 10], [14, 11], [11, 12], [12, 13]]
        self.threshold = 0.1
        self.net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
        self.net.setPreferableBackend(cv2.dnn.DNN_TARGET_CPU)
        # input image dimensions for the network
        self.inWidth = 368
        self.inHeight = 368        

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
   
    def detectHuman(self):
        server_request = self.uri + '/alarms/measure-function' + self.api_key
        publisher = rospy.Publisher('human_detection', Image, queue_size=10)
        while not rospy.is_shutdown():
            if self.frame is not None:
                server_response = get(server_request)
                for data in json.loads(server_response.content):
                    if data['active'] and data['triggered']:
                        frame = self.frame
                        human_detected = False
                        frameHeight, frameWidth = frame.shape[0:2]
                        inpBlob = cv2.dnn.blobFromImage(frame, 1.0/255, (self.inWidth, self.inHeight), (0, 0, 0), swapRB=False, crop=False)
                        self.net.setInput(inpBlob)
                        output = self.net.forward()
                        H, W = output.shape[2:4]
                        points = []
                        for i in range(self.nPoints):
                            probMap = output[0, i, :, :]
                            _, prob, _, point = cv2.minMaxLoc(probMap)
                            x = (frameWidth * point[0])/W
                            y = (frameHeight * point[1])/H
                            if prob > self.threshold : 
                                cv2.circle(frame, (int(x), int(y)), 8, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                                cv2.putText(frame, "{}".format(i), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, lineType=cv2.LINE_AA)
                                points.append((int(x), int(y)))
                            else:
                                points.append(None)
                        for pair in self.POSE_PAIRS:
                            partA, partB = pair
                            if points[partA] and points[partB]:
                                human_detected = True
                                cv2.line(frame, points[partA], points[partB], (0, 255, 255), 2)
                                cv2.circle(frame, points[partA], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
                        datet = str(datetime.now())
                        frame = cv2.putText(frame, datet, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)                       
                        ros_frame = self.cv_bridge.cv2_to_imgmsg(frame, encoding="passthrough")
                        ros_frame.header.stamp = rospy.Time.now()
                        ros_frame.header.frame_id = 'camera_frame'
                        publisher.publish(ros_frame) if human_detected else None
                        self.frame = None
            self.rate.sleep()

    def rtspCleanup(self):
        self.signal = False
