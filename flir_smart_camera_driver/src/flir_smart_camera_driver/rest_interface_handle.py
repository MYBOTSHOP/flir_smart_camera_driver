#!/usr/bin/env python3
from numpy import uint16
import rospy
import subprocess
import json
from requests import get, post
from flir_msgs.srv import *
from flir_msgs.msg import ActiveAlarms, TriggeredAlarms
from functools import partial as bind
from . utils import CameraUtils
    

class RestInterfaceHandle(CameraUtils):
    """ REST interface handle for FLIR camera """
    
    def __init__(self):        
        super(RestInterfaceHandle, self).__init__()      
        self.uri = '{}://{}/api'.format('http' if self.http else 'https', self.device_address)
        self.api_key = '?apiKey={}'.format(self.device_api_key)
        self.is_alive = False      # To check if camera is still alive
        self.ros_msg_dict = dict() # Logic handle for data conversion
        self.ros_msg_type = None   # Logic handle for data conversion

        self.active_alarms_pub = rospy.Publisher('alarms/active', ActiveAlarms, queue_size=10)
        self.triggered_alarms_pub = rospy.Publisher('alarms/triggered', TriggeredAlarms, queue_size=10)

    def isAlive(self, n_replies=1, timeout=1):
        """Check if the camera is still alive or not

        Args:
            n_replies: Number of replies to wait for camera ping
            timeout: Time in seconds to wait for camera response
        Returns:
            None, Shutdowns the ROS Node
        Raises:
            None
        """
        rospy.loginfo('Connecting to the camera at: {}'.format(self.device_address))
        init_success = False
        while not rospy.is_shutdown():
            status, _ = subprocess.getstatusoutput('ping -c{} -w{} {}'.format(n_replies, timeout, self.device_address))
            if status != 0:
                self.is_alive = False
                if init_success:
                    rospy.logerr('Camera connection lost!')
                    rospy.logerr('Killing camera node.')
                    rospy.signal_shutdown('camera connection lost!')
            else:
                init_success = True
                self.is_alive = True
  
    def deserializeMessage(self, msg):
        """Returns any ROS message type and content

        Args:
            msg: Any ROS msg
        Returns:
            List of message content, list of content type
        Raises:
            None
        """

        return msg.__slots__, msg._slot_types
    
    def checkMsgType(self, msg):
        return msg._type

    def genDeviceFocusMsg(self, msg, m_type):
        m_fields, _ = self.deserializeMessage(msg)
        m_type = eval(m_type.split('/')[-1] + '()')
        for field in m_fields:
            if eval('msg.{}'.format(field)) != eval('m_type.{}'.format(field)):
                return field

    def genRequest(self, msg):
        """ Convert any ROS message to a dictionary or dictionary of dictionaries

        Args:
            msg: Any ROS message to convert to a python dictionary
        Returns:
            None: Its a recursive fucntion and updates a shared variable: @self.ros_msg_dict
        Raises:
            None
        """      

        request = {}
        msg_fields, msg_types = self.deserializeMessage(msg)
        for name, d_type in zip(msg_fields, msg_types):
            if '/' not in d_type:
                request.update({name: eval('msg.{}'.format(name))})
            else:
                self.ros_msg_type = name
                self.genRequest((eval('msg.{}'.format(name))))
        if self.ros_msg_type not in self.ros_msg_dict and self.ros_msg_type is not None:
            self.ros_msg_dict.update({self.ros_msg_type: request})
        else: 
            self.ros_msg_dict.update(request)

    def setAttributeCallback(self, name, response_type,  msg):
        """ Callback for all post(set) methods for service

        Args:
            name: Service server name as per swagger file definition
            response_type: ROS service response type
            msg: Request from the client            
        Returns:
            response: of type @reponse_type
        Raises:
            None
        """

        if 'id' in name: 
            name = name.replace('id', msg.id)
        server_request = self.uri + name + self.api_key
        self.ros_msg_dict = dict()
        self.ros_msg_type = None
        self.genRequest(msg)
        m_type = self.checkMsgType(msg)
        request = self.ros_msg_dict
        if m_type in ['flir_msgs/DeviceFocusUpdateRequest', 'flir_msgs/DeviceFocusAutoUpdateRequest']:
            focus_update_msg = self.genDeviceFocusMsg(msg, m_type)
            request = {focus_update_msg: self.ros_msg_dict[focus_update_msg]} 
            if m_type == 'flir_msgs/DeviceFocusAutoUpdateRequest':
                server_request = server_request.replace('_auto', '') 
        log = {'request url': server_request, 'request json': request}
        self.logData(log)
        server_response = post(server_request, json=request)
        response = response_type()
        response.status_code = server_response.status_code
        try:
            response.message  = str(json.loads(server_response.content))
        except json.decoder.JSONDecodeError:
            response.message = server_response.text
        return response

    def getAttributeCallback(self, name, msg):
        """ Callback for all get type of services

        Args:
            name: Service server name as per swagger file definition
            msg: Request message from the client
        Returns:
            response: of type RetrieveDataResponse or IdRetrieveDataResponse
        Raises:
            None
        """

        response = RetrieveDataResponse()
        if isinstance(msg, IdRetrieveDataRequest): 
            name = name.replace('id', msg.id)
            response = IdRetrieveDataResponse()       
        server_request = self.uri + name + self.api_key
        log = {'request url': server_request}
        self.logData(log)
        server_response = get(server_request)
        response.status_code = server_response.status_code
        try:
            response.message = str(json.loads(server_response.content))
        except json.decoder.JSONDecodeError:
            response.message  = server_response.text       
        return response
    
    def checkAlarmStatus(self):
        """ Check alarm status and publish to respective topics

        Args: None
        Returns: None
        Raises: None
        """     
        
        server_request = self.uri + '/alarms/measure-function' + self.api_key
        while not rospy.is_shutdown():
            server_response = get(server_request)
            active_alarms = ActiveAlarms()
            triggered_alarms = TriggeredAlarms()
            for data in json.loads(server_response.content):
                if data['active']:
                    active_alarms.id.append(uint16(data['id']))
                    active_alarms.active.append(data['active'])
                    active_alarms.label.append(data['label'])
                    active_alarms.measurand_type.append(data['measureFuncType'])
                    self.active_alarms_pub.publish(active_alarms) if self.active_alarms_pub.get_num_connections() > 0 else None
                    if data['triggered']:
                        triggered_alarms.id.append(uint16(data['id']))
                        triggered_alarms.active.append(data['active'])
                        triggered_alarms.label.append(data['label'])
                        triggered_alarms.triggered.append(data['triggered'])
                        triggered_alarms.measurand_type.append(data['measureFuncType'])
                        self.triggered_alarms_pub.publish(triggered_alarms) if self.triggered_alarms_pub.get_num_connections() > 0 else None
            self.rate.sleep()

    def initRESTInterfaceHandle(self):
        """Initialize the REST interface get/post commands as ROS services

        Args: None
        Returns: None
        Raises: None
        """        

        for s_type, s_name in self.api_end_points:
            if 'get' in s_type:
                if 'id' in s_name:
                    rospy.Service('get{}'.format(s_name.replace('-', '_')),
                                  IdRetrieveData,
                                  bind(self.getAttributeCallback, s_name))
                else:
                    rospy.Service('get{}'.format(s_name.replace('-', '_')),
                                  RetrieveData,
                                  bind(self.getAttributeCallback, s_name))
            if 'set' in s_type:
                ros_service_type = self.api_remaps[s_name]
                rospy.Service('set{}'.format(s_name.replace('-', '_')),
                              eval(ros_service_type),
                              bind(self.setAttributeCallback, s_name, eval('{}Response'.format(ros_service_type))))        

    def InterfaceCleanup(self):
        rospy.signal_shutdown("Shut down request received")
    