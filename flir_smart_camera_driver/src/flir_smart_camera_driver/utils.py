#!/usr/bin/env python3
import json
import rospy


class CameraUtils(object):
    """ Base class for camera driver. Handles most shared varaibles for child classes

    Args: 
        source: String, To point where get/post methods live in swagger.json file
    Returns: None
    Raises: 
        FileNotFoundError: In case the swagger file does not exist in the given path
        KeyError: If source does not exist in swagger.json
    """

    def __init__(self, source='paths'):
        self.device_address = rospy.get_param('~camera_ip', '0.0.0.0')
        self.is_rgb_required = rospy.get_param('~is_rgb_required', False)
        self.verbosity = rospy.get_param('~verbose', False)
        self.visual_streams_format = rospy.get_param('~visual_format', [])
        self.ir_streams_format = rospy.get_param('~ir_format', [])
        self.http = rospy.get_param('~is_http', False) 
        self.overlay = rospy.get_param('~overlay', False) 
        self.device_api_key = rospy.get_param('~camera_api_key', '')
        self.rate = rospy.Rate(25) # 25 hz
        api_file_path = rospy.get_param('~api_file_path', '')
        self.api_end_points = list()
        self.source = source # Dict key where end points exists
        self.api_remaps = {'/measurements/spots/id': 'FunctionSpot',
                           '/measurements/reftemps/id': 'FunctionRefTemp',
                           '/measurements/deltas/id': 'FunctionDelta',
                           '/measurements/boxes/id': 'FunctionBox',
                           '/measurements/lines/id': 'FunctionLine',
                           '/measurements/polylines/id': 'FunctionPolyline',
                           '/alarms/digin/id': 'DiginAlarm',
                           '/alarms/measure-function/id': 'MeasureFunctionAlarm',
                           '/alarms/temperature-sensor/id': 'TempSensAlarm',
                           '/device/details': 'DeviceDetails',
                           '/device/configuration': 'DeviceConfiguration',
                           '/device/nuc': 'DeviceNuc',
                           '/device/focus': 'DeviceFocusUpdate',
                           '/device/focus_auto': 'DeviceFocusAutoUpdate',
                           '/device/io/outputs/id': 'DigitalOutput',
                           '/image/state': 'ImageState',
                           '/regional/units': 'RegionalUnits',
                           '/regional/time': 'RegionalTime',
                           '/screening/configuration': 'ScreeningConfiguration',
                           '/screening/operator/configuration': 'ScreeningOperatorConfiguration', 
                           '/device/pts/pan': 'DevicePan',
                           '/device/pts/tilt': 'DeviceTilt',
                           '/screening/operator/capture-sample': 'RetrieveData',
                           '/screening/operator/reset-samples': 'RetrieveData',
                           '/device/pts/stop': 'RetrieveData'
                           }
        with open(api_file_path, 'r') as swagger:
            api_handle = json.load(swagger)
        for key in api_handle[self.source].keys():
            # Divide based on get and post methods and resolve names to avoid ROS naming warnings
            resolved_ros_key = key if '/{' not in key else key.replace('{id}', 'id')
            if {'get', 'post'}.issubset(list(api_handle[self.source][key].keys())):
                self.api_end_points.append(['get/set', resolved_ros_key])
                if '/device/focus' == resolved_ros_key:
                    self.api_end_points.append(['set', resolved_ros_key.replace('focus', 'focus_auto')])
            elif 'get' in list(api_handle[self.source][key].keys()):
                self.api_end_points.append(['get', resolved_ros_key])
            elif 'post' in list(api_handle[self.source][key].keys()):
                self.api_end_points.append(['set', resolved_ros_key])

    def logData(self, data):
        if not self.verbosity:
            return
        for key, value in data.items():
            rospy.loginfo("{}: {}".format(key, value))