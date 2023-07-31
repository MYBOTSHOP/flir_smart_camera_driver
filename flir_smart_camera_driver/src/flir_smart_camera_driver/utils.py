#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node

class CameraUtils(Node):
    def __init__(self, source='paths'):
        super().__init__('camera_utils')
        self.device_address = self.get_parameter('camera_ip').get_parameter_value().string_value
        self.is_rgb_required = self.get_parameter('is_rgb_required').get_parameter_value().bool_value
        self.verbosity = self.get_parameter('verbose').get_parameter_value().bool_value
        self.visual_streams_format = self.get_parameter('visual_format').get_parameter_value().string_array_value
        self.ir_streams_format = self.get_parameter('ir_format').get_parameter_value().string_array_value
        self.http = self.get_parameter('is_http').get_parameter_value().bool_value
        self.overlay = self.get_parameter('overlay').get_parameter_value().bool_value
        self.device_api_key = self.get_parameter('camera_api_key').get_parameter_value().string_value
        self.rate = 25  # 25 hz
        api_file_path = self.get_parameter('api_file_path').get_parameter_value().string_value
        self.api_end_points = []
        self.source = source  # Dict key where endpoints exist
        self.api_remaps = {
            '/measurements/spots/id': 'FunctionSpot',
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
            resolved_ros_key = key if '/{' not in key else key.replace('{id}', 'id')
            if {'get', 'post'}.issubset(list(api_handle[self.source][key].keys())):
                self.api_end_points.append(['get/set', resolved_ros_key])
                if '/device/focus' == resolved_ros_key:
                    self.api_end_points.append(['set', resolved_ros_key.replace('focus', 'focus_auto')])
            elif 'get' in list(api_handle[self.source][key].keys()):
                self.api_end_points.append(['get', resolved_ros_key])
            elif 'post' in list(api_handle[self.source][key].keys()):
                self.api_end_points.append(['set', resolved_ros_key])

    def log_data(self, data):
        if not self.verbosity:
            return
        for key, value in data.items():
            self.get_logger().info("{}: {}".format(key, value))

def main(args=None):
    rclpy.init(args=args)
    camera_utils = CameraUtils()
    rclpy.spin(camera_utils)
    camera_utils.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
