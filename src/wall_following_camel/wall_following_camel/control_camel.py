import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from icecream import ic
import numpy as np

class Control_system(Node):
    def __init__(self):
        super().__init__("Control_system")
        self.lidar_sub = self.create_subscription(Float32MultiArray, "/wall_distance", self.lidar_callback,10)
        self.car_pub = self.create_publisher(Twist,"/cmd_vel_cont",10)
        self.horizontal_ray = 0
        self.diagonal_ray = 0
        self.car_params = []
        self.declare_parameters(
        namespace='',
        parameters=[
                ('angle_between', 45)
                # ! different param inputs from yaml with defailut
                # ('bool_param', True),
                # ('int_param', 42),
                # ('str_param', 'default'),
                # ('list_param', [0.0, 0.0]),
                # ('nested_param.sub_param1', 0),
                # ('nested_param.sub_param2', 0)
            ]
        )
        self.angle_between = self.get_parameter('min_time_col').get_parameter_value().integer_value





def main():
    ic('Im alive')
    rclpy.init() 
    braking_system = Control_system()
    rclpy.spin(braking_system)
    rclpy.shutdown()     


if __name__ == '__main__':
    main()
