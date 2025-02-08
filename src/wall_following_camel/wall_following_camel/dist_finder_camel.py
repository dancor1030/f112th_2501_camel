import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from icecream import ic
import numpy as np

class Distance_finder(Node):
    def __init__(self):
        super().__init__("Distance_finder")
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.__lidar_callback,10)
        self.emergency_pub = self.create_publisher(Float32MultiArray,"/wall_distance",10)
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

    def __lidar_callback(self, msg):
        
        self.horizontal_ray =  self.__get_horizontal_ray(msg)

        self.diagonal_ray = self.__get_diagonal_ray(msg)

        self.car_params = self.__get_car_params(self.horizontal_ray, self.diagonal_ray, msg.angle_between)

        self.car_pub.publish(self.car_params) 



    def __get_car_params(horizontal_ray : float,  diagonal_ray : float, angle_between : float) -> float:
        angle = np.arctan2((horizontal_ray*np.cos(angle_between) - diagonal_ray), (horizontal_ray*np.sin(angle_between)))
        distance_to_wall = diagonal_ray*np.cos(angle)
        return [angle, distance_to_wall]


    def __get_horizontal_ray(self, msg) -> float:
        self.horizontal_ray = msg.ranges[len(msg.ranges)//4]
        return self.horizontal_ray
    
    def __get_diagonal_ray(self, msg) -> float:
        self.diagonal_ray = msg.ranges[len(msg.ranges)//4 + self.angle_between]
        return self.diagonal_ray


def main():
    ic('Im alive')
    rclpy.init()
    braking_system = Distance_finder()
    rclpy.spin(braking_system)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
