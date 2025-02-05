import rclpy
from rclpy import Node


class StateVisualizator(Node):
    def __init__(self):
        super().__init__("Visualizator Node")