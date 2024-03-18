#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
#from geometry_msgs.msg import Twist,Pose, Point, Vector3
#from std_msgs.msg import Float64MultiArray
import numpy as np
from builtin_interfaces.msg import Time
from ngc_interfaces.msg import Nu, NuDot, Ned, Tau
import pygame
import BoatVisualizer as bv


class SimpleGui(Node):
    def __init__(self):
        super().__init__("simple_gui")
        self.ned_sub = self.create_subscription(msg_type= Tau, topic= "eta_sim", callback= self.boat_mover, qos_profile=10)
        self.boat_viz = bv.BoatMovementVisualizer(fps=30)
        self.running = True


    def boat_mover(self, msg:Ned):
        while self.running: # if the pygame window is cloesed, the pygame thread is closed
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    self.destroy_node()
        eta = [msg.x, msg.y, 0,0,0,msg.psi]
        self.boat_viz.update(eta= eta)
    
def main(args=None):
    rclpy.init(args= args)
    node = SimpleGui()
    rclpy.spin(node)
    rclpy.shutdown()
