#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
#from geometry_msgs.msg import Twist,Pose, Point, Vector3
#from std_msgs.msg import Float64MultiArray
import numpy as np
from builtin_interfaces.msg import Time
from ngc_interfaces.msg import Nu, NuDot, Ned, Tau
import math
"""
This class holds the methods to return the KT curves
"""
class ThrustTorqueCurve():
        def __init__(self) -> None:
                pass
        
        def my_KT_curve(self, J, pitch):
                """
                Basert på wagner B-series 3-blade AE/AO =0.5, P/D (pitch) 0.5-1.4. 
                The values in the diagram are linearzed.
                """
                Kt0 = 0.38
                dKt = 0.35
                return Kt0 * pitch - dKt *J

def main():
        pass

if __name__ == '__main__':
    main()