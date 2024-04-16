#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from ngc_interfaces.msg import Tau, SetPoints
import numpy as np
from pynput.keyboard import Key, Listener


class SetPointsTeleop(Node):
    def __init__(self):
        super().__init__('set_points_teleop')
        self.timer_period_ = 0.01  # seconds
        self.timer_ = self.create_timer(self.timer_period_, self.publish_commands)
        self.send_cmd = self.create_publisher(SetPoints, 'set_points_thrusters', 1)
        self.set_points = []
        self.n = 0.0
        self.pitch = 0.5
        self.angle = 0.0
        self.rps_scalar = 1.0
        self.pitch_scalar = 0.01
        self.angle_scalar = 0.01
        self.can_print = True
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()
        self.keys_bindings = ["q", "e", "s", "w", "a", "d", "z", "c"]
        self.special_keys_bindings = [Key.up, Key.down, Key.left, Key.right]
        self.get_logger().info("Manual set points controller node has started.")

    def publish_commands(self):
        self.set_points = SetPoints()
        print("NB!: Temporary, hard coded to send set points to a dual controlable pitch and rudder configuration.")
        #TODO: change this to construct the set points based on th eThrusterSystem configuration.
        self.set_points.setpoints = [self.n, self.pitch, self.angle, self.n, self.pitch, self.angle] 
        self.send_cmd.publish(self.set_points)
        
        self.print_cmd()

    def on_press(self, key):
 
        if self._is_special_key(key) and key in self.special_keys_bindings: 
            if key == Key.up:
                self.n += 1* self.rps_scalar
            elif key == Key.down:
                self.n -= 1* self.rps_scalar
            elif key == Key.left:
                self.angle += 1* self.angle_scalar
            elif key == Key.right:
                self.angle -= 1* self.angle_scalar
        elif not self._is_special_key(key) and key.char in self.keys_bindings:
            if key.char == 'w':
                self.pitch += 1* self.pitch_scalar
            elif key.char == 's':
                self.pitch -= 1* self.pitch_scalar
        else:
            print("not a valid key")

    def _is_special_key(self, key):
        try:
            key.char
            return False
        except AttributeError:
            return True

    def print_cmd(self):
        if self.can_print:
            print(f" set points: {self.set_points}")        


def main(args=None):
    try:
        rclpy.init(args=args)
        controller = SetPointsTeleop()

        rclpy.spin(controller)
        controller.destroy_node()

        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Not a valid key")

if __name__ == '__main__':
    main()
