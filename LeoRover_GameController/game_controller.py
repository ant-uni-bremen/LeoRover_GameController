from __future__ import annotations
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import NavSatFix

from inputs import get_gamepad

import math
import threading
import time


class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

        self.publisher = None

    def connectPublisher(self, publisher :VelocityPublisher = None):
        self.publisher = publisher

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    # print(f"{event.state} / {XboxController.MAX_JOY_VAL}")
                    # event.state = 0
                    self.LeftJoystickY = round(event.state / XboxController.MAX_JOY_VAL, 3) # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = round(event.state / XboxController.MAX_JOY_VAL, 3) # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = round(event.state / XboxController.MAX_JOY_VAL, 3) # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = round(event.state / XboxController.MAX_JOY_VAL, 3) # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state
            if self.publisher is not None:
                self.publisher.sendCommand(-self.LeftJoystickY * 0.4, -self.LeftJoystickX * 1.)


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher') # type: ignore
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.4  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_linear = 0.   # for periodic publishing
        self.last_angular = 0.  # for periodic publishing

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.last_linear
        msg.angular.z = self.last_angular
        self.publisher_.publish(msg)       
        self.get_logger().info(f"Publishing to cmd_vel: {msg}")

    def sendCommand(self, linear_x :float, angular_z :float):
        self.last_linear = float(-linear_x * .4)
        self.last_angular = float(-angular_z * 1.)
        self.timer.reset()
        self._publish()

    def timer_callback(self):
        self._publish()


def main(args=None):
    rclpy.init()
    joystick = XboxController()

    velocity_publisher = VelocityPublisher()
    joystick.connectPublisher(velocity_publisher)

    rclpy.spin(velocity_publisher)

    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()