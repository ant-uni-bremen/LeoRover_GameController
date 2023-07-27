from __future__ import annotations
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

from inputs import get_gamepad

import math
import threading
import time
import random
from typing import List
import queue


# ToDo: Right loss button debouncing


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
        self.last_lz = 0
        self.last_rz = 0

    def connectPublisher(self, publisher :VelocityPublisher = None):
        self.publisher = publisher

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                # print(event.code)
                sendCommand = False
                if event.code == 'ABS_Y':
                    # print(f"{event.state} / {XboxController.MAX_JOY_VAL}")
                    # event.state = 0
                    self.LeftJoystickY = round(event.state / XboxController.MAX_JOY_VAL, 3) # normalize between -1 and 1
                    sendCommand = True
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = round(event.state / XboxController.MAX_JOY_VAL, 3) # normalize between -1 and 1
                    sendCommand = True
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = round(event.state / XboxController.MAX_JOY_VAL, 3) # normalize between -1 and 1
                    sendCommand = True
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = round(event.state / XboxController.MAX_JOY_VAL, 3) # normalize between -1 and 1
                    sendCommand = True

                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                    if self.LeftTrigger > .9:
                        self.publisher.loss_rate = round( max(self.publisher.loss_rate - .1, 0), 1)
                    self.last_lz = self.LeftTrigger
                    print(f"New loss rate: {self.publisher.loss_rate}")
                    
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                    if self.RightTrigger > .9:
                        self.publisher.loss_rate = round( min(self.publisher.loss_rate + .1, 1), 1)
                    self.last_rz = self.RightTrigger
                    print(f"New loss rate: {self.publisher.loss_rate}")

                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                    if self.LeftBumper:
                        self.publisher.delay_s = max(self.publisher.delay_s - 0.1, 0)
                    print(f"New delay: {self.publisher.delay_s}")

                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                    if self.RightBumper:
                        self.publisher.delay_s = min(self.publisher.delay_s + 0.1, 5)
                    print(f"New delay: {self.publisher.delay_s}")

                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                    if self.Y:
                        self.publisher.delay_s = 0
                        self.publisher.loss_rate = 0
                        self.publisher.nextCommands = []
                        self.publisher.sendCommand(0., 0.)
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
            if sendCommand and self.publisher is not None:
                self.publisher.sendCommand(-self.LeftJoystickY * 0.4, -self.LeftJoystickX * 1.)



class VelocityPublisher(Node):

    class DelayedCommand(dict):
        def __init__(self):
            self.linear = 0
            self.angular = 0
            self.timeToSend = 0     # ToDo: Maxbe switch to time.time_ns() ?

    def __init__(self):
        super().__init__('velocity_publisher') # type: ignore
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.4  # seconds
        self.keep_alive_timer = self.create_timer(timer_period, self.timer_callback)
        self.send_timer = self.create_timer(10, self.send_timer_callback)
        self.send_timer.cancel()
        self.last_linear = 0.   # for periodic publishing
        self.last_angular = 0.  # for periodic publishing
        # self.nextCommands :List[VelocityPublisher.DelayedCommand] = []
        self.command_queue :queue.Queue[VelocityPublisher.DelayedCommand]= queue.Queue()
        self.delay_s = 0
        self.loss_rate = 0
        

    def _publish(self, linear_x :float|None = None, angular_z :float|None = None):
        msg = Twist()

        if linear_x is not None:
            self.last_linear = linear_x
        if angular_z is not None:
            self.last_angular = angular_z

        msg.linear.x = self.last_linear
        msg.angular.z = self.last_angular

        self.publisher_.publish(msg)       
        # self.get_logger().info(f"Publishing to cmd_vel: {msg}")


    def sendCommand(self, linear_x :float, angular_z :float):

        if random.random() < self.loss_rate:
            print(f"Lost command: {linear_x, angular_z}")
            return

        print(f"SendCommand: {linear_x, angular_z}")

        if self.delay_s == 0:
            self.keep_alive_timer.reset()
            self._publish(linear_x, angular_z)
        else:
            cmd = VelocityPublisher.DelayedCommand()
            cmd.linear = float(linear_x)
            cmd.angular = float(angular_z)
            cmd.timeToSend = time.time() + self.delay_s

            # self.nextCommands.append(cmd)
            self.command_queue.put_nowait(cmd)

            # if len(self.nextCommands) == 1:     # Restart the timer to send that only queued command, as it is not running after emptying the queue
            if self.command_queue.qsize() == 1:     # Restart the timer to send that only queued command, as it is not running after emptying the queue
                self.send_timer.timer_period_ns = self.delay_s * 1e9
                self.send_timer.reset()            


    def send_timer_callback(self):
        self.keep_alive_timer.reset()

        print(f"Send timer callback")
        # cmd = self.nextCommands.pop(0)
        cmd = self.command_queue.get_nowait()
        self.last_angular = cmd.angular
        self.last_linear = cmd.linear
        self._publish()
        
        # if len(self.nextCommands) == 0:
        if self.command_queue.empty():
            self.send_timer.cancel()
        else:
            # self.send_timer.timer_period_ns = (self.nextCommands[0].timeToSend - cmd.timeToSend) * 1e9
            self.send_timer.timer_period_ns = (self.command_queue.queue[0].timeToSend - cmd.timeToSend) * 1e9
            self.send_timer.reset()


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