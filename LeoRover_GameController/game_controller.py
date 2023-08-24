from __future__ import annotations
from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import NavSatFix

from inputs import devices, get_gamepad, UnpluggedError

import math
import threading
import time
import sys


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

        self.publisher :DelayInjector = None
        self.joystick_disconnected = False

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def connectPublisher(self, publisher :VelocityPublisher = None):
        self.publisher = publisher


    def _rescan(self, error_message :str):
        if not self.joystick_disconnected:
            print(error_message)
            self.joystick_disconnected = True
            devices.gamepads = []

        # The inputs-library unfortunately is pretty bad and unmaintained.
        # Therefore, some internals hacking to make it look for "new" devices.
        devices._raw = []
        devices.keyboards = []
        devices.mice = []
        devices.gamepads = []
        devices.other_devices = []
        devices.all_devices = []
        devices.leds = []
        devices.microbits = []
        devices.xinput = None
        devices.xinput_dll = None

        try:
            devices._post_init()
        except Exception as e:
            pass
        time.sleep(1)


    def _monitor_controller(self):  
        while True:
            try:
                events = get_gamepad()
            except UnpluggedError:
                self._rescan("No gamepad connected!")
                continue
            except OSError:
                self._rescan("Gamepad removed!")
                continue
            except Exception:
                continue

            if self.joystick_disconnected:
                print("Gamepad reconnected")
                self.joystick_disconnected = False

            for event in events:
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
                    sendCommand = True
                    if event.state == True:
                        if self.publisher.delay_s >= 0:
                            self.publisher.delay_s = self.publisher.delay_s - 0.1
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                    sendCommand = True
                    if event.state == True:
                        if self.publisher.delay_s <= 5:
                            self.publisher.delay_s = self.publisher.delay_s + 0.1
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
                self.publisher.sendCommand(-self.LeftJoystickY, -self.LeftJoystickX)


class Command(dict):
    def __init__(self):
        self.linear :int = 0
        self.angular :int = 0
        self.timeToSend :int = 0  
             

class DelayInjector(Node):
    def __init__(self):
        super().__init__('delay_injector') # type: ignore
        self.publisher = None
        self.timer = self.create_timer(100., self.timer_callback)
        self.timer.cancel()
        self.timer_running = False
        self.queue :List[Command]= []

        self.delay_s :int = 0.5
        self.delay_inc :int = 0
        self.delay_dec :int = 0 
        


    def connectPublisher(self, publisher :VelocityPublisher = None):
        self.publisher = publisher


    def sendCommand(self, linear_x :float, angular_z :float):
        # self.publisher.sendCommand(linear_x, angular_z) # Currently no delay, just send the command to the real publisher
        cmd = Command()
        cmd.linear = linear_x
        cmd.angular = angular_z
        
        cmd.timeToSend = time.time() + self.delay_s
        
        self.queue.append(cmd)
        
        if not self.timer_running:
            self.timer.timer_period_ns = (cmd.timeToSend - time.time()) * 1000000000
            self.timer.reset()
            self.timer_running = True
        

    def timer_callback(self):
        cmd = self.queue.pop(0)
        self.publisher.sendCommand(cmd.linear, cmd.angular)
        
        if len(self.queue) == 0:
            self.timer.cancel()
            self.timer_running = False
        else:
            self.timer.timer_period_ns = (cmd.timeToSend - time.time()) * 1000000000
            self.timer.reset()



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

    def sendCommand(self, linear_x :float, angular_z :float, ):
        self.last_linear = float(linear_x * .4)
        self.last_angular = float(angular_z * 1.)
        self.timer.reset()
        self._publish()

    def timer_callback(self):
        self._publish()


def main(args=None):
    rclpy.init()
    joystick = XboxController()
    delay_injector = DelayInjector()
    velocity_publisher = VelocityPublisher()

    delay_injector.connectPublisher(velocity_publisher)
    joystick.connectPublisher(delay_injector)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(velocity_publisher)
    executor.add_node(delay_injector)
    executor.spin()

    # rclpy.spin(velocity_publisher)

    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()