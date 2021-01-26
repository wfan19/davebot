#!/usr/bin/env python3
import rospy
import time

from std_msgs.msg import Float64

from qwiic_scmd import QwiicScmd

class JetbotDrive:
    def __init__(self, name):
        rospy.init_node(name)

        self.left_motor_id = 0
        self.right_motor_id = 1

        self.motor_driver = QwiicScmd()
        self.motor_driver.begin()
        time.sleep(.250)
        self.set_motor_power(self.left_motor_id, 0)
        self.set_motor_power(self.right_motor_id, 0)
        self.motor_driver.enable()

        self.left_motor_sub = rospy.Subscriber('~/left_motor', Float64, self.left_power_callback)
        self.right_motor_sub = rospy.Subscriber('~/right_motor', Float64, self.right_power_callback)

    def stop(self):
        self.set_motor_power(self.left_motor_id, 0)
        self.set_motor_power(self.right_motor_id, 0)

        self.motor_driver.disable()

    # Sets motor power given a power from -1 to 1
    def set_motor_power(self, motor_id, power):
        self.motor_driver.set_drive(motor_id, 0, self.power_to_signal(power))

    # Scales an input from between -1 and 1 to between -255 to 255
    def power_to_signal(self, power):
        signal_range = 255
        return int(min(max(power * signal_range, -signal_range), signal_range))

    # Callback that takes Float64 message with motor power in range [-1 1]
    # and passes it to the motor driver as a 0-255 signal
    def left_power_callback(self, msg):
        self.set_motor_power(self.left_motor_id, msg.data)
        rospy.loginfo("Left power command: %f" % msg.data)

    def right_power_callback(self, msg):
        self.set_motor_power(self.right_motor_id, msg.data)
        rospy.loginfo("Right power command: %f" % msg.data)

def main(args=None):
    jetbot_drive = JetbotDrive("drive")
    try:
        rospy.loginfo("[Teensy Motor Control]: Spinning node!")
        rospy.spin()
    except (KeyboardInterrupt):
        rospy.logerr("[Teensy Motor Control]: Killing node")

    # We have exited the loop so now we shutdown
    jetbot_drive.stop()

if __name__ == '__main__':
    main()
