#!/usr/bin/env python

import sys

import rospy
from geometry_msgs.msg import Twist

import PicoBorgRev


class PiBorg(object):
    # Power settings
    __voltageIn = 12.0  # Total battery voltage to the PicoBorg Reverse
    __voltageOut = 6.0  # Maximum motor voltage

    # Setup the power limits
    __maxPower = 1.0  # if __voltageOut > __voltageIn else __voltageOut / float(__voltageIn)

    def __init__(self):

        self.__max_linear = float(rospy.get_param('max_linear', '1.5'))
        self.__max_angular = float(rospy.get_param('max_angular', '0.4'))
        self.__zero_slop = float(rospy.get_param('zero_slop', '0.01'))

        self.__pbr = PicoBorgRev.PicoBorgRev()
        # self.__pbr.i2cAddress = 0x44        # Uncomment and change the value if you have changed the board address
        self.__pbr.Init()
        if not self.__pbr.foundChip:
            boards = PicoBorgRev.ScanForPicoBorgReverse()
            if len(boards) == 0:
                print('No PicoBorg Reverse found, check that you are attached.')
            else:
                print('No PicoBorg Reverse at address %02X, but we did find boards:' % self.__pbr.i2cAddress)
                for board in boards:
                    print('    %02X (%d)' % (board, board))
                print('If you need to change the I2C address change the setup line so it is correct, e.g.')
                print('PBR.i2cAddress = 0x%02X' % (boards[0]))
            sys.exit()

        # self.__pbr.SetEpoIgnore(True)  # Uncomment to disable EPO latch, needed if you do not have a switch / jumper

        # Ensure the communications failsafe has been enabled!
        failsafe = False
        for i in range(5):
            self.__pbr.SetCommsFailsafe(True)
            failsafe = self.__pbr.GetCommsFailsafe()
            if failsafe:
                break
        if not failsafe:
            print('Board %02X failed to report in failsafe mode!' % self.__pbr.i2cAddress)
            sys.exit()

        self.__pbr.ResetEpo()
        self.__pbr.MotorsOff()
        self.__pbr.SetLed(False)

    def __update_twist(self, msg):
        # Set the motors to the new speeds
        linear = msg.linear.x / self.__max_linear
        angular = msg.angular.z / self.__max_angular

        right = linear
        left = linear

        # Turn left
        if angular > self.__zero_slop:
            if left > self.__zero_slop:
                left = left - (left * angular)
            elif left < -self.__zero_slop:
                left = left - (left * angular)
            else:
                # Tank turn left in place
                right = self.__max_linear * angular
                left = self.__max_linear * -angular

        # Turn Right
        if angular < -self.__zero_slop:
            if right > self.__zero_slop:
                right = right - (right * -angular)
            elif right < -self.__zero_slop:
                right = right - (right * -angular)
            else:
                # Tank turn right in place
                right = self.__max_linear * angular
                left = self.__max_linear * -angular

        # print("Lin: {0} Ang: {1} L: {2} R: {3} P: {4}".format(linear, angular, left, right, self.__maxPower))

        self.__pbr.SetMotor1(right * self.__maxPower)
        self.__pbr.SetMotor2(-left * self.__maxPower)

    def start(self):
        rospy.Subscriber('/cmd_vel', Twist, self.__update_twist)

    def stop(self):
        self.__pbr.MotorsOff()


if __name__ == '__main__':
    rospy.init_node('piborg-controller')

    pb = PiBorg()

    try:
        pb.start()
        rospy.spin()
    finally:
        pb.stop()
