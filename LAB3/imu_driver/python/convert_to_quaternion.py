# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
from math import sin, cos, radians
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionResponse


# borrowed from wikipedia article:
# en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def handle_convert_to_quaternion(req):
    cr = cos(radians(req.roll)/2)
    sr = sin(radians(req.roll)/2)
    cp = cos(radians(req.pitch)/2)
    sp = sin(radians(req.pitch)/2)
    cy = cos(radians(req.yaw)/2)
    sy = sin(radians(req.yaw)/2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cp

    print("Returning quaternions... ")
    return convert_to_quaternionResponse(qx, qy, qz, qw)


def convert_to_quaternion_server():
    rospy.init_node('convert_to_quaternion_server')
    s = rospy.Service('convert_to_quaternion', convert_to_quaternion, handle_convert_to_quaternion)
    print("Ready to convert to quaternions!")
    rospy.spin()


if __name__ == "__main__":
    convert_to_quaternion_server()

