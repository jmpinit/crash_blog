#!/usr/bin/env python

import rospy
from ik_control import IKControl
from paint import paint_paths


if __name__ == '__main__':
    rospy.init_node('paint_square')
    ik_ctrl = IKControl()

    square_path = [
        (-0.25, -0.25),
        (0.25, -0.25),
        (0.25, 0.25),
        (-0.25, 0.25),
        (-0.25, -0.25),
    ]

    try:
        paint_paths(ik_ctrl, [square_path])
    except rospy.ROSInterruptException:
        pass
