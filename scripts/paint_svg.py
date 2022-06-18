#!/usr/bin/env python

import os
import rospy
from ik_control import IKControl
from paint import paint_paths
from svg import path_points_from_svg, center_and_fit_paths


if __name__ == '__main__':
    rospy.init_node('paint_svg')
    ik_ctrl = IKControl()

    dir_path = os.path.dirname(os.path.abspath(__file__))
    asset_path = os.path.join(dir_path, '../assets')
    smiley_path = os.path.join(asset_path, 'images/Smiley.svg')

    smiley_paths = center_and_fit_paths(path_points_from_svg(smiley_path), 1, 1)

    try:
        paint_paths(ik_ctrl, smiley_paths)
    except rospy.ROSInterruptException:
        pass
