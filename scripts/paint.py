import math
import numpy as np
import rospy
import tf
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath


def dist(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2))


def paint_paths(ik_ctrl, paths):
    points = []

    tf_listener = tf.TransformListener()
    paint_path_pub = rospy.Publisher('/paint_path', NavPath, queue_size=5, latch=True)

    rospy.sleep(0.5)

    canvas_position, canvas_rotation = tf_listener.lookupTransform('base_link', 'canvas', rospy.Time.now())
    print('Canvas is at', canvas_position)

    path_msg = NavPath()
    path_msg.header.frame_id = 'canvas'
    path_msg.header.stamp = rospy.Time.now()

    seq = 0
    for path in paths:
        for x, y in path:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = path_msg.header.frame_id
            pose_msg.header.seq = seq
            pose_msg.header.stamp = rospy.Time.now()
            seq += 1

            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0

            pose_msg.pose.orientation.w = 1
            pose_msg.pose.orientation.x = 0
            pose_msg.pose.orientation.y = 0
            pose_msg.pose.orientation.z = 0

            path_msg.poses.append(pose_msg)

            points.append((x, y, 0))

    paint_path_pub.publish(path_msg)

    last_x = points[0][0]
    last_y = points[0][1]
    last_z = points[0][2]

    for x, y, z in points:
        d = dist(last_x, last_y, last_z, x, y, z)
        steps = int(d / 0.001)

        for ratio in np.linspace(0, 1, steps):
            ix = last_x + (x - last_x) * ratio
            iy = last_y + (y - last_y) * ratio
            iz = last_z + (z - last_z) * ratio

            ik_ctrl.update((ix + canvas_position[0], iy + canvas_position[1], iz + canvas_position[2]), (1, 0, 0, 0))
            rospy.sleep(0.01)

        last_x = x
        last_y = y
        last_z = z

    rospy.spin()