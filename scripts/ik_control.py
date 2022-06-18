import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from relaxed_ik.msg import EEPoseGoals


def make_pose(position, rotation):
    pose = PoseStamped()

    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]

    pose.pose.orientation.w = rotation[0]
    pose.pose.orientation.x = rotation[1]
    pose.pose.orientation.y = rotation[2]
    pose.pose.orientation.z = rotation[3]

    return pose


def make_position_goal(position):
    pos_goal = Vector3Stamped()

    pos_goal.vector.x = position[0]
    pos_goal.vector.y = position[1]
    pos_goal.vector.z = position[2]

    return pos_goal


def make_rotation_goal(rotation):
    quat_goal = QuaternionStamped()

    quat_goal.quaternion.w = rotation[0]
    quat_goal.quaternion.x = rotation[1]
    quat_goal.quaternion.y = rotation[2]
    quat_goal.quaternion.z = rotation[3]

    return quat_goal


def make_ee_pose_goal(position, rotation, seq):
    ee_pose_goals = EEPoseGoals()

    pose = Pose()

    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]

    pose.orientation.w = rotation[0]
    pose.orientation.x = rotation[1]
    pose.orientation.y = rotation[2]
    pose.orientation.z = rotation[3]

    ee_pose_goals.ee_poses.append(pose)

    ee_pose_goals.header.seq = seq

    return ee_pose_goals


class IKControl:
    def __init__(self):
        # Set the starting position of the tool because CollisionIK
        # interprets goal positions relative to that
        # Determined via utils.get_init_pose(info_file_path) in relaxed_ik_ros1
        # FIXME fix relaxed_ik so it understands goals relative to the
        # transforms in TF
        self.tool_start_position = (0.93, 0, 1.162)

        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)

        self.seq = 0

    def update(self, position, rotation):
        # Add the starting position because CollisionIK interprets goal
        # positions relative to it
        relative_position = (
            position[0] - self.tool_start_position[0],
            position[1] - self.tool_start_position[1],
            position[2] - self.tool_start_position[2],
        )

        ee_pose_goals = make_ee_pose_goal(relative_position, rotation, self.seq)
        self.ee_pose_goals_pub.publish(ee_pose_goals)

        self.seq += 1

