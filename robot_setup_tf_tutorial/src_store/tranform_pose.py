# Transform a given input pose from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped


def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


# Test Case
rospy.init_node("transform_test")

my_pose = Pose()
my_pose.position.y = -0.243546366692
my_pose.position.x = 0.269357979298
my_pose.position.z = 0.00781712215394
my_pose.orientation.x = -0.375593334436 
my_pose.orientation.y = -0.580118715763
my_pose.orientation.z = -0.395113497972
my_pose.orientation.w = 0.605208396912

# kq
# position: 
#   x: 0.269357979298
#   y: -0.243546366692
#   z: 0.212817122154
# orientation: 
#   x: -0.375593328953
#   y: -0.580118707294
#   z: -0.395113492204
#   w: 0.605208405747

# my_pose.position.y =-0.08755 
# my_pose.position.x =-0.0184349
# my_pose.position.z =-0.003777
# my_pose.orientation.x =0.63418 
# my_pose.orientation.y =-0.308281
# my_pose.orientation.z =0.63336
# my_pose.orientation.w =0.31879
# --> ket qua : 
            # position: 
            # x: -0.0184349
            # y: -0.08755
            # z: 0.201223
            # orientation: 
            # x: 0.634181300682
            # y: -0.308280367727
            # z: 0.633358701002
            # w: 0.318789346174

transformed_pose = transform_pose(my_pose, "t265_link", "robot_link")

print(transformed_pose)