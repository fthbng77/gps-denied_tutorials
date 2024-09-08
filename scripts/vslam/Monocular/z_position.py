import rospy
from geometry_msgs.msg import PoseStamped

scale_factor = 27.7 * (1.2)

def transform_pose(orb_pose, x_pos, y_pos):
    mavros_pose = PoseStamped()
    mavros_pose.header = orb_pose.header
    mavros_pose.pose.position.z = orb_pose.pose.position.z * scale_factor
    mavros_pose.pose.position.x = x_pos
    mavros_pose.pose.position.y = y_pos
    mavros_pose.pose.orientation.x = 0
    mavros_pose.pose.orientation.y = 0
    mavros_pose.pose.orientation.z = 0
    mavros_pose.pose.orientation.w = 1
    return mavros_pose

def toggle_position(current_position):
    return 1 if current_position == 0.09999 else 0.09999

def pose_callback(data):
    global x_position, y_position
        
    transformed_pose = transform_pose(data, x_position, y_position)
    rospy.loginfo("Transformed Data to MAVROS: Position: ({}, {}, {})".format(
        transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z))
    
    pub.publish(transformed_pose)
    
    x_position = toggle_position(x_position)
    y_position = toggle_position(y_position)

rospy.init_node('orb_slam_to_mavros_transform')

x_position = 0.09999
y_position = 0.09999

sub = rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, pose_callback)
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
rospy.Timer(rospy.Duration(1), pose_callback, oneshot=False)

rospy.spin()

