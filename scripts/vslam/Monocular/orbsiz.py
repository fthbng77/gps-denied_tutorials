import rospy
from geometry_msgs.msg import PoseStamped

scale_factor = 27.7 * (1.2)
latest_pose = None

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
    return 1 if current_position == 0.99999 else 0.99999

def update_pose(event):
    global latest_pose, x_position, y_position
    if latest_pose is not None:
        transformed_pose = transform_pose(latest_pose, x_position, y_position)
        pub.publish(transformed_pose)
        x_position = toggle_position(x_position)
        y_position = toggle_position(y_position)

def pose_callback(data):
    global latest_pose
    latest_pose = data

rospy.init_node('orb_slam_to_mavros_transform')

x_position = 0.99999
y_position = 0.99999

sub = rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, pose_callback)
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
timer = rospy.Timer(rospy.Duration(0.033), update_pose)
rospy.spin()