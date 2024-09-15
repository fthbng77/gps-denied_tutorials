import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation

# Verileri saklayacağımız listeler
local_position_data = []
vision_pose_data = []

# ROS callback fonksiyonları
def local_position_callback(data):
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    local_position_data.append([x, y, z])

def vision_pose_callback(data):
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    vision_pose_data.append([x, y, z])

# ROS düğümünü başlat
rospy.init_node('real_time_pose_comparison', anonymous=True)

# ROS subscriber'ları başlat
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_position_callback)
rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, vision_pose_callback)

# 3D bir grafik oluştur
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Eksenleri etiketle
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

# Animasyon fonksiyonu
def update_graph(frame):
    # Temizle ve eksenleri tekrar çiz
    ax.cla()
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    # Local Position (mavros local_position) verilerini ekle
    if len(local_position_data) > 0:
        local_position_array = np.array(local_position_data)
        ax.plot(local_position_array[:,0], local_position_array[:,1], local_position_array[:,2], color='red', label='Local Position')

    # Vision Pose (mavros vision_pose) verilerini ekle
    if len(vision_pose_data) > 0:
        vision_pose_array = np.array(vision_pose_data)
        ax.plot(vision_pose_array[:,0], vision_pose_array[:,1], vision_pose_array[:,2], color='green', label='Vision Pose')

    # Başlık ve efsane ekle
    plt.title('Local Position vs Vision Pose')
    plt.legend()

# Animasyonu başlat
ani = FuncAnimation(fig, update_graph, interval=100)

# Grafik gösterimi
plt.show()

# ROS döngüsü
rospy.spin()

