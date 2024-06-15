# Simülasyonda Başlatma

ros ile gazebo başlatma:
```
roslaunch iq_sim yolgazebo.launch 
```
orbslamı başlatma:
```
cp  ~/gps-denied-SLAM/launch/euroc_mono.launch ~/catkin_ws/src/orb_slam3_ros/launch/euroc_mono.launch 
roslaunch orb_slam3_ros euroc_mono.launch
```
GPS'i devredışı bırakmak için:
```
cd ~/gps-denied-SLAM/scripts
python orb_slam_to_mavros.py  
```
Gazebo üzerinden drone manuel olarak hareket ettirilerek ORBSlam aktifleştirilir.

Ardından

SITL'i başlatma:
```
./startsitl.sh
```
apm.launch başlatma:
```
roslaunch iq_sim apm.launch
```
konum bilgilerini görüntülemek için:
```
 rostopic echo /mavros/vision_pose/pose
```
