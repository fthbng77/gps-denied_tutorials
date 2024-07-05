# Tutorials

[Githuba dosya gönderme ve çekme](docs/github_pull_push.md)

[ORBSLAM3 Kurulumu](docs/ORB-SLAM.md)

[No-GPS parametreleri](docs/Visual_navigation_parametres.md)

[Simülasyon Ortamda No-GPS](docs/Simulation_No-GPS.md)

<<<<<<< HEAD
parametreleri ayarlayın ardupilot SITL'den/Mavproxy:

![parameters](imgs/parametres.png)


## Simülasyonda Başlatma
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


## Real ortamda başlatma

```
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out tcpin:localhost:14550
# yeni terminalde
roslaunch mavros apm.launch
#yeni terminalde
sudo ssh gokmen@192.168.43.163 // burada ifconfig ile bağlanılacak pc den adres alınarak düzeltilmesi gerekiyor
rostopic echo /mavros/vision_pose/pose
```
=======
[Real Ortamda No-GPS](docs/Real_Bo-GPS.md)
>>>>>>> 063bb70f8cc120e8be33fe32c0fd812555901a4a
