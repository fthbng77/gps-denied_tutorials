## Visual SLAM
gazebo simülasyon ortamında no_gps uçuş yapmak istiyorsanız güncel parametre verilerine ulaşmak için ardupilotun güncel versiyonunu kurmanı gerekiyor.
```
git config --global http.postBuffer 524288000
git config --global http.lowSpeedLimit 0
git config --global http.lowSpeedTime 999999

cd
git clone --depth 1 https://github.com/ArduPilot/ardupilot.git

cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Visual bazlı navigasyon için Ardupilot parametre ayarları:
```
param set AHRS_EKF_TYPE 3
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 6
param set EK3_SRC1_POSZ 6
param set EK3_SRC1_VELZ 6 
param set EK3_SRC1_YAW 6
param set VISO_TYPE 1
```
GPS'i devredışı bırakmak için
```
param set GPS1_TYPE 0
```

Aracın Home ve origin noktasını ayarlamak gerekiyor:

![Home](imgs/home.jpeg)

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
