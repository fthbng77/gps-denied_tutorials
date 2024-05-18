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

![Home](home.jpeg)

parametreleri ayarlayın ardupilot SITL'den/Mavproxy:

![parameters](parametres.png)

ssh_key:
```
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
```

## Simülasyonu Başlatma
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
cd ~/gps-denied-SLAM
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
