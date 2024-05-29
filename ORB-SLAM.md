ORB-SLAM-ros kurulumu için indirilmesi gerekenler:

pangolin:
```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```

Ardından sonra ORB-Slam3 indirilmesi gerekiyor:

```
# Clone the repo:
git clone https://github.com/thien94/ORB_SLAM3.git ORB_SLAM3

# Build
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
ardından wrapper dosyası indirilicek:
```
cd ~/catkin_ws/src/
git clone https://github.com/thien94/orb_slam3_ros_wrapper.git
```

wrapper dosyasını indirdikten sonra dikkat edilmesi gereken:
CMakeList.txt dosyasındaki kısmı kendiniz için değiştirmeniz gerekiyor aksi halde `catkin build` de hata verebilir.
```
cd ~/catkin_ws/src/orb_slam3_ros_wrapper/

set(ORB_SLAM3_DIR
   /home/fatih/ORB_SLAM3
)
```
değişiklikliği yaptıktan sonra:
```
cd ~/catkin_ws/
catkin build
```
`ORB-SLAM3/Vocabulary/` klasörü içerisinde `ORBvoc.txt` dosyasını `wrapper` içerisindeki `config` dosyasına atmanız gerekiyor.
```
sudo apt install ros-noetic-hector-trajectory-server
```
Çalıştırmak için : my_config.yaml dosyası gerekiyor
Başlattıktan sonra dikkat edilmesi gerekilen bir diğer şey hareket ettirildikten sonra görüntü açılır:
```
roslaunch ORB_SLAM_gokmen.launch
```
