# Real ortamda başlatma

```
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out tcpin:localhost:14550 --console
# yeni terminalde
roslaunch mavros apm.launch
#yeni terminalde
sudo ssh gokmen@192.168.43.163 // burada ifconfig ile bağlanılacak pc den adres alınarak düzeltilmesi gerekiyor
rostopic echo /mavros/vision_pose/pose
```

Bilgisayardan bilgisayara ROS bağlantısı sağlama(.bashrc yazılması gereken):
```
ifconfig ile ip öğrenip düzenlenmesi gerekiyor
# Master ros konusu yayıcak bilgisayarın ip'si
export ROS_MASTER_URI=http://192.168.182.41:11311
# Rosdan gelen verileri görüntülediğimiz bilgisayarın ip'si yazılır
export ROS_IP=192.168.182.41
```
