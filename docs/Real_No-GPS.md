# Real ortamda başlatma

```
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out tcpin:localhost:14550
# yeni terminalde
roslaunch mavros apm.launch
#yeni terminalde
sudo ssh gokmen@192.168.43.163 // burada ifconfig ile bağlanılacak pc den adres alınarak düzeltilmesi gerekiyor
rostopic echo /mavros/vision_pose/pose
```
