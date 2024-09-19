## mavros verilerini dinlemek

pymavlink kodu oluştururken aşağıdaki bağlanma yöntemi ile `connection = mavutil.mavlink_connection('tcp:127.0.0.1:14552')` olması gerekiyor.

```
sudo cat /dev/ttyUSB0
screen /dev/ttyUSB0 57600
```
---------------------------------------------------------------------------------

## MAVPROXY başlat
```
 mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out tcpin:0.0.0.0:14550 --out tcpin:0.0.0.0:14551 --out tcpin:0.0.0.0:14552
```

--------------------------------------------------------------
## mavros başlat

```
roslaunch mavros apm.launch fcu_url:=tcp://127.0.0.1:14550
```
---------------------------------------------------------------------------------
## MissionPlanner başlat

```
cd missionplanner
mono  MissionPlanner.exe
```

---------------------------------------------------------------------------------

## Ayni ağdaki cihazlarin ipsini sorgulama
```
sudo arp-scan --localnet
```

## Raspberry ile bağlanıp kamera çalıştırma bashrcye
bashrcye kayıt edilmesi gerekiyor.
```
# export ROS_MASTER_URI=http://192.168.182.41:11311 
# export ROS_IP=192.168.182.41
```

```
sudo ssh gokmen@192.168.43.163 # ifconfig ile öğreniliyor. 
cd gps-denied_tutorials/launch
roslaunch usb_cam-test.launch
```
