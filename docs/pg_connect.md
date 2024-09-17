## mavros verilerini dinlemek

```
sudo cat /dev/ttyUSB0
screen /dev/ttyUSB0 57600
```
---------------------------------------------------------------------------------

## MAVPROXY başlat
```
mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out tcpin:0.0.0.0:14551
```

--------------------------------------------------------------
## mavros başlat

```
roslaunch mavros apm.launch

fcu_url:=tcp://192.168.148.195:14550
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
