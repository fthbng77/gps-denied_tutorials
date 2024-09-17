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
roslaunch mavros apm.launch fcu_url:=tcp://192.168.148.195:14550
```
---------------------------------------------------------------------------------
## MissionPlanner başlat

```
cd missionplanner
mono  MissionPlanner.exe
```

---------------------------------------------------------------------------------

ayni agdaki cihazlarin ipsini sorgulamak icin
```
sudo arp-scan --localnet
```
