## X-Y Ekseninde Visual Inertial SLAM

Sabit irtifada uçan aracın x ve y ekseninde doğru bir şekilde hareket edebilmesi 

Aşağıya bakan kameralı drone ile çalışmak için launch ve worlds klasörleri içerisindeki xy dosyalarını iq_sim'de doğru dizinler içerisinde koymak gerekiyor
```
cp gps-denied_tutorials/worlds/xy.world catkin_ws/src/iq_sim/worlds/

cp gps-denied_tutorials/launch/xy.launch catkin_ws/src/iq_sim/launch/
```
başlatmak için;
```
roslaunch iq_sim xy.launch
```
