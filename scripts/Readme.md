# VSLAM için bilinmesi gerekenler
Realsense D435i kamera için kalibrasyon SDK `realsense-viewer` içerisinde yapılıyor ardından kamera içerisine kayıt ediliyor

kamera kalibrason verisine erişme: 

``` 
rostopic echo /webcam/camera_info
```

NoGPS uçuş için ORBSLAM3 gelen /orb_slam3/camera_pose konusu kameranın hareketini bize veriyor ama bu direkt olarak kullanılamıyor 
onu MAVROS koordinat çervesine çevirmemiz gerekiyor ardından ölçeklendirmek gerekiyor ölçeklendirmek için bildiğimiz bir nesnenin ölçüsüne göre 
kameranın ölçüsünü ayarlamamız gerekiyor.

```
python orb_slam_to_mavros.py
```

 
