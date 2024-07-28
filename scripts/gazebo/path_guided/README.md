# Yol Takip

Aracın patika takibi gerçekleştirebilmesi için Gövdeye bağlı dinamik olarak değişen bir koordinat sistemi kullanması gerekiyor. 
'MAV_FRAME_BODY_OFFSET_NED' mavlink mesajı kullanılarak aracın koordinat sistemi, aracın baktığı yöne doğru dinamik olarak değişir.  
aracın yolun merkezini bulmasını sağlayan `find_center.py` ve aracın yolun merkezine doğru yaw hareketi `path_guided.py` kodu ile sağlanıyor.

Yol takibinin simülasyon ortamında başlatılması:

```
roslaunch iq_sim yolgazebo.launch
./startsitl
./yol_takip.sh
```

