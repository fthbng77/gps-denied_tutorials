
```
sudo apt update
sudo apt install nmap
```

Sonuçlarda en düşük gecikme süresi olan taramayı yaptığınız wifi
_gateway ile başlayan routerdır.

```
nmap -sn 192.168.1.0/24 # 192.168.1 bu kısma kendi ip adresininz basını yazın
```

Wifi sinyal gücünü dBm cinsinden öğrenme:
```
iwconfig wlo1
```
