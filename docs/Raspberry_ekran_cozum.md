# RaspberryPi ekran sorunu çözümü

RaspberryPi monitöre bağlandığında ekran gelmiyorsa uygulanabilecek çözüm:

### config.txt dosyası düzenleme

Raspberry kurulu olan sd kartı bilgisayarınıza takarak config.txt dosyasının olduğu dizine giderek aşağıdaki komutları çalıştırarak gps-denied_tutorials/config/raspberry dizinindeki config.txt dosyası ile değştirin.

- home/fatih/gps-denied_tutorials/config/raspberry/config.txt ve /media/fatih/writable/boot/firmware dizinlerini kendi dizin yapınıza göre düzenleyin!

```shell
sudo rm -rf config.txt 
sudo cp home/fatih/gps-denied_tutorials/config/raspberry/config.txt /media/fatih/writable/boot/firmware
```

config.txt dosyasını düzenledikten sonra SD kartı ve monitörü RaspberryPi'a takarak raspberry'İ çalıştırın. 
