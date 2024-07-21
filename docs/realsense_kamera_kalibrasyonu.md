dynamic calibrator kurulumu:
```
sudo apt-get update
```
libusb-1.0 kurulumu:
```
sudo apt-get install libusb-dev libusb-1.0-0-dev
```
libglfw kurulumu:
```
sudo apt-get install libglfw3 libglfw3-dev
```
freeglut kurulumu:
```
sudo apt-get install freeglut3 freeglut3-dev
```
intel sunucusunu repo listesine ekleyin:
```
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```
Sunucunun genel anahtarını kaydedin:
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key
F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver
hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```
Mevcut depoların ve paketlerin listesini yenileyin:
```
sudo apt-get update
```
intel RealSense Dinamik Kalibratörü içeren librscalibrationtool paketini yükleyin:
```
sudo apt-get install librscalibrationtool
sudo dpkg -i librscalibrationtool_2.13.1.0_amd64.deb
```
Paket Kurulumunun Kontrol Edin:
```
sudo dpkg -L librscalibrationtool
```
/usr
|…….bin
|
|….. Intel.Realsense.DynamicCalibrator
|
|…. Intel.Realsense.CustomRW
|
|----- share
|
|---- doc
|
|….. librscalibrationtool
|
|---- attributions.txt
|
|---- readme.txt
|
|---- changelog
|
|---- copyright
|
|---- License.txt
|
|---- README.md
|
|
|
|---- target
|
|
|---- print-target-fixed-width.pdf
|
|
|
|---- api
|
|---- DynamicCalibrationAPI-Linux-2.13.1.0.tar.gz

kurulum:
```
sudo apt-get install librealsense2-dkms
```
TEST KURULUMU:
aşağıdaki komutu çalıştırarak dinamik kalibrasyon aracı sürümünü kontrol edin:
Kalibrasyon aracının yürütülebilir dosyaları /usr/bin altında bulunur.
Örneğin:
Araç sürümünü ekrana yazdırmak için:
```
/usr/bin/Intel.Realsense.DynamicCalibrator -v
```
Cihaz bilgilerini ekrana yazdırmak için sisteme bir kamera cihazı bağlayın ve aşağıdaki komutu yazın:
```
/usr/bin/Intel.Realsense.DynamicCalibrator -list
```
Cihaz kalibrasyon verilerini ekrana yazdırmak için aşağıdaki komutu yürütün:
```
/usr/bin/Intel.Realsense.CustomRW -r
```
Kalibratörü denemek için aşağıdaki komutu yürütün:
```
/usr/bin/Intel.Realsense.DynamicCalibrator
```
yada bu:
```
realsense-dynamic-calibrator
```

telefona internetten dynamic calibrator uygulaması indir veya kağıt kullanarak kalibrasyon yap
