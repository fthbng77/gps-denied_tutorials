## Visual SLAM
gazebo simülasyon ortamında no_gps uçuş yapmak istiyorsanız güncel parametre verilerine ulaşmak için ardupilotun güncel versiyonunu kurmanı gerekiyor.
```
git config --global http.postBuffer 524288000
git config --global http.lowSpeedLimit 0
git config --global http.lowSpeedTime 999999

cd
git clone --depth 1 https://github.com/ArduPilot/ardupilot.git

cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Visual bazlı navigasyon için Ardupilot parametre ayarları:
```
param set AHRS_EKF_TYPE 3
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 6
param set EK3_SRC1_POSZ 6
param set EK3_SRC1_VELZ 6 
param set EK3_SRC1_YAW 6
param set VISO_TYPE 1
```
GPS'i devredışı bırakmak için
```
param set GPS1_TYPE 0
```

Aracın Home ve origin noktasını ayarlamak gerekiyor:

![Home](home.jpeg)


ssh_key:
```
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
```
