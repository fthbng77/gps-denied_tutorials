## Visual SLAM
You should download the latest version of Ardupilot:
```
git config --global http.postBuffer 524288000
git config --global http.lowSpeedLimit 0
git config --global http.lowSpeedTime 999999

cd
git clone --depth 1 https://github.com/ArduPilot/ardupilot.git

cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```


kamera kalibrason verisine erişme:
```
rostopic echo /webcam/camera_info
```

![orbslam](orb-slam3.png)
