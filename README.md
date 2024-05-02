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

ssh_key:
```
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
```
