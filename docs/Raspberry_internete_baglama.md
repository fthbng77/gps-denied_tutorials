### Raspberry internete bağlama

```
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

içerisini alttakini yazın

```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=TR

network={
    ssid="WiFi_SSID"
    psk="WiFi_Password"
    key_mgmt=WPA-PSK
}
```

Manual olarak wpa_supplicant başlatma

```
sudo systemctl restart wpa_supplicant
sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf
sudo systemctl restart wpa_supplicant
```

##Raspberry her açıldığında internete bağlanmasını sağlamak için
```
sudo nano /etc/rc.local
```

içerisinde aşağıdaki içeriği kopyalayın ve Ctrl+O Ctrl+X ile kaydedin:
```
#!/bin/bash
# rc.local

# Kablosuz bağlantıyı başlat
sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf

exit 0
```

çalıştırılabilir hale getirmek için:
```
sudo chmod +x /etc/rc.local
```

rc-local servisini etkinleştirmek için:
```
sudo nano /etc/systemd/system/rc-local.service
```
dosyanın içine kopyalayın ve kaydedin:
```
[Unit]
Description=/etc/rc.local Compatibility
ConditionPathExists=/etc/rc.local

[Service]
Type=forking
ExecStart=/etc/rc.local start
TimeoutSec=0
StandardOutput=tty
RemainAfterExit=yes
SysVStartPriority=99

[Install]
WantedBy=multi-user.target

```

Sevisi etkinleştirmek için:
```
sudo systemctl enable rc-local
sudo systemctl start rc-local
```

Raspberry'i yeniden başlatarak çalışıp çalışmadığını kontrol edin:
```
sudo reboot
```
