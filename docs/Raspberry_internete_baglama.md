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
