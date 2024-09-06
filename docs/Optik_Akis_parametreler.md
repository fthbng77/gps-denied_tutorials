## Optik Akış parametreler


Kaynak olarak [Inflight Optical Flow Calibration](https://www.youtube.com/watch?v=Crx97v1bwWo "Optik Akis Kalibrasyonu") 

```
# GPS:
param set EK3_SRC1_POSXY=3
param set EK3_SRC1_VELXY=3, 
param set EK3_SRC1_VELZ=3 

# OptFlow:
param set EK3_SRC2_POSXY=0, 
param set EK3_SRC2_VELXY=5, 
param set EK3_SRC2_VELZ = 0
param set EFK3_OPTIONS = 0 (Do not Fuse All Velocities) tüm hızları karşılaştırma
```
