#!/bin/bash

echo "Segmentation-ros-distance.py arka planda çalıştırılıyor..."
python find_center.py &
# Python scriptlerini paralel olarak çalıştır
echo "Calisankod.py arka planda çalıştırılıyor..."
python path_guided.py &

# Tüm arka plan işlemlerinin bitmesini bekleyin
wait
echo "Tüm işlemler tamamlandı."

