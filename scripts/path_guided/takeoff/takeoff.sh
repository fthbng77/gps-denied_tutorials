#!/bin/bash

echo "takeoff.py çalışıyor"
python takeoff.py &
# Python scriptlerini paralel olarak çalıştır
echo "takeoff_position.py çalışıyor"
python takeoff_position.py &

# Tüm arka plan işlemlerinin bitmesini bekleyin
wait
echo "Tüm işlemler tamamlandı."
