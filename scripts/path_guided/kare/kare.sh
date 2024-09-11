#!/bin/bash

echo "kare2.py çalışıyor"
python kare2.py &
# Python scriptlerini paralel olarak çalıştır
echo "kare_position.py çalışıyor"
python kare_position.py &

# Tüm arka plan işlemlerinin bitmesini bekleyin
wait
echo "Tüm işlemler tamamlandı."

