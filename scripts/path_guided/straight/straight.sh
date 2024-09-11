#!/bin/bash

echo "straight.py çalışıyor"
python straight.py &
# Python scriptlerini paralel olarak çalıştır
echo "straight_position.py çalışıyor"
python straight_position.py &

# Tüm arka plan işlemlerinin bitmesini bekleyin
wait
echo "Tüm işlemler tamamlandı."
