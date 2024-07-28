#!/bin/bash

python gray_image_stabil.py &
python orb_slam_to_mavros.py &

wait
echo "Tüm işlemler tamamlandı."

