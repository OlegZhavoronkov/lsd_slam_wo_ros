#!/usr/bin/env sh
# --skip 5000 starts with panning
# --skip 5500 starts with zooming

./motrack --display --skip 5000 --scale 0.25 --output-scale 0.5 --video-out CAMHDA301-20160517T000000Z.mkv --csv-out CAMHDA301-20160517T000000Z.csv ../data/CAMHDA301-20160517T000000Z.mp4
