#!/bin/bash
echo 'Start Geppetto (recorder)'
python MrGeppetto_recorder.py &
sleep 2

echo 'Start Geppetto (kinect)'
python MrGeppetto_kinect.py False &
sleep 1

echo 'Start Geppetto (myo)'
python MrGeppetto_myo.py False &
