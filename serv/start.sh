#!/bin/bash
echo 'Start Geppetto (recorder)'
python MrGeppetto_recorder.py &

echo 'Start Geppetto (kinect)'
python MrGeppetto_kinect.py &

echo 'Start Geppetto (myo)'
python MrGeppetto_myo.py &
