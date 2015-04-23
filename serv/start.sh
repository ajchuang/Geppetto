#!/bin/bash
echo 'Start Geppetto (recorder)'
python MrGeppetto_recorder.py   $1 4002 &

echo 'Start Geppetto (kinect)'
python MrGeppetto_kinect.py     $1 4000 &

echo 'Start Geppetto (myo)'
python MrGeppetto_myo.py        $1 4001 &
