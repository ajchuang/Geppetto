#!/bin/bash
RUN_REC='python MrGeppetto_recorder.py 1>./log/rec.log 2>./log/rec.err.log &'
RUN_KIN='python MrGeppetto_kinect.py True 1>./log/kin.log 2>./log/kin.err.log &'
RUN_MYO='python MrGeppetto_myo.py True 1>./log/myo.log 2>./log/myo.err.log &'

echo 'Start Geppetto (recorder)'
eval $RUN_REC
PID_REC=$!

# check existence
if ! ps -p $PID_REC > /dev/null
then
    echo '*** Failed to start MrGeppetto recorder ***'
    exit
else
    echo "*** Process started $PID_REC ***"
fi
sleep 2

#################################################
echo 'Start Geppetto (kinect)'
eval $RUN_KIN
PID_KIN=$!
sleep 1

# check existence
if ! ps -p $PID_KIN > /dev/null
then
    kill -9 $PID_REC
    echo '*** Failed to start MrGeppetto KINECT ***'
    exit
else
    echo "*** Process started $PID_KIN ***"
fi

#################################################
echo 'Start Geppetto (myo)'
eval $RUN_MYO
PID_MYO=$!

# check existence
if ! ps -p $PID_MYO > /dev/null
then
    kill -9 $PID_REC
    kill -9 $PID_KIN
    echo '*** Failed to start MrGeppetto MYO ***'
    exit
else
    echo "*** Process started $PID_MYO ***"
    
    # start checker
    ./check.sh $PID_KIN $PID_REC $PID_MYO &
fi

echo -n 'Stop Mr.Geppetto (y/n) :'
read decision

if [ $decision == 'y' ]; then
    kill -9 $PID_REC
    kill -9 $PID_KIN
    kill -9 $PID_MYO
fi
