#!/bin/bash
TEST_MOD='False'
if [ "$1" == "True" ]; then
    TEST_MOD='True'
fi

echo "!!! Test mode = $TEST_MOD !!!"

RUN_REC="python MrGeppetto_recorder.py 1>./log/rec.log 2>./log/rec.err.log &"
RUN_KIN="python MrGeppetto_kinect.py $TEST_MOD 1>./log/kin.log 2>./log/kin.err.log &"
RUN_MYO="python MrGeppetto_myo.py $TEST_MOD 1>./log/myo.log 2>./log/myo.err.log &"
RUN_CPY="python CameraProxy.py 1>./log/cpy.log 2>./log/cpy.err.log"
RUN_BRG="python BridgeServer.py 1>./log/brg.log 2>./log/brg.err.log"

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
    
fi

#################################################
echo 'Start Geppetto (camera proxy)'
eval $RUN_CPY
PID_CPY=$!

# check existence
if ! ps -p $PID_CPY > /dev/null
then
    kill -9 $PID_REC
    kill -9 $PID_KIN
    kill -9 $PID_MYO
    echo '*** Failed to start MrGeppetto (cam proxy) ***'
    exit
else
    echo "*** Process started $PID_CPY ***"
fi

#################################################
echo 'Start Geppetto (bridge server)'
eval $RUN_BRG
PID_BRG=$!

# check existence
if ! ps -p $PID_BRG > /dev/null
then
    kill -9 $PID_REC
    kill -9 $PID_KIN
    kill -9 $PID_MYO
    kill -9 $PID_CPY
    echo '*** Failed to start MrGeppetto (bridge server) ***'
    exit
else
    echo "*** Process started $PID_BRG ***"
fi

# start checker
./check.sh $PID_KIN $PID_REC $PID_MYO $PID_CPY $PID_BRG &

echo -n 'Stop Mr.Geppetto (y/n) :'
read decision

if [ "$decision" == 'y' ]; then
    kill -9 $PID_REC
    kill -9 $PID_KIN
    kill -9 $PID_MYO
    kill -9 $PID_CPY
    kill -9 $PID_BRG
fi
