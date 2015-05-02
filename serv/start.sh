#!/bin/bash
TEST_MOD='False'
if [ "$1" == "True" ]; then
    TEST_MOD='True'
fi

echo "!!! Test mode = $TEST_MOD !!!"

RUN_REC="python MrGeppetto_recorder.py 1>./log/rec.log 2>./log/rec.err.log &"
RUN_KIN="python MrGeppetto_kinect.py $TEST_MOD 1>./log/kin.log 2>./log/kin.err.log &"
RUN_MYO="python MrGeppetto_myo.py $TEST_MOD 1>./log/myo.log 2>./log/myo.err.log &"
RUN_VCM="python MrGeppetto_vcmd.py $TEST_MOD 1>./log/vcm.log 2>./log/vcm.err.log &"
RUN_CPY="python CameraProxy.py &" #1>./log/cpy.log 2>./log/cpy.err.log &"
RUN_BRG="python BridgeServer.py &" #1>./log/brg.log 2>./log/brg.err.log &"

#################################################
eval $RUN_REC
PID_REC=$!
sleep 2

#################################################
eval $RUN_KIN
PID_KIN=$!
sleep 1

#################################################
eval $RUN_MYO
PID_MYO=$!

#################################################
eval $RUN_CPY
PID_CPY=$!

#################################################
eval $RUN_BRG
PID_BRG=$!

#################################################
eval $RUN_VCM
PID_VCM=$!

# start checker
./check.sh $PID_KIN $PID_REC $PID_MYO $PID_CPY $PID_BRG $PID_VCM &

echo -n 'Stop Mr.Geppetto (y/n) :'
read decision

if [ "$decision" == 'y' ]; then
    kill -9 $PID_REC
    kill -9 $PID_KIN
    kill -9 $PID_MYO
    kill -9 $PID_CPY
    kill -9 $PID_BRG
    kill -9 $PID_VCM
fi
