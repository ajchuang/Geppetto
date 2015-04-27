#!/bin/bash

while :
do
    if ! ps -p $1 > /dev/null
    then
        kill $2 1>/dev/null 2>/dev/null
        kill $3 1>/dev/null 2>/dev/null
        exit
    fi

    if ! ps -p $2 > /dev/null 
    then
        kill $1 1>/dev/null 2>/dev/null
        kill $2 1>/dev/null 2>/dev/null
        exit
    fi

    if ! ps -p $3 > /dev/null
    then
        kill $1 1>/dev/null 2>/dev/null
        kill $2 1>/dev/null 2>/dev/null
        exit
    fi

    sleep 5
done
