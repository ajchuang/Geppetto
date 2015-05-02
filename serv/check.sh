#!/bin/bash

while :
do
    if ! ps -p $1 > /dev/null
    then
        echo "Proc $1 failed. Kill all"
        kill $2 1>/dev/null 2>/dev/null
        kill $3 1>/dev/null 2>/dev/null
        kill $4 1>/dev/null 2>/dev/null
        kill $5 1>/dev/null 2>/dev/null
        kill $6 1>/dev/null 2>/dev/null
        exit
    fi

    if ! ps -p $2 > /dev/null 
    then
        echo "Proc $2 failed. Kill all"
        kill $1 1>/dev/null 2>/dev/null
        kill $3 1>/dev/null 2>/dev/null
        kill $4 1>/dev/null 2>/dev/null
        kill $5 1>/dev/null 2>/dev/null
        kill $6 1>/dev/null 2>/dev/null
        exit
    fi

    if ! ps -p $3 > /dev/null
    then
        echo "Proc $3 failed. Kill all"
        kill $1 1>/dev/null 2>/dev/null
        kill $2 1>/dev/null 2>/dev/null
        kill $4 1>/dev/null 2>/dev/null
        kill $5 1>/dev/null 2>/dev/null
        kill $6 1>/dev/null 2>/dev/null
        exit
    fi

    if ! ps -p $4 > /dev/null
    then
        echo "Proc $4 failed. Kill all"
        kill $1 1>/dev/null 2>/dev/null
        kill $2 1>/dev/null 2>/dev/null
        kill $3 1>/dev/null 2>/dev/null
        kill $5 1>/dev/null 2>/dev/null
        kill $6 1>/dev/null 2>/dev/null
        exit
    fi

    if ! ps -p $5 > /dev/null
    then
        echo "Proc $5 failed. Kill all"
        kill $1 1>/dev/null 2>/dev/null
        kill $2 1>/dev/null 2>/dev/null
        kill $3 1>/dev/null 2>/dev/null
        kill $4 1>/dev/null 2>/dev/null
        kill $6 1>/dev/null 2>/dev/null
        exit
    fi

    if ! ps -p $6 > /dev/null
    then
        echo "Proc $6 failed. Kill all"
        kill $1 1>/dev/null 2>/dev/null
        kill $2 1>/dev/null 2>/dev/null
        kill $3 1>/dev/null 2>/dev/null
        kill $4 1>/dev/null 2>/dev/null
        kill $5 1>/dev/null 2>/dev/null
        exit
    fi

    sleep 5
done
