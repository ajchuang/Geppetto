#!/bin/bash
echo "Read log: $1"

while true; do
    clear
    tail -n 20 $1
    sleep 10
done
