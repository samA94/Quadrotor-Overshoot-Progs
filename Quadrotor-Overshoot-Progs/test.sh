#!/bin/bash
while true; do
    if test -e /dev/ttyUSB0; then
        #echo Hello > /dev/pts/27
        #cat /dev/ttyUSB0 > /dev/pts/27

        #test_Var=$( cat /dev/ttyUSB0)
        #echo $test_Var > /dev/ttyUSB4
        #echo $test_Var > /dev/ttyUSB3
        #echo $test_Var > /dev/pts/27

        cat /dev/ttyUSB0 | tee /dev/pts/27 /dev/ttyUSB3 /dev/ttyUSB4

    #else
        #echo Fail > /dev/pts/27
    fi
done


