#!/bin/bash

trap ctrl_c INT


function ctrl_c() {
        echo "** Trapped CTRL-C"
	kill %1
	kill %2
}


JLinkExe -Device NRF52832_XXAA -Speed 4000 -If SWD -AutoConnect 1 &
sleep 2
JLinkRTTClient
kill %1
