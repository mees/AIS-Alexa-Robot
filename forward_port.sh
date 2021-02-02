#!/bin/bash
trap : SIGTERM SIGINT
# # kill the whole process tree on exit
# trap "echo caught signal;exit" INT TERM HUP QUIT
# trap "kill 0; echo killed processes" EXIT
socat TCP4-LISTEN:9000,fork,range=172.18.0.1/16 TCP4:localhost:9090 &
PID_SOCAT=$!
echo "Your local port is being forwarded. Press Ctrl+C to stop forwarding"
wait $PID_SOCAT
if [[ $? -gt 128 ]]
then
    kill $PID_SOCAT
    kill 0 # just to be sure, kill complete process tree
fi
echo "Port forwarding ended. You might have to wait for the TCP connections to time out before another process can take over the port."
