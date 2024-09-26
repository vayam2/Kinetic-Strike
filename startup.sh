#!/bin/bash
sleep 10
export LOCALAPPDATA="LOCALAPPDATA"
screen -L -Logfile vehicle_proxy.log -S vehicle_proxy -d -m bash -c "mavproxy.py --force-connected --out=127.0.0.1:6969 --daemon"
