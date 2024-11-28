#!/bin/bash

# Send thrust commands to multiple thrusters in parallel
gz topic -t /model/bluerov2_heavy/joint/thruster5_joint/cmd_thrust -m gz.msgs.Double -p 'data: 25.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster6_joint/cmd_thrust -m gz.msgs.Double -p 'data: 25.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster7_joint/cmd_thrust -m gz.msgs.Double -p 'data: 25.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster8_joint/cmd_thrust -m gz.msgs.Double -p 'data: 25.0' &

wait
