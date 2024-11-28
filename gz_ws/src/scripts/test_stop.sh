#!/bin/bash

# Send thrust commands to multiple thrusters in parallel
gz topic -t /model/bluerov2_heavy/joint/thruster0_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster5_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster6_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0.0' &
gz topic -t /model/bluerov2_heavy/joint/thruster7_joint/cmd_thrust -m gz.msgs.Double -p 'data: 0.0' &

wait
