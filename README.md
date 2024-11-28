# BlueROV2 simulation SITL in Gazebo

The following repository offers the BlueROV2 simulation SITL with GazeboSim. Users can plan complex missions using ROS 2, ArduPilot or QGroundControl by defining waypoints and survey grids.

SITL allows you to simulate the vehicle hardware and firmware ([ArduSub](https://www.ardusub.com/) ) on your host directly.


![image](https://github.com/oceansystemslab/bluerov2_ardupilot_SITL/assets/30973337/01a13faf-5d00-45c2-95a7-72d45fc0b9f9)


## Prerequisites

- Download and Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) (optional).
- Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to support Docker to access GPU (required).
- Repository has been tested on: Ubuntu 22.04, Ubuntu 24.04, ArchLinux (Kernel 6.8).


## Build

```bash
git clone https://github.com/markusbuchholz/gazebosim_bluerov2_ardupilot_sitl.git

cd gazebosim_bluerov2_ardupilot_sitl/bluerov2_ardupilot_SITL/docker

sudo ./build.sh

```

## Build in Docker

Adjust in ```run.sh```.

```bash
local_gz_ws="/home/markus/bluerov2_ardupilot_SITL/gz_ws"
local_SITL_Models="/home/markus/bluerov2_ardupilot_SITL/SITL_Models"
```

```bash
sudo ./run.sh

colcon build

source install/setup.bash

cd ../gz_ws

colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17

source install/setup.bash

source gazebo_exports.sh
 
```

## Run Gazebo BlueRov2 simulator

Note:
- IMPORTANT: Run Gazebo first as it contains the ArduPilot plugin. Later, start the ArduPilot SITL (Rover) simulator. Lastly, you can run QGroundControl.

```bash
sudo docker exec -it bluerov2_sitl /bin/bash

ros2 launch move_blueboat launch_robot_simulation.launch.py
```

## Run SITL

Notes:

- The flag ```-l``` is the localization (lat,lon,alt,heading). Check your favorite location with Google Maps.
- ```sim_vehicle.py --help ``` -prints all available commands and flags.
- in ```run.sh``` adjust these two lines for your host specific:


```bash

sudo docker exec -it bluerov2_sitl /bin/bash

cd ../ardupilot

sim_vehicle.py -v ArduSub -f vectored_6dof --model JSON --map --console -l 55.99541530863445,-3.3010225004910683,0,0

# if you need to recompile 
Tools/environment_install/install-prereqs-ubuntu.sh -y

# after recompiling 
. ~/.profile

```
## Start QGC (outside Docker)

```bash
./QGroundControl.AppImage
```

## Move BlueRov2

Move the vehicle using Ardusub to navigate the designated waypoints, utilizing either ROS 2 or ArduPilot.

```bash
# It is recommended to Arm the vehicle first

arm throttle
```

```bash
ros2 launch move_blueboat wp_rov
```

```bash
cd  /gz_ws/src/extras_rov

python3 wp_pos_req.py
```

## Change the Motion Velocity

The following program, before motion, changes the ArduSub parameters influencing the global velocity motion for the vehicle.
- ```WPNAV_SPEED``` - for horizontal speed
- ```WPNAV_SPEED_DN``` - for descent speed
- ```WPNAV_SPEED_UP``` - for climb speed

```bash
ros2 launch move_blueboat wp_velo_rov
```

## PlotJuggler

```bash
ros2 run plotjuggler plotjuggler
```

## Run Motor Test (PWM control)

Setting the ArduSub [RCPassThru](https://ardupilot.org/copter/docs/parameters.html#servo1-function-servo-output-function) parameter allows setting PWM directly for individual motors with values ranging from 1100 to 1900.

```bash
ros2 run plotjuggler plotjuggler

cd extras

python3 test_motor_thrust.py
```
![image](https://github.com/oceansystemslab/bluerov2_ardupilot_SITL/assets/30973337/79f27219-d755-482e-8fcf-7b5d4d39c53d)

### Performance Charts 
[Details](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/)

![image](https://github.com/oceansystemslab/bluerov2_ardupilot_SITL/assets/30973337/a8924194-1262-4255-b896-6182d27357b7)


## Acknowledgement

- [blue](https://github.com/Robotic-Decision-Making-Lab/blue)
- [orca4](https://github.com/clydemcqueen/orca4)
