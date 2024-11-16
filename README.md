# Polaris Gem e2 Manager

This is a high level manager of the [Polaris Gem e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git). The system behind it has been built to receive mocked sensor data and control the vehicle's course based on them.

## Environment

As it is required by the Polaris project, I set up an Ubuntu 20.04 machine with ROS Noetic, following at the same time the GNU and cmake versions that were already in place.

So,

- C++ standard: C++17
- Compiler: GNU gcc/g++ packages both in 9.4.0 version
- Debugger: GNU gdb (GDB) package in 10.2 version
- cmake: version 3.16.3

Not to lie, I missed some recent C++ features, but it was fine (some of them are commented on the code).

## How to run

All you have to do is to build the docker image by executing the following line and being at the same level with the `Dockerfile`:

> docker build -t polaris-gem-manager .

Then to run it, make the `run_container.sh` executable,

> chmod +x run_container.sh

and execute it,

> ./run_container.sh

This way you will land into the workspace having everything built! The only thing you have to do, is to run the nodes.

You will have the `terminator` terminal installed, which you can launch by executing `terminator -u`.

In different terminals/windows run,

1. `roscore`
2. `roslaunch gem_gazebo gem_gazebo_rviz.launch`
3. `rosrun gem_pure_pursuit_sim_new_interface pure_pursuit_sim.py` or `rosrun gem_stanley_sim_new_interface stanley_sim.py`
4. `roslaunch gem_manager polaris_manager_simulation.launch` (don't forget to modify the node parameter according to the controller)

From this point, you have the following options to test with,

- `rosrun gem_mock_sensors battery_node`
- `rosrun gem_mock_sensors temperature_node`
- `rosrun gem_mock_sensors gps_node`
- `rosrun gem_mock_sensors signal_node`
- `rosrun gem_mock_sensors emergency_stop_node`

And to inspect the underneath message exchange, `rostopic echo` followed by,

- `/gem_manager/battery_level`
- `/gem_manager/temperature`
- `/gem_manager/gps_accuracy`
- `/gem_manager/signal_strength`
- `/gem_manager/emergency_button`
- `/gem_manager/robot_state`
- `/gem_manager/waypoints`

---

## Author

@[georgiosmatzarapis](https://georgiosmatzarapis.com)
