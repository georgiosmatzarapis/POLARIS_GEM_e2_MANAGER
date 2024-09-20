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

---

## Author

@[georgiosmatzarapis](https://georgiosmatzarapis.com)
