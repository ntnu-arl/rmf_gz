# minimal RMF simulation

this repo contains minimal ROS2 launchfiles and resources for RMF simulation in Gazebo

## usage

### requirements

* ROS2 humble
* [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/) with [forked gz-sim](https://github.com/ntnu-arl/gz-sim)
    * install via apt:
    ```bash
    sudo apt-get install gz-harmonic ros-humble-ros-gzharmonic
    ```
    * clone [ntnu-arl's fork](https://github.com/ntnu-arl/gz-sim/tree/sim8/dev/multicopter_control)
    * checkout branch `sim8/dev/multicopter_control`
    * install as
    ```bash
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=/usr ..
    make -j12
    sudo make install
    ```
    > Doxygen building may fail on make with gz-sim 8.9 and Ubuntu 22.04, it appears that it was fixed on 8.10.
    In this case, add `-DCMAKE_BUILD_DOCS=false` to the configure options.

### install

clone and install normally via colcon

## content

### models

* RMF (with front-facing depth camera and lidar)

### worlds

* ...

### scripts

* random, Poisson Discs-based world generations (pillars or 3D)
