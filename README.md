## Flight Mission 

In its current implemenation, the UAS will being with a straight line flight for ~$30$ seconds, before making a ~$270^\circ$ turn, then continuing on with straight line flight until the program is terminated.

### Future work includes:
- Using Gazebo to simulate sensor measurments
- Implementing EKF and VINS Fusion, adding state uncertainty
- Ping QGroundControl when the UAS believes it has flown over the target
- Increasing duration of turning phase to mimic sensor calibration

## REQUIREMENTS
- Ubuntu (22.04)
- USE PX4 1.17, px4_msgs and px4_ros2_cpp must be on the main branch as well.
- PX4 and ROS 2 installations as detailed by https://docs.px4.io/main/en/ros2/user_guide.html 
    - This involves setting up the PX4 toolchain, ROS 2, and the Micro XRCE-DDS Agent 
    - Tested on PX4 v1.17.0 ROS 2 Humble
- QGroundControl
    - Works with the latest QGC daily 

## Setup
1. Setup and build a new ROS workspace
    ```bash
    # Make a new folder for the workspace
    mkdir -p custom_flightmode/src
    cd custom_flightmode/src

    # Clone components
    git clone https://github.com/PX4/px4_msgs.git
    git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib.git
    git clone https://github.com/Undead-Reckoning/px4_ros2_fixed_wing_flight.git

    # Build the workspace and setup the shell to run ros commands later
    cd ..
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ```
2. **In a new terminal**, start MicroXRCEAgent
    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```
3. **In another new terminal**, start a Gazebo PX4 simulation
    ```bash
    cd ~/PX4-Autopilot # Change this depending on where you installed PX4
    
    # Ignore these 2 lines if you are already on the correct version of PX4
    git checkout release/1.17
    git submodule update --init --recursive

    # Note that at some point we would like to replace rc_cessna
    make px4_sitl make px4_sitl gz_rc_cessna
    ```
4. Start QGroundControl. It should connect to the drone in the simulator.
5. **Return to the first terminal** and run the following. If you closed the terminal, you'll need to run the 2 `source` commands from the first step again beforehand.
    ```bash
    ros2 run px4_ros2_fixed_wing_flight fly_uas
    ```
6. The drone should automatically arm, takeoff, and start flying.

