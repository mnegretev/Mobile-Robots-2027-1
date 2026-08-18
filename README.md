# Mobile Robots, FI-UNAM, 2027 1
Software for the Mobile Robots course at FI-UNAM, 2027-1

## Requirements

* Ubuntu 24.04: https://ubuntu.com/download/desktop/thank-you?version=24.04.3&architecture=amd64&lts=true
* ROS Jazzy Jalisco: https://docs.ros.org/en/jazzy/Installation.html
* Google DeepMind MuJoCo (use version 3.8): https://mujoco.readthedocs.io/en/latest/programming/#building-from-source
* Webots 2025a: https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb

## Installation

Note: This instructions assume you already installed Ubuntu, MuJoCo and ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2027-1/
* $ cd Mobile-Robots-2027-1
* $ sudo apt update
* $ sudo apt upgrade
* $ ./Setup.sh
* $ cd ros2_ws
* $ echo "alias cb='colcon build && source install/local_setup.bash'" >> ~/.bashrc
* $ source ~/.bashrc
* $ cb

## Testing

If everything was correctly installed and built, run:

* $ cd ~/Mobile-Robots-2027-1/ros2_ws
* $ cb
* $ ros2 launch house_simul house_simul.launch.py
  
And, in another terminal:

* $ cd ~/Mobile-Robots-2027-1/ros2_ws
* $ cb
* $ ros2 launch  navig_utils navig_utils.launch.py

If everything was correctly installed and built, you should see the Ros2 visualizer (RViz2):
![rviz](https://github.com/mnegretev/Mobile-Robots-2027-1/blob/main/Media/rviz2.png)

A simulated house environment:
![gazebo](https://github.com/mnegretev/Mobile-Robots-2027-1/blob/main/Media/gz.png)

And a Graphic User Interface (GUI):
![GUIExample](https://github.com/mnegretev/Mobile-Robots-2027-1/blob/main/Media/gui.png)

## Contact
Dr. Marco Negrete<br>
Full Time Professor A<br>
Head of the Signal Processing Department<br>
School of Engineering, UNAM <br>
marco.negrete@ingenieria.unam.edu<br>
https://mnegretev.info<br>
https://lira.unam.mx<br>

