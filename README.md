# Simulated Virtual GPS

## CS1980 Capstone Project 

### Sponsor 

Luis Oliveria 

### Authors

Jonathan Miller 

Olivia Neights 

Zixin Ye

## Frameworks and Tools 

### Ubuntu 24.04

    
Ubuntu is one of the most popular Linux Operating Systems. 24.04 was the latest stable long term release version at the time of this project. Many of the following tools work well and are easy to isntall on an Ubuntu system, compared with Mac or Windows.    

### ROS 2

ROS and ROS 2 are popular robotics frameworks that can be used with C++ and Python. They help facilitate the communication between distributed components common to robotics and autonomous driving systems.  

We are using the "Jazzy" version of ROS 2, which matches to the 24.04 version of Ubuntu. This link will lead the installation instructions. 

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Remember to follow the step to change your .bashrc file and the line `source /opt/ros/jazzy/setup.bash`, so that every time you open a new terminal it automatically sources your ROS 2 installation, and you will be able to run ROS commands easily. 

You will steed need to source your custom packages in your workspaces with `source install/setup.bash`

### MAVROS

MAVROS is an additional ROS 2 library, which is designed to help a ROS node communicate with a flying drone that is using the Mavlink API. Mavlink is a common interface that drone flight controllers (such as PX4) use to send commands to the platform.  

To install the MAVROS libraries simply run these commands:

`sudo apt install ros-jazzy-mavros`    (jazzy could be replaced by other ROS 2 versions)

`ros2 run mavros install_geographiclib_datasets.sh`

If the second command does not work, I have found that the shell script lives at `/opt/ros/jazzy/lib/mavros`. 

You can `cd` there and just run the script with `sudo ./install_geographiclib_datasets.sh`.

### TF2

TF2 is a popular library that is used for complex mathematical calculations and is compatible with ROS and ROS 2. It is chosen due to its reputation. 

**add install instructions here**

### PX4-Autopilot

PX4 is the flight control system used by the Holybro x500 quadcopter drone. We have selected this model because the sponsor has several physical models, and some of the team members have experience working with it. 
PX4-Autopilot is a set of libraries for the flight control system. It includes premade models that can be used with the Gazebo simulator.

Install instructions:

Clone the PX4-Autopilot repository from Github. This project assumes that you have just cloned it to your home directory. 

`git clone https://github.com/PX4/PX4-Autopilot.git --recursive`

After that you should run this setup script to install all other necessary dependencies on Ubuntu.

`bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`

### QGroundControl

This is an application which is used as a Ground Control Station (GCS) for flying drones. A GCS is often necessary when programmatically controlling a flying drone, becuase if it loses connection or control, it needs a fallback control system so the operator can land the drone safely. In some cases the GCS could be a remote controller, but this is an application that allows monitoring of the drone on a map and provides or mission control for end users. This application was chosen due to team member's familiarity. 

Installing on Ubuntu:

Follow the steps for Ubuntu Linux from this link: 

`https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html`

This project and start.sh script assume the AppImage file lives in your ~/Downloads. It can be moved, just remember to alter the script to point to the new path.

### Gazebo 

Gazebo is a popular simulation software for robotics applications. This was chosen due to its popularity in the field, and was one technology the project sponsor was insistent on using. 

Installation steps:

`sudo apt-get install ros-jazzy-ros-gz`

This should install the gazebo version that matches your ROS distribution. In our case since we have ROS Jazzy, it should install GZ Harmonic.


## Hello World 

If all of those packages and libraries are successfully installed, then you should be able to launch gazebo with 1 x500 model by running a simple make command. 
Go to the PX4-Autopilot folder and run this make command.

`make px4_sitl gz_x500`

It should launch a gazebo window automatically and have a quadcopter drone sitting at the origin point. 
Sometimes on the first try it may fail. Just end the process and run again. 





