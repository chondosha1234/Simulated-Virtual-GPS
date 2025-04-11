# Simulated Virtual GPS

## CS1980 Capstone Project 

### Sponsor 

Luis Oliveira 

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

Install instructions:

`sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-tools`

`sudo apt install ros-jazzy-tf-transformations`

### Raspimouse

raspimouse is a small ground robot. It was chosen for its simplicity and gz compatible models. 

Install instructions:

`sudo apt install ros-jazzy-raspimouse-*`

There is a launch file in our code which is borrowed and adapted from the raspimouse provided launch files. It works from our project because it automatically locates the necessary files that are installed through this apt command.

For our project we need to add a PosePublisher for the raspimouse so we can get its Pose from the gazebo simulation.
This can be done by altering the file `raspimouse.urdf.xacro` at the path `/opt/ros/jazzy/share/raspimouse_description/urdf`.
Scroll down to where it says `<!-- =============== Gazebo =============== -->` and add the following lines.

      <gazebo>
        <plugin name="gz::sim::systems::PosePublisher" filename="gz-sim-pose-publisher-system">
          <publish_model_pose>true</publish_model_pose>
          <publish_link_pose>false</publish_link_pose>
          <publish_collision_pose>false</publish_collision_pose>
          <publish_visual_pose>false</publish_visual_pose>
          <publish_nested_model_pose>true</publish_nested_model_pose>
        </plugin>
      </gazebo>


### PX4-Autopilot

PX4 is the flight control system used by the Holybro x500 quadcopter drone. We have selected this model because the sponsor has several physical models, and some of the team members have experience working with it. 
PX4-Autopilot is a set of libraries for the flight control system. It includes premade models that can be used with the Gazebo simulator.

Install instructions:

Clone the PX4-Autopilot repository from Github. This project assumes that you have just cloned it to your home directory. 

`git clone https://github.com/PX4/PX4-Autopilot.git --recursive`

After that you should run this setup script to install all other necessary dependencies on Ubuntu.

`bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`

In order for to get appropriate gazebo simulation model positions, we need to add a plugin to the model.sdf file for the x500 drone. 
Add the following xml code to the file `model.sdf` at the path `~/PX4-Autopilot/Tools/simulation/gz/models/x500`.
Near the top of the file is a list of plugins and can be added in that area.

    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
            <publish_model_pose>true</publish_model_pose>  
            <publish_link_pose>false</publish_link_pose>   
            <publish_collision_pose>false</publish_collision_pose>      
            <publish_visual_pose>false</publish_visual_pose>     
            <publish_nested_model_pose>true</publish_nested_model_pose>     
    </plugin>


### QGroundControl

This is an application which is used as a Ground Control Station (GCS) for flying drones. A GCS is often necessary when programmatically controlling a flying drone, becuase if it loses connection or control, it needs a fallback control system so the operator can land the drone safely. In some cases the GCS could be a remote controller, but this is an application that allows monitoring of the drone on a map and provides or mission control for end users. This application was chosen due to team member's familiarity. 

Installing on Ubuntu:

Follow the steps for Ubuntu Linux from this link: 

https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

This project and start.sh script assume the AppImage file lives in your ~/Downloads. It can be moved, just remember to alter the script to point to the new path.

### Gazebo 

Gazebo is a popular simulation software for robotics applications. This was chosen due to its popularity in the field, and was one technology the project sponsor was insistent on using. 

Installation steps:

`sudo apt-get install ros-jazzy-ros-gz`

This should install the gazebo version that matches your ROS distribution. In our case since we have ROS Jazzy, it should install GZ Harmonic.

Running inside a virtual machine may cause a "Unable to create the rendering window" error. In this case, the user can try disabling DRI by:

`export LIBGL_DRI3_DISABLE=1`

or force software rendering

`export MESA_GL_VERSION_OVERRIDE=3.3`

## Hello World 

If all of those packages and libraries are successfully installed, then you should be able to launch gazebo with 1 x500 model by running a simple make command. 
Go to the PX4-Autopilot folder and run this make command.

`make px4_sitl gz_x500`

It should launch a gazebo window automatically and have a quadcopter drone sitting at the origin point. 
Sometimes on the first try it may fail. Just end the process and run again. 

## Further Tests and Experiments 

There is a scripts folder which has shell scripts. 
These build and launch all the necessary tools and frameworks for our simulations. 
These were made to avoid having to open dozens of terminal windows. 
The output from these processes is sent to log files in the top level directory.
There are also several ROS2 launch files in our project which start up multiple ROS2 nodes for our experiments. 
The launch files are also called in the scripts at appropriate places. 

These scripts are found in the `scripts/` folder and can be easily run with `./start.sh` for example.

### Stage 1

Version 1 of the project was to set up the infrastructure and architecture of the simulation and get a basic virtual GPS to work.
This setup has 4 flying x500 drones positioned on the ground in the 4 cardinal directions around the origin.
One raspimouse is placed at the origin point. 
Using our virtual GPS for the raspimouse, we are able to calculate its position using 4 positions and distances of the 4 drones. 

### Stage 2

Now we want to develop a system where there are less than 4 drones. 
The GPS trilateration equations require at least 4 points to get the location, so if we are limited to less drones, we need a way to buffer measurements.
We will have 2 flying drones and they will takeoff and move in some defined pattern (for example a square flight path, or just back and forth).
As they move they take consistent distance measurements and know their own positions.
From this we can get the necessary 4 points for the GPS calculation. 
As more measurements arrive, the oldest in the buffer can be replaced. 
The goal is to replicate the same GPS measurements and raspimouse movement as in version 1, but with 2 drones. 
This can also be extedned to using only 1 drone.

### Stage 3 

At this point we have a working GPS implementation which accurately can pinpoint the raspimouse.
It works with 4 drones, and the buffered versions with 1 and 2 drones also.
Now we will move the raspimouse and see if this affects the accuracy of the measurements. 
We can use some basic patterns like just moving in a straight line or moving in a square.
We can use the gazebo simulation data to get the actual robot position and compare it to our GPS position to obtain an error. 
After testing this we noticed that the 4 drone setup works well (albeit with some amount of error that is way off and can be filtered out).
The 1 and 2 drone setups have noticeably higher error as the raspimouse moves. 

### Stage 4 

After accomplishing the goals of Stage 3, we decided to apply a kalman filter node to improve upon the error of the 1 and 2 drone setups. 
The kalman filter node has a predict and an update step. 
The update step is triggered by obtaining a new measurement from our GPS topic.
The prediction step can run multiple times between new measurements.
We also can include the ability to track the velocity by listening to the raspimouse's basic odometry.
This odometry just has forward/backward velocity and angular velocity, but it can be converted to the world's xy plane velocity. 
This filter shows a noticeable improvement for the 1 and 2 drone setups. 




