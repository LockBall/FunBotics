# HW2 - three ROS nodes:
# Node 1: publishes points (as geometry_msg/Point type) with random x, y, z values on topic /obstacles_detected

# Node 2: subscribes to /obstacles_detected, ingests those points, adds a unique identifier to them (your choice of format) and #publishes this new data structure as, YOUR MESSAGE TYPE from question 4, on topic /registered_obstacles

#Node 3: Subscribes to /registered_obstacles and logs an info-level message to rosout [output to terminal / screen / console] for each #one in a human readable format such as: “Detected obstacle <id> at position <x>,<y>,<z>”

# I attempted to scriptify this process but ran into issues enar the end.
# uncommented commands work, commands for testing are optional

# Assuming docker is installed and the user setup is complete.
# funbotics is the <container name> , you can choose any container name but wold have to change it for everything below.

# cleanup possible pre-existing container
docker stop funbotics
docker rm funbotics

# create container, install kinetic if missing
docker run -di --rm --name funbotics ros:kinetic-perception
# -d detach, in background;   -i interactive even in background;   --rm removes container when done;   -t terminal

# verify container creation for testing
docker container ps -a

# create catkin workspace & -p parent directories
docker exec funbotics mkdir -p /root/catkin_ws/src

# Go to the catkin workspace src directory and clone the git repository
docker exec -i funbotics bash -c "cd ~/catkin_ws/src && git clone https://github.com/LockBall/FunBotics.git"

# Not Working
# docker exec -id funbotics -c /bin/bash “source /opt/ros/kinetic/setup.bash”
# docker exec -id funbotics bash -c /bin/bash “source /opt/ros/kinetic/setup.bash”
# source is not a command. there is no straightforward way to pass the source statement to the docker container

# terminal in existing container
docker exec -it funbotics bash

source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws

#      requires ↑↑
catkin_make

source /root/catkin_ws/devel/setup.bash

# requires ↑
roslaunch funbotics_hw2 3nodes.launch

# ctrl + shift + c   to exit
