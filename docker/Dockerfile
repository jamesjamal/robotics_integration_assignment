# This is an example Docker File
#  Command to build it
# docker built -t <image name > .
FROM osrf/ros:noetic-desktop-full

SHELL [ "/bin/bash" , "-c" ]
RUN apt-get update

RUN apt-get install -y git && apt-get install -y python3-pip


RUN source /opt/ros/noetic/setup.bash 

RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src/
    



RUN git clone https://github.com/jamesjamal/robotics_integration_assignment.git && \
    cd ~/catkin_ws
    
RUN source /opt/ros/noetic/setup.bash 




RUN apt-get install -y ros-noetic-ackermann-msgs ros-noetic-geometry2 

RUN apt-get install -y ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins 

RUN apt-get install -y  ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator

RUN pip  install numpy==1.20

RUN pip install cvxpy

RUN cd / && \
    mv robotics_integration_assignment ~/catkin_ws/src    

RUN echo "ALL Done"
