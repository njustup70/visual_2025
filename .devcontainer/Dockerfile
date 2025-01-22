FROM elainasuki/ros:ros2-humble-full-v2

# 定义用户和用户组
ARG USERNAME=Elaina
WORKDIR /home/$USERNAME
RUN apt-get install -y ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan 
RUN  echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc
CMD ["/bin/bash"]

