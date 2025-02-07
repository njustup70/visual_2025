FROM elainasuki/ros:ros2-humble-full-v2

# 定义用户和用户组
ARG USERNAME=Elaina
WORKDIR /home/$USERNAME

RUN apt-get update && apt-get install -y ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan ros-humble-foxglove-bridge iputils-ping

COPY packages/linefit_ground_segementation /home/$USERNAME/packages/ros_ws/src/linefit_ground_segementation
RUN cd /home/$USERNAME/packages/ros_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source ~/location/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source ~/packages/ros_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "bass source ~/packages/ros_ws/install/setup.bash" >> /home/$USERNAME/.config/fish/config.fish \
    && echo "bass source ~/location/install/setup.bash" >> /home/$USERNAME/.config/fish/config.fish 

CMD ["/bin/bash"]

