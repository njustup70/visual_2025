FROM elainasuki/ros:ros2-humble-full-v3

# 定义用户和用户组
ARG USERNAME=Elaina
WORKDIR /home/$USERNAME

RUN apt-get update && apt-get install -y ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan
USER ${USERNAME}
# COPY --chown=${USERNAME} packages/linefit_ground_segementation /home/$USERNAME/packages/ros_ws/src/linefit_ground_segementation
# RUN cd /home/$USERNAME/packages/ros_ws \
#     && . /opt/ros/humble/setup.sh \
#     && colcon build --symlink-install
# && bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# RUN  sudo add-apt-repository ppa:borglab/gtsam-release-4.1 && sudo apt install -y libgtsam-dev libgtsam-unstable-dev\ 
#     && cd /home/$USERNAME/packages/ros_ws/src && git clone https://github.com/YJZLuckyBoy/liorf_localization.git \
#     && cd liorf_localization && git checkout liorf_localization-ros2 \
#     && . /opt/ros/humble/setup.sh \
#     && colcon build --symlink-install
USER root
#安装导航依赖
RUN apt-get update && apt-get install -y ros-humble-nav2-bringup
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_LOCALHOST_ONLY=1" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_LOCALHOST_ONLY=1" >> /home/$USERNAME/.config/fish/config.fish \
    && echo "source ~/visual/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source ~/packages/ros_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "bass source ~/packages/ros_ws/install/setup.bash" >> /home/$USERNAME/.config/fish/config.fish \
    && echo "bass source ~/visual/install/setup.bash" >> /home/$USERNAME/.config/fish/config.fish 

CMD ["/bin/bash"]

