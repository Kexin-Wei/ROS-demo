FROM ros:jazzy
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_VERSION=jazzy
ARG BASH_FILE=/home/${USERNAME}/.bashrc

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install prerequests
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip neovim openssh-client terminator 

# Install gazebo
# RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y curl lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y gz-harmonic

# install ros
# RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y ros-${ROS_VERSION}-turtlesim ros-${ROS_VERSION}-urdf-tutorial ros-${ROS_VERSION}-ros-gz '~nros-${ROS_VERSION}-rqt*'

# add ros path to .bashrc
RUN touch ${BASH_FILE} && chown ${USERNAME}:${USERNAME} ${BASH_FILE}
RUN echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ${BASH_FILE} \
    && echo "source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" >> ${BASH_FILE} \
    && echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> ${BASH_FILE} 

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
