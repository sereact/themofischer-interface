
FROM osrf/ros:humble-desktop-full

# Install necessary dependencies for Python and ROS2
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-dev-tools \
    sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install node
ENV NODE_VERSION=18.20.3
RUN apt update && apt install -y curl
RUN curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.34.0/install.sh | bash
ENV NVM_DIR="/root/.nvm/"
# RUN ls -l ${NVM_DIR}
RUN . "$NVM_DIR/nvm.sh" && nvm install ${NODE_VERSION} && . "$NVM_DIR/nvm.sh" && nvm use v${NODE_VERSION} && . "$NVM_DIR/nvm.sh" && nvm alias default v${NODE_VERSION}
ENV PATH="/root/.nvm/versions/node/v${NODE_VERSION}/bin/:${PATH}"
RUN node --version
RUN npm --version

RUN echo "Creating user sereact"
WORKDIR /home/sereact

ARG USERNAME=sereact
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  #
  # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && cp /root/.bashrc /home/sereact/.bashrc && chown -R sereact:sereact /home/sereact/

USER sereact
USER root

WORKDIR /thermofischer_interface/colcon_ws
COPY ./colcon_ws/src /thermofischer_interface/colcon_ws/src
# Install ROS2 dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 package in colcon_ws
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && colcon build"]

WORKDIR /
COPY ./run_services /run_services
COPY ./wheelhouse/*.whl /thermofischer_interface/
RUN pip3 install /thermofischer_interface/*.whl
WORKDIR /