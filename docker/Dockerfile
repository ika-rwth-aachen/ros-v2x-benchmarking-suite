ARG DEBIAN_FRONTEND=noninteractive
ARG WORKSPACE=/root/ws
ARG ROS_DEPENDENCIES_INSTALL_SCRIPT=$WORKSPACE/.install-dependencies.sh

# === TEMPORARY STAGE FOR DETERMINING DEPENDENCIES =============================
FROM osrf/ros:noetic-desktop-full AS dependencies
ARG DEBIAN_FRONTEND
ARG WORKSPACE
ARG ROS_DEPENDENCIES_INSTALL_SCRIPT

# create ROS workspace
ENV WORKSPACE=$WORKSPACE
RUN mkdir --parents $WORKSPACE
WORKDIR $WORKSPACE

# copy repository ROS packages
COPY src src

# determine dependencies with rosdep
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && \
    apt-get install --yes python3-pip && \
    rosdep update && \
    rosdep install --simulate --default-yes --ignore-src --from-paths src | tee $ROS_DEPENDENCIES_INSTALL_SCRIPT && \
    chmod +x $ROS_DEPENDENCIES_INSTALL_SCRIPT

# === STAGE FOR DEVELOPMENT IMAGE ==============================================
FROM osrf/ros:noetic-desktop-full AS development
ARG DEBIAN_FRONTEND
ARG ROS_DEPENDENCIES_INSTALL_SCRIPT

# create ROS workspace
ENV WORKSPACE=/root/ws
RUN mkdir --parents $WORKSPACE
WORKDIR $WORKSPACE

# install dependencies
COPY --from=dependencies $ROS_DEPENDENCIES_INSTALL_SCRIPT $ROS_DEPENDENCIES_INSTALL_SCRIPT
RUN apt-get update && \
    apt-get install --yes python3-pip python3-catkin-tools && \
    rosdep update && \
    $ROS_DEPENDENCIES_INSTALL_SCRIPT && \
    rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# === STAGE FOR EXECUTION IMAGE ================================================
FROM development AS execution
ARG DEBIAN_FRONTEND

# copy repository ROS packages
COPY src src

# build ROS workspace
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build -DCMAKE_BUILD_TYPE=Release --force-color --no-status --summarize
