FROM althack/ros2:humble-full
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN sudo apt update
ARG WORKSPACE
WORKDIR ${WORKSPACE}
USER ros
RUN python3 -m pip install pre-commit

# Make sure bash (for the root and ros users) is ready to ros.
RUN for user in /root /home/ros; do \
# Source ros and the workspace.
echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then . /opt/ros/${ROS_DISTRO}/setup.bash; fi" \
>> ${user}/.bashrc; \
echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then . ${WORKSPACE}/install/setup.bash; fi" \
>> ${user}/.bashrc; \
echo "alias sc='. ${WORKSPACE}/install/setup.bash'" >> ${user}/.bashrc; \
echo ". /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ${user}/.bashrc; \
done

ENV PYTHONPATH "/workspace"