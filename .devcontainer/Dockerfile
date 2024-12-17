FROM ghcr.io/cmaybe/dev-mpc-driving:latest

ARG USER_NAME=cmaybe
ARG GROUP_NAME=CAU
ARG WORKSPACE_NAME=ros2_ws

ARG PROJECT_NAME=mpc-driving

# Add user info
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${GROUP_NAME} 

RUN useradd --create-home --shell /bin/bash \
	--uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} \
	# Possible security risk
	&& echo "${USER_NAME}:${GROUP_NAME}" | sudo chpasswd \
	&& echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}"

# Make workspace 
RUN mkdir -p /home/${USER_NAME}/${WORKSPACE_NAME}/src/${PROJECT_NAME} \
	&& chown -R ${USER_NAME}:${GROUP_NAME} /home/${USER_NAME}/${WORKSPACE_NAME}
ENV HOME /home/${USER_NAME}
ENV WORKSPACE ${HOME}/${WORKSPACE_NAME}

# Shell
USER ${USER_NAME}
WORKDIR ${WORKSPACE}
ENV SHELL "/bin/bash"

RUN echo "export USER=${USER_NAME}" >> ${HOME}/.bashrc \
	&& echo "export GROUP=${GROUP_NAME}" >> ${HOME}/.bashrc \
	&& echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc

# Shell
USER ${USER_NAME}
WORKDIR ${WORKSPACE}
ENV SHELL "/bin/bash"
