# This is a merged version Dockerfile of official ros-base and ros-core image
# Retrieved from https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy
FROM ubuntu:jammy
ARG USER_NAME=cmaybe
ARG GROUP_NAME=CAU
ARG PROJECT_NAME=mpc-driving


# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get -y -q --no-install-recommends install \
        tzdata \
    && apt-get clean

# Install packages
RUN apt-get update \
    && apt-get install -y -q --no-install-recommends \
		apt-utils \
    	bash-completion \
		clang-format \
        dirmngr \
        gnupg2 \
		sshpass \
        sudo \
		vim \
    && apt-get clean

# Update and install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    dirmngr \
    git \
    gnupg2 \
    python3-numpy \
    python3-matplotlib \
    python2.7-dev \
    python3 \
    python3-dev \
    python3-pip \
    && apt-get clean

# Dependency packages
RUN apt-get -q update \
	&& apt-get -y -q --no-install-recommends install \
    libboost-all-dev \
    libpython3-dev \
    && apt-get clean

# Install matplotlib-cpp
RUN git clone https://github.com/lava/matplotlib-cpp.git  --recursive /opt/matplotlib-cpp \
	&& mkdir -p /opt/matplotlib-cpp/build && cd /opt/matplotlib-cpp/build \
	&& cmake \
	-DCMAKE_BUILD_TYPE=Release \
	.. \
	&& make install \
    && cd /opt && rm -r /opt/matplotlib-cpp

# Install Eigen 3.4.0	
RUN git clone --branch 3.4.0 https://gitlab.com/libeigen/eigen.git /opt/eigen \
	&& mkdir -p /opt/eigen/build && cd /opt/eigen/build \
	&& cmake \
	       -DCMAKE_BUILD_TYPE=Release \
		   -DEIGEN_BUILD_DOC=OFF \
		   -DBUILD_TESTING=OFF \
	   .. \
	&& make install \
    && cd /opt && rm -r /opt/eigen

# Install OSQP
RUN git clone --recursive https://github.com/oxfordcontrol/osqp --branch v0.6.3 /opt/osqp \
	&& cd /opt/osqp && git submodule update --recursive \
	&& mkdir -p /opt/osqp/build && cd /opt/osqp/build \
	&& cmake -G "Unix Makefiles" .. \
	&& cmake --build . \
	&& cmake --build . --target install 

# Install OSQP-Eigen
RUN git clone --recursive https://github.com/robotology/osqp-eigen.git --branch v0.8.0 /opt/osqp-eigen \
	&& mkdir -p /opt/osqp-eigen/build && cd /opt/osqp-eigen/build \
	&& cmake \
		-DCMAKE_BUILD_TYPE=Release \
		-DBUILD_TESTS=OFF \
		.. \
	&& make install \
    && cd /opt && rm -r /opt/osqp-eigen


# Setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Add user info
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${GROUP_NAME} 

RUN useradd --create-home --shell /bin/bash \
               --uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} \
	# Possible security risk
	&& echo "${USER_NAME}:${GROUP_NAME}" | sudo chpasswd \
	&& echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}"

RUN usermod -aG video ${USER_NAME}

# Make workspace 
RUN mkdir -p /home/${USER_NAME}/${PROJECT_NAME}
ENV HOME /home/${USER_NAME}
ENV WORKSPACE ${HOME}/${PROJECT_NAME}
RUN chown -R ${USER_NAME}:${GROUP_NAME} ${WORKSPACE}

# Shell
USER ${USER_NAME}
WORKDIR ${WORKSPACE}
ENV SHELL "/bin/bash"

CMD ["bash"]