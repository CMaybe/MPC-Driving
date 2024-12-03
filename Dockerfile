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
		swig \
		vim \
		wget \
    && apt-get clean

# Update and install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
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
	libssl-dev \
	libtool \
    && apt-get clean

	
# Install CMake 3.24.3
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.3/cmake-3.24.3.tar.gz \
	&& tar -zxvf cmake-3.24.3.tar.gz && rm cmake-3.24.3.tar.gz \
	&& mv cmake-3.24.3 /opt/cmake \
	&& cd /opt/cmake \
	&& ./bootstrap \
	&& make install \
    && make -j$(USE_PROC) \
    && cd /opt && rm -r /opt/cmake

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


# Install altro
RUN git clone https://github.com/zixinz990/altro.git /opt/altro \
&& mkdir -p /opt/altro/build && cd /opt/altro/build \
&& cmake \
	   -DCMAKE_BUILD_TYPE=Release \
	   -DBUILD_SHARED_LIBS=ON \
	   -DALTRO_BUILD_TESTS=OFF \
	   -DCMAKE_INSTALL_PREFIX=/usr/local \
   .. \
&& make install \
&& cd /opt && rm -r /opt/altro

# Install CasADi
RUN git clone https://github.com/casadi/casadi.git --branch main /opt/casadi \
	&& mkdir -p /opt/casadi/build && cd /opt/casadi/build \
	&& cmake \
		-DCMAKE_BUILD_TYPE=Release \
		-DBUILD_TESTS=OFF \
		-DWITH_PYTHON=ON \
		-DWITH_PYTHON3=ON \
		.. \
	&& make install \
    && cd /opt && rm -r /opt/casadi

# Install gram_savitzky_golay
RUN git clone --recursive https://github.com/arntanguy/gram_savitzky_golay.git /opt/gram_savitzky_golay \
	&& cd /opt/gram_savitzky_golay && git submodule init && git submodule update --recursive \
	&& mkdir -p /opt/gram_savitzky_golay/build && cd /opt/gram_savitzky_golay/build \
	&& cmake \
		-DCMAKE_BUILD_TYPE=Release \
		-DBUILD_TESTS=OFF \
		.. \
	&& make install \
    && cd /opt && rm -r /opt/gram_savitzky_golay


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