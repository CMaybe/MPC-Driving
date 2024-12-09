# ROS_DISTRO is humble
FROM ros:humble-ros-base 
ARG USER_NAME=cmaybe
ARG GROUP_NAME=CAU
ARG PROJECT_NAME=mpc-driving


# Install packages
RUN apt-get update \
    && apt-get install -y -q --no-install-recommends \
    apt-utils \
    bash-completion \
    clang-format \
    gnupg2 \
    sshpass \
    sudo \
    vim \
    wget \
    && apt-get clean

# Update and install necessary dependencies
RUN apt-get update \
    && apt-get install -y -q --no-install-recommends \
    build-essential \
    dirmngr \
    cmake \
    git \
    gnupg2 \
    python3-numpy \
    python3-matplotlib \
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


# Install Ipopt
# Update and install necessary dependencies
RUN apt-get update \
    && apt-get install -y -q --no-install-recommends \
    cppad \
    gfortran\
    patch \
    pkg-config \
    liblapack-dev \
    libmetis-dev\
    && apt-get clean

RUN	git clone --branch stable/2.0 https://github.com/coin-or-tools/ThirdParty-ASL.git /opt/ThirdParty-ASL \
    && mkdir -p /opt/ThirdParty-ASL  && cd /opt/ThirdParty-ASL \
    && ./get.ASL && ./configure \
    && make install \
    && cd /opt && rm -r /opt/ThirdParty-ASL


RUN	git clone --branch stable/3.0 https://github.com/coin-or-tools/ThirdParty-Mumps.git /opt/ThirdParty-Mumps \
    && mkdir -p /opt/ThirdParty-Mumps && cd /opt/ThirdParty-Mumps \
    && ./get.Mumps && ./configure \
    && make install \
    && cd /opt && rm -r /opt/ThirdParty-Mumps


RUN	git clone --branch stable/3.14 https://github.com/coin-or/Ipopt.git /opt/Ipopt \
    && mkdir -p /opt/Ipopt/build && cd /opt/Ipopt/build \
    && ../configure --prefix=/usr/local \
    && make install \
    && cd /opt && rm -r /opt/Ipopt


# Setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

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
RUN mkdir -p /home/${USER_NAME}/${PROJECT_NAME}
ENV HOME /home/${USER_NAME}
ENV WORKSPACE ${HOME}/${PROJECT_NAME}
RUN chown -R ${USER_NAME}:${GROUP_NAME} ${WORKSPACE}

# Shell
USER ${USER_NAME}
WORKDIR ${WORKSPACE}
ENV SHELL "/bin/bash"
