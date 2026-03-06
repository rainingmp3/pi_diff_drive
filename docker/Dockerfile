ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        g++ \
        git \
        ninja-build \
        libtool \
        libtool-bin \
        autoconf \
        automake \
        pkg-config \
        unzip \
        curl \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch stable \
    https://github.com/neovim/neovim.git /tmp/neovim && \
    cd /tmp/neovim && \
    make CMAKE_BUILD_TYPE=Release && \
    make install && \
    rm -rf /tmp/neovim

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ripgrep \
        fd-find \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/* \
           /tmp/* \
           /var/tmp/*

RUN ln -sf /usr/bin/rg /usr/local/bin/rg 2>/dev/null || true
