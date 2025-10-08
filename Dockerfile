# Start with a base image that has the modern libraries
FROM ubuntu:24.04

# Set up environment and install required packages
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
        build-essential \
        cmake \
        git \
        libpcl-dev \
        libboost-all-dev \
        libeigen3-dev \
        xterm \
        # Install a basic text editor or utility if needed
        && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app
