# Start with a base image that has the modern libraries
FROM ubuntu:24.04

# Set up environment and install required packages
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
      build-essential cmake git \
      libpcl-dev libboost-all-dev libeigen3-dev \
      mesa-utils libgl1-mesa-dri libglx-mesa0 libgl1 libglu1-mesa \
      libx11-6 libxext6 libxrender1 libsm6 libice6 \
      libxrandr2 libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app
