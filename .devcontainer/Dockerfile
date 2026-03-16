FROM mcr.microsoft.com/devcontainers/base:ubuntu

# Combine system installs and cleanup into one layer
RUN apt-get update && apt-get install -y \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    build-essential \
    cmake \
    ninja-build \
    git \
    clangd \
    openocd \
    libusb-1.0-0 \
    usbutils \
    jq \
    python3 \
    python3-pip \
    python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies in a fresh layer
RUN python3 -m pip install --no-cache-dir --break-system-packages protobuf grpcio-tools

# Set up ST-Link permissions
RUN mkdir -p /etc/udev/rules.d && \
    echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666"' > /etc/udev/rules.d/99-stlink.rules