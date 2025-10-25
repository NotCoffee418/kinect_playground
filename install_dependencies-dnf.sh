#!/bin/bash

echo "Installing libfreenect2 dependencies..."
sudo dnf install -y \
    libusb1-devel \
    turbojpeg-devel \
    glfw-devel \
    cmake \
    gcc-c++ \
    git

echo "Downloading stb_image_write.h..."
curl -O https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h

echo "Done!"