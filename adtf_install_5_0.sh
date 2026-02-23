#!/bin/bash

# Install Google Logging (glog) from source
echo "Cloning and installing Google Logging (glog)..."
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog /root/tof/glog && \
cd /root/tof/glog && \
mkdir build && cd build && \
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
cmake --build . --target install

# Install Libwebsockets from source
echo "Cloning and installing Libwebsockets..."
git clone --branch v4.3-stable --depth 1 https://github.com/warmcat/libwebsockets /root/tof/libwebsockets && \
cd /root/tof/libwebsockets && \
mkdir build && cd build && \
cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
cmake --build . --target install

# Install protobuf from source
echo "Cloning and installing protobuf..."
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf /root/tof/protobuf && \
cd /root/tof/protobuf && \
mkdir build && cd build && \
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/usr/local ../cmake && \
cmake --build . --target install

# Clone and build ADI ToF SDK with USE_DEPTH_COMPUTE_OPENSOURCE option
echo "Cloning and building ADI ToF SDK..."
git clone --branch multiple-tof-nvidia https://github.com/XYZReality/ToF_XYZ.git /root/tof/ToF_XYZ && \
cd /root/tof/ToF_XYZ && \
mkdir build && cd build && \
cmake -DNVIDIA=1 -DWITH_EXAMPLES=on -DUSE_DEPTH_COMPUTE_OPENSOURCE=ON -DCMAKE_PREFIX_PATH=/usr/local .. && \
cmake --build . --target install && ldconfig

echo "Installation completed successfully!"