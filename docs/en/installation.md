

# Installation

## Build from Source (Linux)

Requirements
+ C++17
+ GCC 7.5+
+ CMake 3.16+
+ CUDA 7.5+

Dependencies from the default Ubuntu repositories:
```shell
    sudo apt-get install \
        wget \
        git \
        cmake \
        build-essential \
        python3 \
        python3-pip \
        libglew-dev \
        libatlas-base-dev \
        libgtest-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        qtbase5-dev \
        libqt5opengl5-dev \
        libeigen3-dev \
        libceres-dev \
        libopencv-dev
```

If the version of CMake is less than 3.16, update it.
```shell
wget https://cmake.org/files/v3.21/cmake-3.21.0-linux-x86_64.tar.gz
tar -xf cmake-3.21.0-linux-x86_64.tar.gz
cp -r cmake-3.21.0-linux-x86_64 /usr/share/
ln -sf /usr/share/cmake-3.21.0-linux-x86_64/bin/cmake /usr/bin/cmake
```

Compile xrsfm
```shell
git clone https://github.com/openxrlab/xrsfm.git
cd xrsfm && cmake -B build && cmake --build build -j4
```

### Dockerfile

We provide a [Dockerfile](../../Dockerfile) to build an image. Ensure that you are using [docker version](https://docs.docker.com/engine/install/) >=19.03 and `"default-runtime": "nvidia"` in daemon.json.

```shell
docker build -t xrsfm .
```

Run it with

```shell
docker run --name xrsfm-container --gpus all --network=host -it xrsfm
```

Compile xrsfm

```shell
git clone https://github.com/openxrlab/xrsfm.git
cd xrsfm && cmake -B build && cmake --build build -j4
```
