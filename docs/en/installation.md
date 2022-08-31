

# Installation

## Build from Source (Linux) 

Requirements
+ C++17
+ GCC 7.5+
+ CMake 3.16+
+ CUDA 7.5+
+ XRPrimer
+ Pangolin

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
        libgtest-dev
```

Install cmake
```shell
wget https://cmake.org/files/v3.21/cmake-3.21.0-linux-x86_64.tar.gz
tar -xf cmake-3.21.0-linux-x86_64.tar.gz
cp -r cmake-3.21.0-linux-x86_64 /usr/share/
ln -sf /usr/share/cmake-3.21.0-linux-x86_64/bin/cmake /usr/bin/cmake
```

Install [XRPRimer](https://github.com/openxrlab/xrprimer)
```shell
git clone git@github.com:openxrlab/xrprimer.git
cd xrprimer
cmake -S . -Bbuild -DBUILD_EXTERNAL=ON -DCMAKE_BUILD_TYPE=Release 
cmake --build build --target install -j4
```

Install [Pangolin](git@github.com:stevenlovegrove/Pangolin.git)
```shell
git clone git@github.com:stevenlovegrove/Pangolin.git
cd Pangolin && cmake -B build && cmake --build build -j4 
sudo make install
```

Compile XRSfM::
```shell
git clone git@github.com:openxrlab/xrsfm.git
cd xrsfm && cmake -B build && cmake --build build -j4
```

### Dockerfile

We provide a [Dockerfile](../../Dockerfile) to build an image. Ensure that you are using [docker version](https://docs.docker.com/engine/install/) >=19.03 and `"default-runtime": "nvidia"` in daemon.json.

```shell
docker build -t xrsfm .
```

Run it with

```shell
docker run --gpus all --network=host -it xrsfm
```