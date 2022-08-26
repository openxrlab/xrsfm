

# Installation

## Build from Source (Linux) 

Requirements
+ C++17
+ GCC 7.5+
+ CMake 3.15+
+ CUDA 7.0+
+ XRPrimer

Dependencies from the default Ubuntu repositories:

    sudo apt-get install \
        git \
        cmake \
        build-essential \
        libgoogle-glog-dev \ 
        libgflags-dev \
        libgl1-mesa-dev \
        libglew-dev \
        libgtest-dev


Install [XRPRimer](https://github.com/openxrlab/xrprimer)

Install [Pangolin](git@github.com:stevenlovegrove/Pangolin.git)
```
    git clone git@github.com:openxrlab/xrsfm.git
    cd Pangolin && cmake -B build && cmake --build build -j4
```

Compile XRSfM::
```
    git clone git@github.com:openxrlab/xrsfm.git
    cd xrsfm && cmake -B build && cmake --build build -j4
```
### Dockerfile



### Docker image
 
