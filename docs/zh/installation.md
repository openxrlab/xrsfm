

# 安装

## 从源码安装 (Linux) 

版本要求
+ C++17
+ GCC 7.5+
+ CMake 3.15+
+ CUDA 7.0+
+ XRPrimer

来自默认Ubuntu存储库的依赖项:

    sudo apt-get install \
        git \
        cmake \
        build-essential \
        libgoogle-glog-dev \ 
        libgflags-dev \
        libgl1-mesa-dev \
        libglew-dev \
        libgtest-dev


安装 [XRPRimer](https://github.com/openxrlab/xrprimer)

安装 [Pangolin](git@github.com:stevenlovegrove/Pangolin.git)
```
    git clone git@github.com:openxrlab/xrsfm.git
    cd Pangolin && cmake -B build && cmake --build build -j4 
    sudo make install
```

编译 XRSfM
```
    git clone git@github.com:openxrlab/xrsfm.git
    cd xrsfm && cmake -B build && cmake --build build -j4
```
### Dockerfile



### Docker image
 
