

# 安装

## 从源码安装 (Linux)

版本要求:
+ C++17
+ GCC 7.5+
+ CMake 3.16+
+ CUDA 7.5+

来自默认Ubuntu存储库的依赖项:
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

如果cmake版本小于3.16，按照以下步骤更新:
```shell
wget https://cmake.org/files/v3.21/cmake-3.21.0-linux-x86_64.tar.gz
tar -xf cmake-3.21.0-linux-x86_64.tar.gz
cp -r cmake-3.21.0-linux-x86_64 /usr/share/
ln -sf /usr/share/cmake-3.21.0-linux-x86_64/bin/cmake /usr/bin/cmake
```

编译 XRSfM:
```shell
git clone https://github.com/openxrlab/xrsfm.git
cd xrsfm && cmake -B build && cmake --build build -j4
```

### Dockerfile

我们提供了[Dockerfile](../../Dockerfile)文件来方便环境配置。
使用前请确认[docker version](https://docs.docker.com/engine/install/) >=19.03，并且在daemon.json文件中已经设置`"default-runtime": "nvidia"`。

```shell
docker build -t xrsfm .
```
启动容器:
```shell
docker run --name xrsfm-container --gpus all --network=host -it xrsfm
```

编译 XRSfM:
```shell
git clone https://github.com/openxrlab/xrsfm.git
cd xrsfm && cmake -B build && cmake --build build -j4
```
