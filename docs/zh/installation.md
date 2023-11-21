

# 安装

## 从源码安装 (Linux)

版本要求
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
        libgoogle-glog-dev
```

如果cmake版本小于3.16，按照以下步骤更新
```shell
wget https://cmake.org/files/v3.21/cmake-3.21.0-linux-x86_64.tar.gz
tar -xf cmake-3.21.0-linux-x86_64.tar.gz
cp -r cmake-3.21.0-linux-x86_64 /usr/share/
ln -sf /usr/share/cmake-3.21.0-linux-x86_64/bin/cmake /usr/bin/cmake
```

安装 [XRPRimer](https://github.com/openxrlab/xrprimer)
```shell
git clone git@github.com:openxrlab/xrprimer.git
cd xrprimer
git checkout xrslam-opencv3.4.7
cmake -S. -Bbuild -DBUILD_EXTERNAL=ON -DCMAKE_BUILD_TYPE=Release -DENABLE_PRECOMPILED_HEADERS=OFF
cmake --build build --target install -j4
```

确保xrsfm和xrprimer的根目录保持一致。
```
xrprimer
├──
...
xrsfm
├── docs
├── scripts
├── src
...
```


编译 XRSfM
```shell
git clone git@github.com:openxrlab/xrsfm.git
cd xrsfm && cmake -B build && cmake --build build -j4
```

注意：如果您在编译xrprimer时遇到困难，可以尝试在没有xrprimer的情况下安装xrsfm。这需要OpenCV和Ceres-Solver。然后，您可以使用以下命令安装xrsfm：
```shell
cd xrsfm && cmake -B build -DXRPRIMER_ENABLED=OFF && cmake --build build -j4
```

### 通过Docker镜像运行

We provide a [Dockerfile](../../Dockerfile) to build an image. Ensure that you are using [docker version](https://docs.docker.com/engine/install/) >=19.03 and `"default-runtime": "nvidia"` in daemon.json.

```shell
docker build -t xrsfm .
```

Run it with

```shell
docker run --gpus all --network=host -it xrsfm
```
