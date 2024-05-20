FROM nvidia/cuda:11.6.1-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i s:/archive.ubuntu.com:/mirrors.tuna.tsinghua.edu.cn/ubuntu:g /etc/apt/sources.list
RUN cat /etc/apt/sources.list
RUN apt-get clean && apt-get update
RUN apt-get install -y \
    wget \
    git \
    cmake \
    build-essential \
    python3 \
    python3-pip \
    libglew-dev \
    libatlas-base-dev \
    libgtest-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libeigen3-dev \
    libceres-dev \
    libopencv-dev

Run pip3 install numpy
