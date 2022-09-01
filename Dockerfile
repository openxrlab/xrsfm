FROM nvidia/cuda:10.2-devel-ubuntu18.04

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
    libgtest-dev

Run pip3 install numpy
