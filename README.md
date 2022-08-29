# XRSfM

## Introduction

English | [简体中文](README_CN.md)

XRSfM is an open-source SfM codebase. It is a part of the [OpenXRLab](https://openxrlab.com/) project.
The software is licensed under the xxxx license. 
If you use the efficient matching method for your research, please cite:

```bibtex
@inproceedings{ye2020efficient,
  title={Efficient covisibility-based image matching for large-scale sfm},
  author={Ye, Zhichao and Zhang, Guofeng and Bao, Hujun},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  year={2020}
}
```

## Getting Started

1.Build the library manually as described in [installation.md](docs/en/installation.md).

2.Download the provided test data or prepare data with your own images.

3.Run the following script to automatically reconstruction:
```
python3 ./scripts/auto_reconstruction.py --data_path ${your_data_path}$.
```

## Build your own ARDemo

In addition to the reconstruction function, we have built a complete process to facilitate users to build location-based ardemo. For details, please refer to [ARDemo] ().

## License

This project is released under the [Apache 2.0 license](LICENSE).

## Acknowledgement

XRSfM is an open source project that is contributed by researchers and engineers from both the academia and the industry.
We appreciate all the contributors who implement their methods or add new features, as well as users who give valuable feedbacks.


## Projects in OpenXRLab

- [XRPrimer](https://github.com/openxrlab/xrprimer): OpenXRLab foundational library for XR-related algorithms.
- [XRSLAM](https://github.com/openxrlab/xrslam): OpenXRLab Visual-inertial SLAM Toolbox and Benchmark.
- [XRSfM](https://github.com/openxrlab/xrsfm): OpenXRLab Structure-from-Motion Toolbox and Benchmark.
- [XRLocalization](https://github.com/openxrlab/xrlocalization): OpenXRLab Visual Localization Toolbox and Server.
- [XRMoCap](https://github.com/openxrlab/xrmocap): OpenXRLab Multi-view Motion Capture Toolbox and Benchmark.
- [XRMoGen](https://github.com/openxrlab/xrmogen): OpenXRLab Human Motion Generation Toolbox and Benchmark.
- [XRNeRF](https://github.com/openxrlab/xrnerf): OpenXRLab Neural Radiance Field (NeRF) Toolbox and Benchmark.
