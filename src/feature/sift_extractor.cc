//
// Created by yzc on 19-4-25.
//

#include "sift_extractor.h"

#include <GL/glew.h>

#include <numeric>

namespace xrsfm{
std::unique_ptr<SiftGPU> SiftExtractor::create_siftgpu() {
  std::vector<int> gpu_indices(1, 0);
  int num_cuda_devices = 1;
  //    cudaGetDeviceCount(&num_cuda_devices);
  //    printf("----%d\n",num_cuda_devices);
  gpu_indices.resize(num_cuda_devices);
  std::iota(gpu_indices.begin(), gpu_indices.end(), 0);

  std::vector<std::string> sift_gpu_args;

  sift_gpu_args.emplace_back("./sift_gpu");

  // Use CUDA version by default if darkness adaptivity is disabled.
  if (!options.darkness_adaptivity && gpu_indices[0] < 0) {
    gpu_indices[0] = 0;
  }
  if (gpu_indices[0] >= 0) {
    sift_gpu_args.emplace_back("-cuda");
    sift_gpu_args.emplace_back(std::to_string(gpu_indices[0]));
  }

  // Darkness adaptivity (hidden feature). Significantly improves
  // distribution of features. Only available in GLSL version.
  sift_gpu_args.emplace_back("-da");

  // No verbose logging.
  sift_gpu_args.emplace_back("-v");
  sift_gpu_args.emplace_back("0");

  // Fixed maximum image dimension.
  sift_gpu_args.emplace_back("-maxd");
  sift_gpu_args.emplace_back(std::to_string(options.max_image_size));

  // Keep the highest level features.
  sift_gpu_args.emplace_back("-tc2");
  sift_gpu_args.emplace_back(std::to_string(options.max_num_features));

  // First octave level.
  sift_gpu_args.emplace_back("-fo");
  sift_gpu_args.emplace_back(std::to_string(options.first_octave));

  sift_gpu_args.emplace_back("-no");
  sift_gpu_args.emplace_back(std::to_string(options.num_octaves));

  // Number of octave levels.
  sift_gpu_args.emplace_back("-d");
  sift_gpu_args.emplace_back(std::to_string(options.octave_resolution));

  // Peak threshold.
  sift_gpu_args.emplace_back("-t");
  sift_gpu_args.emplace_back(std::to_string(options.peak_threshold));

  // Edge threshold.
  sift_gpu_args.emplace_back("-e");
  sift_gpu_args.emplace_back(std::to_string(options.edge_threshold));
  if (options.upright) {
    // Fix the orientation to 0 for upright features.
    sift_gpu_args.emplace_back("-ofix");
    // Maximum number of orientations.
    sift_gpu_args.emplace_back("-mo");
    sift_gpu_args.emplace_back("1");
  } else {
    // Maximum number of orientations.
    sift_gpu_args.emplace_back("-mo");
    sift_gpu_args.emplace_back(std::to_string(options.max_num_orientations));
  }
  std::vector<const char *> sift_gpu_args_cstr;
  sift_gpu_args_cstr.reserve(sift_gpu_args.size());
  for (const auto &arg : sift_gpu_args) {
    sift_gpu_args_cstr.emplace_back(arg.c_str());
  }

  std::unique_ptr<SiftGPU> sift_gpu;
  sift_gpu.reset(new SiftGPU);
  sift_gpu->ParseParam(sift_gpu_args_cstr.size(), sift_gpu_args_cstr.data());
  sift_gpu->gpu_index = gpu_indices[0];

  return sift_gpu;
}

SiftExtractor::SiftExtractor(int nfeatures) {
  options.max_num_features = nfeatures;
  sift_gpu1 = create_siftgpu();
}

SiftExtractor::~SiftExtractor() = default;

Eigen::MatrixXf L1RootNormalizeFeatureDescriptors(const Eigen::MatrixXf &descriptors) {
  Eigen::MatrixXf descriptors_normalized(descriptors.rows(), descriptors.cols());
  for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
    const float norm = descriptors.row(r).lpNorm<1>();
    descriptors_normalized.row(r) = descriptors.row(r) / norm;
    descriptors_normalized.row(r) = descriptors_normalized.row(r).array().sqrt();
  }
  return descriptors_normalized;
}

void SiftExtractor::ExtractFLOAT(const cv::Mat &_image, std::vector<cv::KeyPoint> &keypoints,
                                 FeatureDescriptors &descriptors) {
  cv::Mat image;
  if (_image.type() != CV_8UC1) {
    cv::cvtColor(_image, image, cv::COLOR_RGB2GRAY);
  } else {
    image = _image;
  }

  SiftGPU *sift_gpu = sift_gpu1.get();

  //    size_t num_features = 0;
  //    const float default_dog_threshold = sift_gpu->_dog_threshold;
  //
  //    const int max_iter = 5;
  //    const int min_num_feature = 100;
  //    int iter = 0;
  //    while (num_features < min_num_feature && iter++ < max_iter) {
  //        const int code = sift_gpu->RunSIFT(image.cols, image.rows, image.data, GL_LUMINANCE,
  //        GL_UNSIGNED_BYTE); const int kSuccessCode = 1; if (code != kSuccessCode) {
  //            printf("fail\n");
  //            return;
  //        }
  //        sift_gpu->_dog_threshold = 0.8 * sift_gpu->_dog_threshold;
  //        num_features = static_cast<size_t>(sift_gpu->GetFeatureNum());
  //    }
  //    sift_gpu->_dog_threshold = default_dog_threshold;
  const int code =
      sift_gpu->RunSIFT(image.cols, image.rows, image.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
  const int kSuccessCode = 1;
  if (code != kSuccessCode) {
    printf("fail\n");
    exit(0);
    return;
  }
  size_t num_features = sift_gpu->GetFeatureNum();
  keypoints.resize(num_features);
  descriptors.resize(num_features, 128);

  std::vector<SiftKeypoint> keypoints_data(num_features);
  sift_gpu->GetFeatureVector(keypoints_data.data(), descriptors.data());

  for (size_t i = 0; i < num_features; ++i) {
    keypoints[i] = cv::KeyPoint(keypoints_data[i].x, keypoints_data[i].y, 0);
    keypoints[i].size = keypoints_data[i].s;
    keypoints[i].angle = keypoints_data[i].o;
  }

  // L1 Norm
  descriptors = L1RootNormalizeFeatureDescriptors(descriptors);
  // L2 Norm
  //    descriptors = descriptors.rowwise().normalized();
}

void SiftExtractor::ExtractUINT8(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                                 UINT8Descriptors &descriptors) {
  FeatureDescriptors float_desc;
  ExtractFLOAT(image, keypoints, float_desc);
  descriptors = FeatureDescriptorsToUnsignedByte(float_desc);
}
}
