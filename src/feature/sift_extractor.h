//
// Created by yzc on 19-4-25.
//

#ifndef XRSFM_SRC_FEATURE_SIFT_EXTRACTOR_H
#define XRSFM_SRC_FEATURE_SIFT_EXTRACTOR_H

#include <SiftGPU.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "base/types.h"

namespace xrsfm {
template <typename T1, typename T2> T2 TruncateCast(const T1 value) {
    return std::min(
        static_cast<T1>(std::numeric_limits<T2>::max()),
        std::max(static_cast<T1>(std::numeric_limits<T2>::min()), value));
}

inline UINT8Descriptors
FeatureDescriptorsToUnsignedByte(const FeatureDescriptors &descriptors) {
    UINT8Descriptors descriptors_unsigned_byte(descriptors.rows(),
                                               descriptors.cols());
    for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
        for (Eigen::MatrixXf::Index c = 0; c < descriptors.cols(); ++c) {
            const float scaled_value = std::round(512.0f * descriptors(r, c));
            descriptors_unsigned_byte(r, c) =
                TruncateCast<float, uint8_t>(scaled_value);
        }
    }
    return descriptors_unsigned_byte;
}

struct SiftExtractionOptions {
    // Number of threads for feature extraction.
    int num_threads = -1;

    // Whether to use the GPU for feature extraction.
    bool use_gpu = true;

    // Index of the GPU used for feature extraction. For multi-GPU extraction,
    // you should separate multiple GPU indices by comma, e.g., "0,1,2,3".
    std::string gpu_index = "-1";

    // Maximum image size, otherwise image will be down-scaled.
    int max_image_size = 3200;

    // Maximum number of features to detect, keeping larger-scale features.
    int max_num_features = 8192;

    // First octave in the pyramid, i.e. -1 upsamples the image by one level.
    int first_octave = -1;

    // Number of octaves.
    int num_octaves = 4;

    // Number of levels per octave.
    int octave_resolution = 3;

    // Peak threshold for detection.
    double peak_threshold = 0.02 / octave_resolution;

    // Edge threshold for detection.
    double edge_threshold = 10.0;

    // Estimate affine shape of SIFT features in the form of oriented ellipses
    // as opposed to original SIFT which estimates oriented disks.
    bool estimate_affine_shape = false;

    // Maximum number of orientations per keypoint if not estimate_affine_shape.
    int max_num_orientations = 1; // note here:2

    // Fix the orientation to 0 for upright features.
    bool upright = false;

    // Whether to adapt the feature detection depending on the image darkness.
    // Note that this feature is only available in the OpenGL SiftGPU version.
    bool darkness_adaptivity = false;

    // Domain-size pooling parameters. Domain-size pooling computes an average
    // SIFT descriptor across multiple scales around the detected scale. This
    // was proposed in "Domain-Size Pooling in Local Descriptors and Network
    // Architectures", J. Dong and S. Soatto, CVPR 2015. This has been shown to
    // outperform other SIFT variants and learned descriptors in "Comparative
    // Evaluation of Hand-Crafted and Learned Local Features", Schönberger,
    // Hardmeier, Sattler, Pollefeys, CVPR 2016.
    bool domain_size_pooling = false;
    double dsp_min_scale = 1.0 / 6.0;
    double dsp_max_scale = 3.0;
    int dsp_num_scales = 10;

    enum class Normalization {
        // L1-normalizes each descriptor followed by element-wise square
        // rooting.
        // This normalization is usually better than standard L2-normalization.
        // See "Three things everyone should know to improve object retrieval",
        // Relja Arandjelovic and Andrew Zisserman, CVPR 2012.
        L1_ROOT,
        // Each vector is L2-normalized.
        L2,
    };
    Normalization normalization = Normalization::L1_ROOT;

    bool Check() const;
};

class SiftExtractor {
  public:
    SiftExtractor(int nfeatures = 8192);
    ~SiftExtractor();
    void ExtractFLOAT(const cv::Mat &image,
                      std::vector<cv::KeyPoint> &keypoints,
                      FeatureDescriptors &descriptors);
    void ExtractUINT8(const cv::Mat &image,
                      std::vector<cv::KeyPoint> &keypoints,
                      UINT8Descriptors &descriptors);
    std::unique_ptr<SiftGPU> create_siftgpu();
    SiftExtractionOptions options;
    std::unique_ptr<SiftGPU> sift_gpu1;
};
} // namespace xrsfm

#endif // WSFM_SIFT_EXTRACTOR_H
