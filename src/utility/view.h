//
// Created by SENSETIME\yezhichao1 on 2020/10/18.
//

#ifndef ECIM_VIEW_H
#define ECIM_VIEW_H

#include <pangolin/pangolin.h>

#include <opencv2/opencv.hpp>

#include "base/map.h"
#include "viewer_handle.h"

namespace xrsfm {

void DrawFeature(const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints);

void DrawFeatureMatches(const cv::Mat &img1, const cv::Mat &img2, const std::vector<cv::KeyPoint> &kpts1,
                        const std::vector<cv::KeyPoint> &kpts2, const std::vector<Match> &matches,
                        const std::vector<char> mask = std::vector<char>(0));

void DrawFeatureMatches(const cv::Mat &img1, const cv::Mat &img2, const std::vector<Eigen::Vector2d> &kpts1,
                        const std::vector<Eigen::Vector2d> &kpts2, const std::vector<Match> &matches,
                        const std::vector<char> mask = std::vector<char>(0));

void DrawFeatureMatches1(const cv::Mat &img1, const std::vector<Eigen::Vector2d> &kpts1,
                         const std::vector<Eigen::Vector2d> &kpts2, const std::vector<Match> &matches,
                         const std::vector<char> mask = std::vector<char>(0));

void DrawFeatureFlow(const cv::Mat &img1, const std::vector<Eigen::Vector2d> &pts1,
                     const std::vector<Eigen::Vector2d> &pts2, const std::vector<Match> &matches,
                     const std::vector<int> &states, const std::string window_name = "feature flow");

void DrawCameras(const std::vector<Pose> &cameras, const std::vector<Eigen::Vector3d> colors, double camera_size);

void DrawPoints(const std::vector<Eigen::Vector3d> &points);

// TODO remove a viewer
class Viewer {
  public:
    void Init();
    void Draw(const Map &map, bool stop);
    void DrawKey(const Map &map, bool stop);
    void DrawLoop(const Map &map, const LoopInfo loop_info);
    void DrawGraph(const Map &map, bool stop);
    void DrawColor(const Map &map, bool stop);

    pangolin::OpenGlRenderState s_cam;
    pangolin::View d_cam;
    //    pangolin::MyHandler3D *handle;
    std::unique_ptr<pangolin::MyHandler3D> handler;
    //    void *handle;
    bool color_in_order_;
    double camera_size_ = 0.1;
    Map map;
};

} // namespace xrsfm
#endif // ECIM_VIEW_H
