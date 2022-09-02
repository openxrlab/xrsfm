//
// Created by SENSETIME\yezhichao1 on 2020/10/19.
//
#include "view.h"

#include <unistd.h>
#include <thread>
#include <pangolin/pangolin.h>

#include "viewer_handle.h"
#include "utility/global.h"

#define OPEN_VIEW

namespace xrsfm {
constexpr int width = 1024, height = 768, focal = 500;
const cv::Scalar cv_green(0, 255, 0), cv_gray(80, 80, 80), cv_blue(255, 0, 0),
    cv_yellow(0, 255, 255), cv_red(0, 0, 255), cv_brown(40, 140, 140);

void DrawMatchesColmap(const std::string dir_path, const FramePair &fp,
                       const Map &map) {
    auto &f1 = map.frames_[fp.id1];
    auto &f2 = map.frames_[fp.id2];
    cv::Mat image1 = cv::imread(dir_path + f1.name);
    cv::Mat image2 = cv::imread(dir_path + f2.name);
    DrawFeatureMatches(image1, image2, f1.points, f2.points, fp.matches);
    cv::waitKey();
}

void DrawFrameMatches(const Map &map, const std::vector<cv::Mat> &images,
                      int id) {
    for (auto &t_frame_pair : map.frame_pairs_) {
        if (t_frame_pair.id1 == id || t_frame_pair.id2 == id) {
            int id1 = t_frame_pair.id1;
            int id2 = t_frame_pair.id2;
            printf("Find Pair: %d %d %d\n", id1, id2, t_frame_pair.inlier_num);
            DrawFeatureMatches(images[id1], images[id2],
                               map.frames_[id1].points, map.frames_[id2].points,
                               t_frame_pair.matches, t_frame_pair.inlier_mask);
            DrawFeatureMatches1(images[id1], map.frames_[id1].points,
                                map.frames_[id2].points, t_frame_pair.matches,
                                t_frame_pair.inlier_mask);
        }
    }
}

void DrawFeature(const cv::Mat &image,
                 const std::vector<cv::KeyPoint> &keypoints) {
    for (const auto &kpt : keypoints) {
        cv::circle(image, kpt.pt, 1, cv::Scalar(0, 255, 0), -1);
    }
    cv::imshow("", image);
    cv::waitKey(0);
}

void DrawFeatureMatches(const cv::Mat &img1, const cv::Mat &img2,
                        const std::vector<cv::KeyPoint> &kpts1,
                        const std::vector<cv::KeyPoint> &kpts2,
                        const std::vector<Match> &matches,
                        const std::vector<char> mask) {
    int rows = img1.rows > img2.rows ? img1.rows : img2.rows;
    cv::Mat img(rows, img1.cols + img2.cols, img1.type(), cv::Scalar(0, 0, 0));
    cv::Mat img1p = img(cv::Rect(0, 0, img1.cols, img1.rows));
    cv::Mat img2p = img(cv::Rect(img1.cols, 0, img2.cols, img2.rows));
    img1.copyTo(img1p);
    img2.copyTo(img2p);
    cv::Scalar color;
    for (size_t i = 0; i < matches.size(); ++i) {
        auto match = matches[i];
        // std::cout<<match.id1<<" "<<match.id2<<std::endl;
        color = mask.empty() || mask[i] ? cv_green : cv_red;
        // if (!mask.empty() && !mask[i])
        line(img, kpts1[match.id1].pt,
             kpts2[match.id2].pt + cv::Point2f(img1.cols, 0), color, 1);
    }
    constexpr double scale = 1.0;
    resize(img, img,
           cv::Size(int(img.cols * scale), int(img.rows * scale))); /**/
    imshow("0", img);
}

void DrawFeatureMatches(const cv::Mat &img1, const cv::Mat &img2,
                        const std::vector<vector2> &kpts1,
                        const std::vector<vector2> &kpts2,
                        const std::vector<Match> &matches,
                        const std::vector<char> mask) {
    std::vector<cv::KeyPoint> kpts1_;
    for (auto &pt : kpts1) {
        cv::KeyPoint kpt;
        kpt.pt = cv::Point2f(pt.x(), pt.y());
        kpts1_.emplace_back(kpt);
    }
    std::vector<cv::KeyPoint> kpts2_;
    for (auto &pt : kpts2) {
        cv::KeyPoint kpt;
        kpt.pt = cv::Point2f(pt.x(), pt.y());
        kpts2_.emplace_back(kpt);
    }
    DrawFeatureMatches(img1, img2, kpts1_, kpts2_, matches, mask);
}

void DrawFeatureFlow(const cv::Mat &img1, const std::vector<vector2> &pts1,
                     const std::vector<vector2> &pts2,
                     const std::vector<Match> &matches,
                     const std::vector<int> &states,
                     const std::string window_name) {
    std::vector<cv::KeyPoint> kpts1;
    for (auto &pt : pts1) {
        kpts1.emplace_back(cv::KeyPoint(pt.x(), pt.y(), -1));
    }
    std::vector<cv::KeyPoint> kpts2;
    for (auto &pt : pts2) {
        ;
        kpts2.emplace_back(cv::KeyPoint(pt.x(), pt.y(), -1));
    }

    cv::Mat img = img1;

    for (size_t i = 0; i < matches.size(); ++i) {
        auto match = matches[i];
        cv::Scalar color, dst_color, src_color = cv_yellow;
        if (states[i] == 1)
            color = dst_color = cv_green;
        else if (states[i] == 0)
            color = dst_color = cv_red;
        else if (states[i] == -1) {
            color = dst_color = cv_brown;
        } else {
            std::cerr << "unexpected state";
        }
        cv::circle(img, kpts1[match.id1].pt, 2, src_color);
        line(img, kpts1[match.id1].pt, kpts2[match.id2].pt, color, 1);
        cv::circle(img, kpts2[match.id2].pt, 2, dst_color);
    }

    constexpr double scale = 1.0;
    resize(img, img, cv::Size(int(img.cols * scale), int(img.rows * scale)));
    imshow(window_name, img);
}

void DrawFeatureMatches1(const cv::Mat &img1,
                         const std::vector<cv::KeyPoint> &kpts1,
                         const std::vector<cv::KeyPoint> &kpts2,
                         const std::vector<Match> &matches,
                         const std::vector<char> mask) {
    cv::Mat img = img1.clone();
    cv::Scalar color;

    for (size_t i = 0; i < matches.size(); ++i) {
        auto match = matches[i];
        color = mask.empty() || mask[i] ? cv_green : cv_red;
        cv::circle(img, kpts1[match.id1].pt, 2, cv_blue);
        line(img, kpts1[match.id1].pt, kpts2[match.id2].pt, color, 1);
        cv::circle(img, kpts2[match.id2].pt, 2, cv_yellow);
    }

    constexpr double scale = 1.0;
    resize(img, img, cv::Size(int(img.cols * scale), int(img.rows * scale)));
    imshow("1", img);
}

void DrawFeatureMatches1(const cv::Mat &img1, const std::vector<vector2> &kpts1,
                         const std::vector<vector2> &kpts2,
                         const std::vector<Match> &matches,
                         const std::vector<char> mask) {
    std::vector<cv::KeyPoint> kpts1_;
    for (auto &pt : kpts1) {
        cv::KeyPoint kpt;
        kpt.pt = cv::Point2f(pt.x(), pt.y());
        kpts1_.emplace_back(kpt);
    }
    std::vector<cv::KeyPoint> kpts2_;
    for (auto &pt : kpts2) {
        cv::KeyPoint kpt;
        kpt.pt = cv::Point2f(pt.x(), pt.y());
        kpts2_.emplace_back(kpt);
    }
    DrawFeatureMatches1(img1, kpts1_, kpts2_, matches, mask);
}

void DrawCameras(const std::vector<Pose> &cameras,
                 const std::vector<vector3> colors, double camera_size) {
    const float w = camera_size;
    const float h = w * 0.5f;
    const float z = w * 0.6f;
    const int num_camera = cameras.size();
    for (int i = 0; i < num_camera; ++i) {
        const auto &camera = cameras[i];
        glPushMatrix();
        const Eigen::Matrix3d R = camera.q.toRotationMatrix();
        const vector3 t = camera.t;
        Eigen::Matrix4d Tcw;
        Tcw << R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2), t(1),
            R(2, 0), R(2, 1), R(2, 2), t(2), 0, 0, 0, 1;
        Eigen::Matrix4d Twc = Tcw.inverse();
        glMultMatrixd(Twc.data());
        glColor3d(colors[i](0), colors[i](1), colors[i](2));

        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);

        //        glColor3f(1,0,0);
        //        glVertex3f(0,0,0);
        //        glVertex3f(2*w,0,0);
        //
        //        glColor3f(0,1,0);
        //        glVertex3f(0,0,0);
        //        glVertex3f(0,2*w,0);
        //
        //        glColor3f(0,0,1);
        //        glVertex3f(0,0,0);
        //        glVertex3f(0,0,2*w);

        glEnd();
        glPopMatrix();
    }
}

void DrawPoints(const std::vector<vector3> &points) {
    glPointSize(1.0f);
    glBegin(GL_POINTS);
    for (const auto &pt : points) {
        glColor3f(0.0f, 0.0f, 0.0f);
        glVertex3d(pt(0), pt(1), pt(2));
    }
    glEnd();
}

void DrawPoints(const std::vector<vector3> &points,
                const std::vector<vector3> colors) {
    glPointSize(1.0f);
    glBegin(GL_POINTS);
    for (int i = 0; i < points.size(); ++i) {
        glColor3d(colors[i](0), colors[i](1), colors[i](2));
        glVertex3d(points[i](0), points[i](1), points[i](2));
    }
    glEnd();
}

} // namespace xrsfm
