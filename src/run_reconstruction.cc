
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "base/map.h"
#include "mapper/incremental_mapper.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "geometry/epipolar_geometry.hpp"

using namespace xrsfm;

bool self_calibration(std::vector<Frame> &frames,
                      std::vector<FramePair> &frame_pairs, double w, double h,
                      double &estimated_focal) {
    std::map<float, float> focal_score_map;
    for (int k = 0; k < 100; ++k) {
        const double f = 0.25 * w + 2 * w * k / 100;
        focal_score_map[f] = 0;
    }

    std::vector<int> inlier_vec;
    for (const auto &fp : frame_pairs) {
        inlier_vec.push_back(fp.inlier_num);
    }
    std::sort(inlier_vec.begin(), inlier_vec.end());
    const int inlier_th = inlier_vec.at(int(inlier_vec.size()) / 2);

    int count = 0;
    for (const auto &fp : frame_pairs) {
        if (fp.inlier_num < inlier_th)
            continue;

        std::vector<Eigen::Vector2d> points1, points2;
        for (const auto &match : fp.matches) {
            points1.push_back(frames.at(fp.id1).points[match.id1]);
            points2.push_back(frames.at(fp.id2).points[match.id2]);
        }
        double distance = 0;
        for (int i = 0; i < points1.size(); ++i) {
            points1.at(i).x() -= 0.5 * w;
            points1.at(i).y() -= 0.5 * h;
            points2.at(i).x() -= 0.5 * w;
            points2.at(i).y() -= 0.5 * h;
            distance = (points2.at(i) - points1.at(i)).norm();
        }
        distance /= points1.size();
        FramePair tmp_fp;
        SolveFundamentalCOLMAP(points1, points2, tmp_fp);

        for (int k = 0; k < 100; ++k) {
            const double f = 0.25 * w + 2 * w * k / 100;
            Eigen::Matrix3d A = tmp_fp.F;
            A(0, 2) /= f;
            A(1, 2) /= f;
            A(2, 0) /= f;
            A(2, 1) /= f;
            A(2, 2) /= (f * f);
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU |
                                                         Eigen::ComputeFullV);
            Eigen::Vector3d s = svd.singularValues();
            s /= s(0);
            focal_score_map[f] += s(1);
        }
        count++;
    }

    std::vector<std::pair<float, float>> focal_vec(focal_score_map.begin(),
                                                   focal_score_map.end());
    std::sort(
        focal_vec.begin(), focal_vec.end(),
        [](const std::pair<float, float> &a, const std::pair<float, float> &b) {
            return a.second > b.second;
        });

    estimated_focal = focal_vec.at(0).first;
    std::cout << focal_vec.at(0).first << " " << focal_vec.at(0).second / count
              << std::endl;

    return true;
}

inline void DrawMatches(const cv::Mat &image1, const cv::Mat &image2,
                        const std::vector<cv::KeyPoint> &kpts1,
                        const std::vector<cv::KeyPoint> &kpts2,
                        const std::vector<cv::DMatch> &matches,
                        const std::string name, const double scale = 0.15) {
    cv::Mat output_image(cv::Size(image1.cols, image1.rows + image2.rows),
                         image1.type());
    cv::Rect roi1(0, 0, image1.cols, image1.rows);
    image1.copyTo(output_image(roi1));
    cv::Rect roi2(0, image1.rows, image2.cols, image2.rows);
    image2.copyTo(output_image(roi2));
    cv::resize(output_image, output_image,
               cv::Size(scale * output_image.cols, scale * output_image.rows));

    for (auto &kpt : kpts1) {
        cv::circle(output_image, scale * kpt.pt, 2, cv::Scalar(255, 0, 0));
    }
    for (auto &kpt : kpts2) {
        auto pt2 = scale * (cv::Point2f(0, image1.rows) + kpt.pt);
        cv::circle(output_image, pt2, 2, cv::Scalar(255, 0, 0));
    }
    for (auto &match : matches) {
        auto pt1 = scale * kpts1.at(match.queryIdx).pt;
        auto pt2 =
            scale * (cv::Point2f(0, image1.rows) + kpts2.at(match.trainIdx).pt);
        cv::line(output_image, pt1, pt2, cv::Scalar(0, 255, 0));
    }
    cv::imshow("", output_image);
    cv::waitKey();
    // cv::imwrite(name, output_image);
}

void PreProcess(const std::string dir_path, const std::string camera_path,
                Map &map) {
    std::vector<Frame> frames;
    std::vector<FramePair> frame_pairs;
    ReadFeatures(dir_path + "ftr.bin", frames, true);
    ReadFramePairs(dir_path + "fp.bin", frame_pairs);

    // view matches
    auto fp = FindPair(frame_pairs, 0, 13);
    // cv::Mat image1 = cv::imread(dir_path + "/images/" + frames.at(0).name);
    // cv::Mat image2 = cv::imread(dir_path + "/images/" + frames.at(13).name);
    // std::vector<cv::DMatch> matches;
    // for (const auto &match : fp.matches) {
    //     matches.push_back(cv::DMatch(match.id1, match.id2, match.distance));
    // }
    // DrawMatches(image1, image2, frames.at(0).keypoints_,
    //             frames.at(13).keypoints_, matches, "", 1.0);

    // convert keypoint to points(for reconstruction)
    for (auto &frame : frames) {
        const int num_points = frame.keypoints_.size();
        frame.points.clear();
        frame.track_ids_.assign(num_points, -1);
        for (const auto &kpt : frame.keypoints_) {
            const auto &pt = kpt.pt;
            Eigen::Vector2d ept(pt.x, pt.y), eptn;
            frame.points.push_back(ept);
        }
    }

    // set cameras
    std::map<int, Camera> cameras;
    if (camera_path == "auto") {
        std::vector<ImageSize> image_size;
        LoadImageSize(dir_path + "size.bin", image_size);
        const int w = image_size.at(0).width;
        const int h = image_size.at(0).height;

        double focal = 0.5 * w;
        // self_calibration(frames, frame_pairs, w, h, focal);
        std::vector<xrsfm::FramePair> fps = {fp};
        self_calibration(frames, fps, w, h, focal);

        Camera seq(0, "SIMPLE_RADIAL", w, h);
        seq.params_ = {focal, 0.5 * w, 0.5 * h, 0};
        cameras[0] = seq;
    } else {
        cameras = ReadCamerasText(camera_path);
    }

    CHECK_EQ(cameras.size(), 1);
    const int camera_id = cameras.begin()->first;

    for (auto &frame : frames) {
        frame.camera_id = camera_id;
    }

    map.camera_map_ = cameras;
    map.frames_ = frames;
    map.frame_pairs_ = frame_pairs;

    map.RemoveRedundancyPoints();
    map.Init();
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    // 1.Read Config
    std::string bin_path, camera_path, output_path;
    int init_id1 = -1, init_id2 = -1;
    if (argc <= 2) {
        std::string config_path = "./config_seq.json";
        if (argc == 2) {
            config_path = argv[1];
        }
        auto config_json = LoadJSON(config_path);
        bin_path = config_json["bin_path"];
        camera_path = config_json["camera_path"];
        output_path = config_json["output_path"];
        init_id1 = config_json["init_id1"];
        init_id2 = config_json["init_id2"];
    } else if (argc >= 4 && argc <= 6) {
        bin_path = argv[1];
        camera_path = argv[2];
        output_path = argv[3];
        if (argc >= 5) {
            init_id1 = std::stoi(argv[4]);
        }
        if (argc == 6) {
            init_id2 = std::stoi(argv[5]);
        }
    } else {
        exit(-1);
    }
    std::cout << "Read Config Done!" << std::endl;

    // 2. Map PreProcess
    Map map;
    PreProcess(bin_path, camera_path, map);
    std::cout << "PreProcess Done!" << std::endl;

    // 3. Map Reconstruction
    IncrementalMapper imapper;
    imapper.options.init_id1 = init_id1;
    imapper.options.init_id2 = init_id2;
    imapper.options.correct_pose = true;
    imapper.options.stop_when_register_fail = true;
    imapper.Reconstruct(map);
    std::cout << "Reconstruction Done!" << std::endl;

    // 4. Output
    WriteColMapDataBinary(output_path, map);

    return 0;
}
