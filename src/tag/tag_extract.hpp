
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <experimental/filesystem>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

#include "base/map.h"
#include "base/camera.h"
#include "geometry/track_processor.h"
#include "optimization/ba_solver.h"
#include "optimization/cost_factor_ceres.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"
#include "utility/viewer.h"

namespace xrsfm {

std::map<std::string, std::map<int, std::vector<vector2>>>
tag_extract(std::string image_dir) {
    std::vector<std::string> image_vec;
    for (const auto &fe :
         std::experimental::filesystem::directory_iterator(image_dir)) {
        image_vec.emplace_back(fe.path().filename());
    }
    std::sort(image_vec.begin(), image_vec.end());

    std::map<std::string, std::map<int, std::vector<vector2>>> tag_info_vec;

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    for (auto &image_name : image_vec) {
        std::string img_path = image_dir + image_name;
        cv::Mat frame, gray;
        frame = cv::imread(img_path.c_str());
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = {.width = gray.cols,
                         .height = gray.rows,
                         .stride = gray.cols,
                         .buf = gray.data};

        zarray_t *detections = apriltag_detector_detect(td, &im);
        const int num_detect = zarray_size(detections);
        if (num_detect == 0)
            continue;

        auto &tag_info = tag_info_vec[image_name];
        for (int i = 0; i < num_detect; i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            // Do stuff with detections here.
            std::vector<vector2> p_vec(4);
            for (int k = 0; k < 4; ++k)
                p_vec[k] = vector2(det->p[k][0], det->p[k][1]);
            tag_info[det->id] = p_vec;
        }
    }

    // Cleanup.
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);

    return tag_info_vec;
}

std::map<std::string, std::map<int, std::vector<vector2>>>
tag_extract(const std::string &image_dir,
            const std::vector<std::string> &image_vec) {
    std::map<std::string, std::map<int, std::vector<vector2>>> tag_info_vec;
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    for (auto &image_name : image_vec) {
        std::string img_path = image_dir + image_name;
        cv::Mat frame, gray;
        frame = cv::imread(img_path.c_str());
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = {.width = gray.cols,
                         .height = gray.rows,
                         .stride = gray.cols,
                         .buf = gray.data};

        zarray_t *detections = apriltag_detector_detect(td, &im);
        const int num_detect = zarray_size(detections);
        if (num_detect == 0)
            continue;

        auto &tag_info = tag_info_vec[image_name];
        for (int i = 0; i < num_detect; i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            // Do stuff with detections here.
            std::vector<vector2> p_vec(4);
            for (int k = 0; k < 4; ++k)
                p_vec[k] = vector2(det->p[k][0], det->p[k][1]);
            tag_info[det->id] = p_vec;
        }
    }

    // Cleanup.
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);

    return tag_info_vec;
}

std::vector<vector3> get_tag(double tag_length) {
    // double tag_len = 0.113;
    std::vector<vector3> pt_tag(4);
    pt_tag[0] = vector3(0, 0, 0);
    pt_tag[1] = vector3(tag_length, 0, 0);
    pt_tag[2] = vector3(tag_length, 0, tag_length);
    pt_tag[3] = vector3(0, 0, tag_length);
    return pt_tag;
}

void tag_refine(std::string image_dir, std::string map_dir,
                const double tag_length, std::string output_path) {
    // load map
    Map map;
    ReadColMapDataBinary(map_dir, map);
    const Camera &cam_seq = map.cameras_[0];
    for (auto &[id, frame] : map.frame_map_) {
        if (!frame.registered)
            continue;
        frame.points_normalized.resize(frame.points.size());
        for (size_t j = 0; j < frame.points.size(); j++) {
            ImageToNormalized(cam_seq, frame.points[j],
                              frame.points_normalized[j]);
        }
    }
    std::map<std::string, int> name2id;
    for (auto &[id, frame] : map.frame_map_) {
        name2id[frame.name] = id;
    }

    // get tag info
    auto tag_info_vec = tag_extract(image_dir);
    std::map<int, std::map<int, std::vector<vector2>>> tag_obs,
        tag_obs_normalized;
    for (auto &[name, tag_info] : tag_info_vec) {
        const int frame_id = name2id[name];
        for (auto &[tag_id, pts] : tag_info) {
            tag_obs[tag_id][frame_id] = pts;
        }
    }
    for (auto &[tag_id, frame_obs] : tag_obs) {
        // too few measurement to triangulte
        if (frame_obs.size() < 4)
            continue;
        std::cout << "tag id:" << tag_id
                  << " observer number: " << frame_obs.size() << std::endl;
        for (auto &[frame_id, pts] : frame_obs) {
            std::vector<vector2> pts_normlized(4);
            for (int i = 0; i < 4; ++i) {
                ImageToNormalized(cam_seq, pts[i], pts_normlized[i]);
            }
            tag_obs_normalized[tag_id][frame_id] = pts_normlized;
        }
    }

    // compute tag points
    const int num_tag = tag_obs.size();
    std::map<int, std::vector<vector3>> pt_world_vec;
    for (auto &[tag_id, frame_obs] : tag_obs_normalized) {
        std::vector<vector3> pt_world(4);
        std::vector<std::vector<std::pair<Pose, vector2>>> obs_vec(4);
        for (auto &[frame_id, pts] : frame_obs) {
            auto &Tcw = map.frame_map_[frame_id].Tcw;
            for (int j = 0; j < 4; ++j) {
                obs_vec[j].emplace_back(Tcw, pts[j]);
            }
        }
        for (int j = 0; j < 4; ++j) {
            CreatePoint3dRAW(obs_vec[j], pt_world[j]);
        }
        pt_world_vec[tag_id] = pt_world;
    }

    double scale = 1.0;
    std::map<int, Pose> tag_vec;
    std::vector<vector3> pt_tag = get_tag(tag_length);
    ceres::Problem problem;
    // compute scale and tag_pose
    for (auto &[tag_id, frame_obs] : tag_obs_normalized) {
        auto &pt_world = pt_world_vec[tag_id];

        for (auto &[frame_id, pts_n] : frame_obs) {
            auto &Tcw = map.frame_map_[frame_id].Tcw;
            for (int i = 0; i < 4; ++i) {
                ceres::CostFunction *cost_function =
                    new ProjectionCost(pts_n[i]);
                problem.AddResidualBlock(cost_function, nullptr,
                                         Tcw.q.coeffs().data(), Tcw.t.data(),
                                         pt_world[i].data());
            }
            problem.SetParameterBlockConstant(Tcw.q.coeffs().data());
            problem.SetParameterBlockConstant(Tcw.t.data());
        }

        auto &T_w_tag = tag_vec[tag_id];
        for (int i = 0; i < 4; ++i) {
            ceres::CostFunction *cost_function = new TagCost(pt_tag[i], 1.0);
            problem.AddResidualBlock(
                cost_function, nullptr, T_w_tag.q.coeffs().data(),
                T_w_tag.t.data(), &scale, pt_world[i].data());
            problem.SetParameterBlockConstant(pt_world[i].data());
        }
        problem.SetParameterization(T_w_tag.q.coeffs().data(), new QuatParam);
    }
    problem.SetParameterLowerBound(&scale, 0, 0.2);

    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    // refine map
    for (auto &[id, frame] : map.frame_map_) {
        if (!frame.registered)
            continue;
        for (int i = 0; i < frame.track_ids_.size(); ++i) {
            if (frame.track_ids_[i] == -1)
                continue;
            Track &track = map.track_map_[frame.track_ids_[i]];
            if (track.outlier)
                continue;
            ceres::CostFunction *cost_function =
                new ProjectionCost(frame.points_normalized[i]);
            problem.AddResidualBlock(cost_function, nullptr,
                                     frame.Tcw.q.coeffs().data(),
                                     frame.Tcw.t.data(), track.point3d_.data());
        }
        problem.SetParameterization(frame.Tcw.q.coeffs().data(), new QuatParam);
        problem.SetParameterBlockConstant(frame.Tcw.q.coeffs().data());
        problem.SetParameterBlockConstant(frame.Tcw.t.data());
    }

    for (auto &[tag_id, frame_obs] : tag_obs_normalized) {
        auto &pt_world = pt_world_vec[tag_id];
        for (int i = 0; i < 4; ++i) {
            problem.SetParameterBlockVariable(pt_world[i].data());
        }
    }

    ceres::Solve(solver_options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << scale << std::endl;

    // resize map
    for (auto &[id, frame] : map.frame_map_) {
        auto &Tcw = frame.Tcw;
        Tcw.t /= scale;
    }
    for (auto &[id, track] : map.track_map_) {
        track.point3d_ /= scale;
    }
    WriteColMapDataBinary2(output_path, map);
}
} // namespace xrsfm
