
#include <fstream>

#include "base/map.h"
#include "base/camera.h"
#include "geometry/track_processor.h"
#include "optimization/ba_solver.h"
#include "optimization/cost_factor_ceres.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"
#include "utility/viewer.h"

using namespace xrsfm;

void logRPE(const Map &map) {
    int count = 0;
    double sre = 0;
    for (const auto &[id, frame] : map.frame_map_) {
        if (!frame.registered) continue;
        for (size_t i = 0; i < frame.track_ids_.size(); ++i) {
            const auto &track_id = frame.track_ids_[i];
            if (track_id == -1) continue;
            Track &track = map.track_map_[track_id];
            if (track.outlier) continue;
            const Eigen::Vector3d p_c = frame.Tcw.q * track.point3d_ + frame.Tcw.t;
            const Eigen::Vector2d res = p_c.hnormalized() - frame.points_normalized[i];
            // std::cout << frame.points_normalized[i] << std::endl;
            const double focal = map.cameras_.at(0).fx();

            count++;
            sre += res.norm() * focal;
        }
    }
    printf("avg rpe %lf %d\n", sre / count, count);
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

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    constexpr double tag_length = 0.113;
    // std::string dir = "/data/Rokid/20220726173432/results/";
    // std::string tag_info_path = dir + "tag_info.txt";
    // std::string output_path = "/data/Rokid/20220726173432/refine/";
    std::string dir = argv[1];
    std::string tag_info_path = argv[2];
    std::string output_path = argv[3];

    Map map;
    std::map<std::string, int> name2id;
    ReadColMapDataBinary(dir, map);
    map.cameras_[0].log();
    for (auto &[id, frame] : map.frame_map_) {
        if (!frame.registered) continue;
        frame.points_normalized.resize(frame.points.size());
        for (size_t j = 0; j < frame.points.size(); j++) {
            ImageToNormalized(map.cameras_[0], frame.points[j], frame.points_normalized[j]);
        }
    }
    for (auto &[id, frame] : map.frame_map_) {
        name2id[frame.name] = id;
    }

    // load tag info
    std::ifstream file(tag_info_path);
    std::map<int, std::map<int, std::vector<Eigen::Vector2d>>> tag_obs;
    std::map<int, std::map<int, std::vector<Eigen::Vector2d>>> tag_obs_normalized;
    std::string line;
    while (std::getline(file, line)) {
        // std::cout << line << std::endl;
        std::stringstream ss(line);
        std::string name;
        int num_tag = -1;
        ss >> name >> num_tag;

        if (name2id.count(name) == 0) continue;
        const int frame_id = name2id.at(name);

        int tag_id = -1;
        std::vector<Eigen::Vector2d> pts(4);
        for (int k = 0; k < num_tag; ++k) {
            file >> tag_id;
            for (int j = 0; j < 4; ++j) {
                file >> pts[j].x() >> pts[j].y();
            }
            tag_obs[tag_id][frame_id] = pts;
        }
    }
    for (auto &[tag_id, frame_obs] : tag_obs) {
        std::cout << tag_id << " " << frame_obs.size() << std::endl;
        if (frame_obs.size() < 4) continue;
        for (auto &[frame_id, pts] : frame_obs) {
            std::vector<Eigen::Vector2d> pts_normlized(4);
            for (int j = 0; j < 4; ++j) {
                ImageToNormalized(map.cameras_[0], pts[j], pts_normlized[j]);
            }
            tag_obs_normalized[tag_id][frame_id] = pts_normlized;
        }
    }

    // compute tag points
    const int num_tag = tag_obs.size();
    std::cout << "num tag type:" << tag_obs.size() << std::endl;
    std::map<int, std::vector<Eigen::Vector3d>> pt_world_vec;
    for (auto &[tag_id, frame_obs] : tag_obs_normalized) {
        std::vector<Eigen::Vector3d> pt_world(4);
        std::vector<std::vector<std::pair<Pose, Eigen::Vector2d>>> obs_vec(4);
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
                ceres::CostFunction *cost_function = new ProjectionCost(pts_n[i]);
                problem.AddResidualBlock(cost_function, nullptr, Tcw.q.coeffs().data(), Tcw.t.data(), pt_world[i].data());
            }
            problem.SetParameterBlockConstant(Tcw.q.coeffs().data());
            problem.SetParameterBlockConstant(Tcw.t.data());
        }

        auto &T_w_tag = tag_vec[tag_id];
        for (int i = 0; i < 4; ++i) {
            // pt_world = scale * T_w_tag * pt_tag;
            ceres::CostFunction *cost_function = new TagCost(pt_tag[i], 1.0);
            problem.AddResidualBlock(cost_function, nullptr, T_w_tag.q.coeffs().data(), T_w_tag.t.data(), &scale,
                                     pt_world[i].data());
            problem.SetParameterBlockConstant(pt_world[i].data());
        }
        problem.SetParameterization(T_w_tag.q.coeffs().data(), new QuatParam);
    }

    ceres::Solver::Options solver_options;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    // refine map
    for (auto &[id, frame] : map.frame_map_) {
        if (!frame.registered) continue;
        for (int i = 0; i < frame.track_ids_.size(); ++i) {
            if (frame.track_ids_[i] == -1) continue;
            Track &track = map.track_map_[frame.track_ids_[i]];
            if (track.outlier) continue;
            ceres::CostFunction *cost_function = new ProjectionCost(frame.points_normalized[i]);
            problem.AddResidualBlock(cost_function, nullptr, frame.Tcw.q.coeffs().data(), frame.Tcw.t.data(),
                                     track.point3d_.data());
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

    ViewerThread viewer;
    viewer.start();
    viewer.update_map_colmap(map);
    viewer.add_tag_points(pt_world_vec);

    sleep(1000);
    viewer.stop();
    return 0;
}
