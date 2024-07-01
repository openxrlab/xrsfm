//
// Created by yzc on 19-4-2.
//

#include "pnp.h"

#include "estimators/absolute_pose.h"
#include "ransac/loransac.h"
#include "optimization/cost_factor_ceres.h"
#include "geometry/projection.hpp"

namespace xrsfm {

bool PoseEstimateInitColmap(const std::vector<vector2> &cor_points2ds,
                            const std::vector<vector3> &cor_points3ds,
                            const double max_error, Pose &tcw,
                            std::vector<char> &inlier_mask) {
    using namespace colmap;
    RANSACOptions ransac_options;
    ransac_options.max_error = max_error;
    ransac_options.min_inlier_ratio = 0.25;
    ransac_options.min_num_trials = 100;
    ransac_options.confidence = 0.9999;
    LORANSAC<P3PEstimator, EPNPEstimator> ransac(ransac_options);
    auto report = ransac.Estimate(cor_points2ds, cor_points3ds);
    if (!report.success || report.support.num_inliers == 0) {
        return false;
    }

    inlier_mask = report.inlier_mask;
    Eigen::Matrix3x4d proj_matrix = report.model;
    quaternion quat(proj_matrix.leftCols<3>());
    tcw.q = quat;
    tcw.t = proj_matrix.rightCols<1>();
    return true;
}

bool RegisterImage(const int frame_id, Map &map) {
    constexpr int min_num_correspondence = 20;
    constexpr double max_error_pixel = 8.0;

    Frame &frame = map.frame(frame_id);
    Camera &camera = map.Camera(frame.camera_id);

    std::vector<vector2> points2ds;
    std::vector<vector3> points3ds;
    std::vector<std::pair<int, int>> id_pair_vec;
    map.SearchCorrespondences(frame, points2ds, points3ds, id_pair_vec, true);
    if (id_pair_vec.size() < min_num_correspondence)
        return false;

    // pose estimate [init]
    const double max_error = max_error_pixel / camera.fx();
    std::vector<char> inlier_mask;
    if (!PoseEstimateInitColmap(points2ds, points3ds, max_error, frame.Tcw,
                                inlier_mask)) {
        LOG(WARNING) << "fail to register frame " << frame_id;
        return false;
    }

    // pose estimate [refine]
    {
        ceres::Problem problem;
        double *camera_param = camera.params_.data();
        const int camera_model_id = camera.model_id_;
        for (int id = 0; id < inlier_mask.size(); ++id) {
            if (!inlier_mask[id])
                continue;
            const auto &[p2d_id, track_id] = id_pair_vec[id];
            ceres::CostFunction *cost_function =
                ReProjectionCostCreate(camera_model_id, frame.points[p2d_id]);
            ceres::LossFunction *loss_function = new ceres::HuberLoss(5.99);
            problem.AddResidualBlock(
                cost_function, loss_function, frame.Tcw.q.coeffs().data(),
                frame.Tcw.t.data(), points3ds[id].data(), camera_param);
            problem.SetParameterBlockConstant(points3ds[id].data());
        }
        problem.SetParameterization(frame.Tcw.q.coeffs().data(),
                                    new ceres::EigenQuaternionParameterization);
        problem.SetParameterBlockConstant(camera_param);

        ceres::Solver::Options solver_options;
        solver_options.max_num_iterations = 10;
        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);
        std::cout << "Initial cost : " << std::setprecision(6)
                  << std::sqrt(summary.initial_cost /
                               summary.num_residuals_reduced)
                  << " [px]" << std::endl;
        std::cout << "Final cost : " << std::setprecision(6)
                  << std::sqrt(summary.final_cost /
                               summary.num_residuals_reduced)
                  << " [px]" << std::endl;
    }

    frame.registered = frame.is_keyframe = true;
    // Continue tracks
    std::vector<int> level_vec;
    int num_inlier = 0;
    for (int id = 0; id < inlier_mask.size(); ++id) {
        if (!inlier_mask[id])
            continue;
        num_inlier++;
        const auto &[p2d_id, track_id] = id_pair_vec[id];
        auto &track = map.track(track_id);
        if (track.observations_.count(frame_id) == 0) {
            frame.track_ids_[p2d_id] = track_id;
            track.observations_[frame_id] = p2d_id;
            map.AddNumCorHavePoint3D(frame_id, p2d_id);
            level_vec.emplace_back(track.hierarchical_level);
        } else { // if this track has been observed
            // TODO compare the reprojection error
        }
    }
    std::sort(level_vec.begin(), level_vec.end());
    frame.hierarchical_level = (level_vec.size() >= MIN_OBS_NUM_LEVEL
                                    ? level_vec[MIN_OBS_NUM_LEVEL - 1]
                                    : level_vec.back()) +
                               1;

    printf("PnP %d/%zu\n", num_inlier, points2ds.size());
    return true;
}

bool RegisterImageLocal(const int _frame_id, const std::set<int> cor_set,
                        Map &map) {
    Frame &next_frame = map.frame(_frame_id);
    // search for 2D-3D correspondences
    std::vector<vector2> cor_points2ds;
    std::vector<vector3> cor_points3ds;
    std::vector<std::pair<int, int>> cor_2d_3d_ids;
    map.SearchCorrespondencesLocal(next_frame, cor_set, cor_points2ds,
                                   cor_points3ds, cor_2d_3d_ids, true);
    printf("next frame id: %d cor number: %d\n", _frame_id,
           (int)cor_2d_3d_ids.size());
    if (cor_2d_3d_ids.size() < 20)
        return false;

    map.tmp_frame = next_frame;
    map.tmp_frame.track_ids_.assign(map.tmp_frame.track_ids_.size(), -1);
    const double max_error = 16.0 / map.Camera(next_frame.camera_id).fx();
    std::vector<char> inlier_mask;
    if (PoseEstimateInitColmap(cor_points2ds, cor_points3ds, max_error,
                               map.tmp_frame.Tcw, inlier_mask)) {
        map.tmp_frame.registered = true;

        int count = 0;
        for (int id = 0; id < inlier_mask.size(); ++id) {
            if (!inlier_mask[id])
                continue;
            count++;
            const auto &[p2d_id, track_id] = cor_2d_3d_ids[id];
            map.tmp_frame.track_ids_[p2d_id] = track_id;
        }
        printf("PnP %d/%zu\n", count, cor_points2ds.size());
        return true;
    }
    LOG(WARNING) << "fail to register frame " << _frame_id;

    return false;
}

void GetPoseEstimateData(const Map &map, const int image_id,
                         std::vector<std::pair<int, int>> &correspondences) {
    // TODO it is not one-to-one correspondences
    correspondences.clear();
    for (int i = 0; i < map.frame(image_id).track_ids_.size(); ++i) {
        const auto &corrs =
            map.corr_graph_.frame_node_vec_.at(image_id).corrs_vector.at(i);
        for (const auto &[cor_image_id, p2d_id] : corrs) {
            const int mappoint_id =
                map.frame(cor_image_id).track_ids_.at(p2d_id);
            if (mappoint_id == -1)
                continue;
            correspondences.emplace_back(i, mappoint_id);
        }
    }
}

std::vector<char>
get_local_mask(const Map &map, const int image_id,
               const std::vector<std::pair<int, int>> &correspondences,
               const std::set<int> &image_id_set) {
    std::vector<char> local_mask(correspondences.size(), true);
    if (image_id_set.empty())
        return local_mask;

    for (int i = 0; i < correspondences.size(); ++i) {
        const auto [p2d_id, p3d_id] = correspondences.at(i);
        const auto &corrs =
            map.corr_graph_.frame_node_vec_.at(image_id).corrs_vector.at(
                p2d_id);

        local_mask.at(i) = false;
        for (const auto &[cor_image_id, cor_p2d_id] : corrs) {
            if (image_id_set.count(cor_image_id) == 0)
                continue;
            local_mask.at(i) = true;
            break;
        }
    }
    return local_mask;
}

int count_inlier(const std::vector<char> &mask) {
    int count = 0;
    for (int k = 0; k < mask.size(); ++k) {
        if (mask[k])
            count++;
    }
    return count;
}

RegisterResult RegisterImageLocal1(const int frame_id,
                                   const std::set<int> references, Map &map) {
    RegisterResult reg_result;
    reg_result.frame_id = frame_id;
    Frame &frame = map.frame(frame_id);
    Camera &camera = map.Camera(frame.camera_id);

    std::vector<std::pair<int, int>> correspondences;
    GetPoseEstimateData(map, frame_id, correspondences);
    std::vector<char> local_mask =
        get_local_mask(map, frame_id, correspondences, references);
    printf("[PnP] image_id %u init/all %d/%d\n", frame_id,
           count_inlier(local_mask), (int)correspondences.size());

    std::vector<vector2> pts2d, pts2d_init;
    std::vector<vector3> pts3d, pts3d_init;
    for (int i = 0; i < correspondences.size(); ++i) {
        const auto &[p2d_id, p3d_id] = correspondences.at(i);
        const vector2 &p2d = frame.points.at(p2d_id);
        const vector3 &p3d = map.track(p3d_id).point3d_;
        pts2d.push_back(p2d);
        pts3d.push_back(p3d);
        if (local_mask.at(i)) {
            Eigen::Vector2d pt2d_N;
            ImageToNormalized(camera, p2d, pt2d_N);
            pts2d_init.push_back(pt2d_N);
            pts3d_init.push_back(p3d);
        }
    }

    Pose &pose = reg_result.tcw;
    std::vector<char> inlier_mask;
    const double max_error = 12.0 / camera.fx();
    if (!PoseEstimateInitColmap(pts2d_init, pts3d_init, max_error, pose,
                                inlier_mask))
        return reg_result;

    inlier_mask.assign(pts2d.size(), false);
    for (size_t i = 0; i < pts2d.size(); ++i) {
        if (ReprojectionError(pose, camera, pts2d.at(i), pts3d.at(i)) > 12.0)
            continue;
        inlier_mask.at(i) = true;
    }

    // pose estimate [refine]
    {
        ceres::Problem problem;
        double *camera_param = camera.params_.data();
        for (int i = 0; i < inlier_mask.size(); ++i) {
            if (!inlier_mask[i])
                continue;
            ceres::CostFunction *cost_function =
                ReProjectionCostCreate(camera.model_id_, pts2d.at(i));
            ceres::LossFunction *loss_function = new ceres::HuberLoss(5.99);
            problem.AddResidualBlock(cost_function, loss_function,
                                     pose.q.coeffs().data(), pose.t.data(),
                                     pts3d.at(i).data(), camera_param);
            problem.SetParameterBlockConstant(pts3d.at(i).data());
        }
        problem.SetParameterization(pose.q.coeffs().data(),
                                    new ceres::EigenQuaternionParameterization);
        problem.SetParameterBlockConstant(camera_param);

        ceres::Solver::Options solver_options;
        solver_options.max_num_iterations = 10;
        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);
        std::cout << "Cost : " << std::setprecision(6)
                  << std::sqrt(summary.initial_cost /
                               summary.num_residuals_reduced)
                  << " [px] => " << std::setprecision(6)
                  << std::sqrt(summary.final_cost /
                               summary.num_residuals_reduced)
                  << " [px]" << std::endl;
    }

    int num_inlier = 0;
    reg_result.correspondences_2d3d.clear();
    for (size_t i = 0; i < pts2d.size(); ++i) {
        if (ReprojectionError(pose, camera, pts2d.at(i), pts3d.at(i)) > 12.0)
            continue;

        reg_result.correspondences_2d3d.push_back(correspondences.at(i));
        num_inlier++;
    }

    reg_result.success = true;
    reg_result.num_inlier = num_inlier;
    reg_result.num_correspondences = correspondences.size();
    return reg_result;
}

} // namespace xrsfm
