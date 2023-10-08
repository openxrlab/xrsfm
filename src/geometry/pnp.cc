//
// Created by yzc on 19-4-2.
//

#include "pnp.h"

#include "colmap/estimators/absolute_pose.h"
#include "colmap/optim/loransac.h"
#include "optimization/ba_solver.h"
#include "optimization/cost_factor_ceres.h"
#include "umeyama.h"

namespace xrsfm {

bool RegisterImage(const int frame_id, Map &map) {
    constexpr int min_num_correspondence = 20;
    constexpr double max_error_pixel = 8.0;

    Frame &frame = map.frames_[frame_id];

    std::vector<vector2> points2ds;
    std::vector<vector3> points3ds;
    std::vector<std::pair<int, int>> id_pair_vec;
    map.SearchCorrespondences(frame, points2ds, points3ds, id_pair_vec, true);
    if (id_pair_vec.size() < min_num_correspondence)
        return false;

    // pose estimate [init]
    const double max_error =
        max_error_pixel / map.cameras_[frame.camera_id].fx();
    std::vector<char> inlier_mask;
    if (!SolvePnP_colmap(points2ds, points3ds, max_error, frame.Tcw,
                         inlier_mask)) {
        LOG(WARNING) << "fail to register frame " << frame_id;
        return false;
    }

    // pose estimate [refine]
    {
        ceres::Problem problem;
        double *camera_param = map.cameras_[frame.camera_id].params_.data();
        const int camera_model_id = map.cameras_[frame.camera_id].model_id_;
        for (int id = 0; id < inlier_mask.size(); ++id) {
            if (!inlier_mask[id])
                continue;
            const auto &[p2d_id, track_id] = id_pair_vec[id];
            ceres::CostFunction *cost_function =
                ReProjectionCostCreate(camera_model_id, frame.points[p2d_id]);
            problem.AddResidualBlock(
                cost_function, nullptr, frame.Tcw.q.coeffs().data(),
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
        auto &track = map.tracks_[track_id];
        // TODO do something if this track has been observed by this frame
        if (track.observations_.count(frame_id) == 0) {
            frame.track_ids_[p2d_id] = track_id;
            track.observations_[frame_id] = p2d_id;
            map.AddNumCorHavePoint3D(frame_id, p2d_id);
            level_vec.emplace_back(track.hierarchical_level);
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

bool RegisterNextImageLocal(const int _frame_id, const std::set<int> cor_set,
                            Map &map, Pose &tcw,
                            std::vector<std::pair<int, int>> &cor_2d_3d_ids) {
    Frame &next_frame = map.frames_[_frame_id];
    // search for 2D-3D correspondences
    std::vector<vector2> cor_points2ds;
    std::vector<vector3> cor_points3ds;
    map.SearchCorrespondences1(next_frame, cor_set, cor_points2ds,
                               cor_points3ds, cor_2d_3d_ids, true);
    printf("frame: %d  cor num: %zu\n", _frame_id, cor_2d_3d_ids.size());
    if (cor_2d_3d_ids.size() < 20)
        return false;
    const double max_error = 16.0 / map.cameras_[next_frame.camera_id].fx();
    std::vector<char> inlier_mask;
    if (SolvePnP_colmap(cor_points2ds, cor_points3ds, max_error, tcw,
                        inlier_mask)) {
        // Continue tracks
        std::vector<std::pair<int, int>> inlier_cor_2d_3d_ids;
        for (int id = 0; id < inlier_mask.size(); ++id) {
            if (!inlier_mask[id])
                continue;
            inlier_cor_2d_3d_ids.emplace_back(cor_2d_3d_ids[id]);
        }
        printf("PnP %zu/%zu\n", inlier_cor_2d_3d_ids.size(),
               cor_2d_3d_ids.size());
        cor_2d_3d_ids = inlier_cor_2d_3d_ids;
        return true;
    }
    LOG(WARNING) << "fail to register frame " << _frame_id;
    return false;
}

bool RegisterNextImageLocal(const int _frame_id, const std::set<int> cor_set,
                            Map &map) {
    Frame &next_frame = map.frames_[_frame_id];
    // search for 2D-3D correspondences
    std::vector<vector2> cor_points2ds;
    std::vector<vector3> cor_points3ds;
    std::vector<std::pair<int, int>> cor_2d_3d_ids;
    map.SearchCorrespondences1(next_frame, cor_set, cor_points2ds,
                               cor_points3ds, cor_2d_3d_ids, true);
    printf("next frame id: %d cor number: %d\n", _frame_id,
           (int)cor_2d_3d_ids.size());
    if (cor_2d_3d_ids.size() < 20)
        return false;

    map.tmp_frame = next_frame;
    map.tmp_frame.track_ids_.assign(map.tmp_frame.track_ids_.size(), -1);
    const double max_error = 16.0 / map.cameras_[next_frame.camera_id].fx();
    std::vector<char> inlier_mask;
    if (SolvePnP_colmap(cor_points2ds, cor_points3ds, max_error,
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

bool SolvePnP(Camera cam, std::vector<vector2> cor_points2ds,
              std::vector<vector3> cor_points3ds, Frame &frame,
              std::vector<int> &inlier) {
    size_t cor_num = cor_points2ds.size();
    if (cor_num < 20) {
        printf("there are no enough correspondence\n");
        return false;
    }
    std::vector<cv::Point3d> cvpt3d(cor_num);
    std::vector<cv::Point2d> cvpt2d(cor_num);
    for (int i = 0; i < cor_num; ++i) {
        cvpt2d[i] = cv::Point2d(cor_points2ds[i](0), cor_points2ds[i](1));
        cvpt3d[i] = cv::Point3d(cor_points3ds[i](0), cor_points3ds[i](1),
                                cor_points3ds[i](2));
    }
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0] = cam.fx();
    camera_matrix.ptr<double>(0)[2] = cam.cx();
    camera_matrix.ptr<double>(1)[1] = cam.fy();
    camera_matrix.ptr<double>(1)[2] = cam.cy();
    camera_matrix.ptr<double>(2)[2] = 1.0f;

    cv::Mat distortion_coeffs = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
    cv::Mat rvec, tvec, rmat;

    // there are randomness in cv::solvePnPRansac
    srand(0);
    bool success =
        solvePnPRansac(cvpt3d, cvpt2d, camera_matrix, distortion_coeffs, rvec,
                       tvec, false, 50, 3.5, 0.99, inlier, cv::SOLVEPNP_EPNP);
    if (success) {
        Rodrigues(rvec, rmat);
        matrix3 rmatrix;
        rmatrix << rmat.ptr<double>(0)[0], rmat.ptr<double>(0)[1],
            rmat.ptr<double>(0)[2], rmat.ptr<double>(1)[0],
            rmat.ptr<double>(1)[1], rmat.ptr<double>(1)[2],
            rmat.ptr<double>(2)[0], rmat.ptr<double>(2)[1],
            rmat.ptr<double>(2)[2];
        vector3 t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        frame.Tcw = Pose(quaternion(rmatrix), t);
        frame.registered = true;
        printf("PnP %d/%d\n", inlier.size(), cvpt2d.size());
    }
    return success;
}

using namespace colmap;

struct AbsolutePoseEstimationOptions {
    // Whether to estimate the focal length.
    bool estimate_focal_length = false;

    // Number of discrete samples for focal length estimation.
    size_t num_focal_length_samples = 30;

    // Minimum focal length ratio for discrete focal length sampling
    // around focal length of given camera.
    double min_focal_length_ratio = 0.2;

    // Maximum focal length ratio for discrete focal length sampling
    // around focal length of given camera.
    double max_focal_length_ratio = 5;

    // Number of threads for parallel estimation of focal length.
    int num_threads = 1;

    // Options used for P3P RANSAC.
    RANSACOptions ransac_options;

    void Check() const {
        CHECK_GT(num_focal_length_samples, 0);
        CHECK_GT(min_focal_length_ratio, 0);
        CHECK_GT(max_focal_length_ratio, 0);
        CHECK_LT(min_focal_length_ratio, max_focal_length_ratio);
        ransac_options.Check();
    }
};

bool EstimateAbsolutePose(const AbsolutePoseEstimationOptions &options,
                          const std::vector<vector2> &points2D,
                          const std::vector<vector3> &points3D, Pose &pose,
                          std::vector<char> *inlier_mask);

bool SolvePnP_colmap(const std::vector<vector2> &cor_points2ds,
                     const std::vector<vector3> &cor_points3ds,
                     const double max_error, Pose &tcw,
                     std::vector<char> &inlier_mask) {
    AbsolutePoseEstimationOptions abs_pose_options;
    abs_pose_options.num_threads = 1;
    abs_pose_options.num_focal_length_samples = 30;
    abs_pose_options.ransac_options.max_error = max_error;
    abs_pose_options.ransac_options.min_inlier_ratio = 0.25;
    // Use high confidence to avoid preemptive termination of P3P RANSAC
    // - too early termination may lead to bad registration.
    abs_pose_options.ransac_options.min_num_trials = 100;
    abs_pose_options.ransac_options.confidence = 0.9999;
    abs_pose_options.estimate_focal_length = false;

    bool success = EstimateAbsolutePose(abs_pose_options, cor_points2ds,
                                        cor_points3ds, tcw, &inlier_mask);

    return success;
}

typedef LORANSAC<P3PEstimator, EPNPEstimator> AbsolutePoseRANSAC;

void EstimateAbsolutePoseKernel(const std::vector<vector2> &points2D,
                                const std::vector<vector3> &points3D,
                                const RANSACOptions &options,
                                AbsolutePoseRANSAC::Report *report) {
    // Estimate pose for given focal length.
    AbsolutePoseRANSAC ransac(options);

    std::vector<vector2> points2D_N(points2D.size());
    for (size_t i = 0; i < points2D.size(); ++i) {
        points2D_N[i] = points2D[i];
    }

    *report = ransac.Estimate(points2D_N, points3D);
}

bool EstimateAbsolutePose(const AbsolutePoseEstimationOptions &options,
                          const std::vector<vector2> &points2D,
                          const std::vector<vector3> &points3D, Pose &tcw,
                          std::vector<char> *inlier_mask) {
    AbsolutePoseRANSAC::Report report;

    EstimateAbsolutePoseKernel(points2D, points3D, options.ransac_options,
                               &report);

    if (!report.success || report.support.num_inliers == 0) {
        return false;
    }

    Eigen::Matrix3x4d proj_matrix;
    proj_matrix = report.model;
    *inlier_mask = report.inlier_mask;
    quaternion quat(proj_matrix.leftCols<3>());

    tcw.q = quat;
    tcw.t = proj_matrix.rightCols<1>();
    //    if (IsNaN(*qvec) || IsNaN(*tvec)) {
    //        return false;
    //    }

    return true;
}
} // namespace xrsfm
