//
// Created by yzc on 19-4-2.
//

#include "pnp.h"

#include "colmap/estimators/absolute_pose.h"
#include "colmap/optim/loransac.h"
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

    const double max_error =
        max_error_pixel / map.cameras_[frame.camera_id].fx();
    std::vector<char> inlier_mask;
    if (SolvePnP_colmap(points2ds, points3ds, max_error, frame.Tcw,
                        inlier_mask)) {
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
    LOG(WARNING) << "fail to register frame " << frame_id;
    return false;
}

bool RegisterNextImage(const int _frame_id, Map &map, Pose &tcw,
                       std::vector<std::pair<int, int>> &cor_2d_3d_ids) {
    Frame &next_frame = map.frames_[_frame_id];
    // search for 2D-3D correspondences
    std::vector<vector2> cor_points2ds;
    std::vector<vector3> cor_points3ds;
    map.SearchCorrespondences(next_frame, cor_points2ds, cor_points3ds,
                              cor_2d_3d_ids, true);
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

bool ComputeRegisterInlierLoop(const int _frame_id, LoopInfo &loop_info,
                               Map &map) {
    printf("ComputeRegisterInlierLoop next frame id: %d \n", _frame_id);
    Frame &next_frame = map.frames_[_frame_id];
    for (size_t i = 0; i < loop_info.cor_frame_ids_vec.size(); ++i) {
        const auto &cor_set = loop_info.cor_frame_ids_vec[i];
        // search for 2D-3D correspondences
        std::vector<vector2> cor_points2ds;
        std::vector<vector3> cor_points3ds;
        std::vector<std::pair<int, int>> cor_2d_3d_ids;
        map.SearchCorrespondences1(next_frame, cor_set, cor_points2ds,
                                   cor_points3ds, cor_2d_3d_ids, true);

        Frame tmp;
        std::vector<char> inlier_mask;
        const double max_error = 8.0 / map.cameras_[next_frame.camera_id].fx();
        if (SolvePnP_colmap(cor_points2ds, cor_points3ds, max_error, tmp.Tcw,
                            inlier_mask)) {
            tmp.registered = true;
            // Continue tracks
            int count = 0;
            for (int id = 0; id < inlier_mask.size(); ++id) {
                if (!inlier_mask[id])
                    continue;
                count++;
            }
            loop_info.twc_vec.push_back(tmp.Tcw.inverse());
            loop_info.num_inlier_vec.push_back(count);
            printf("Inlier %d/%zu\n", count, cor_points2ds.size());
        } else {
            loop_info.num_inlier_vec.push_back(-1);
            printf("Inlier %d/%zu\n", -1, cor_points2ds.size());
        }
    }
    return true;
}

bool RegisterNextImageLoop(const int _frame_id, LoopInfo &loop_info, Map &map) {
    Frame &next_frame = map.frames_[_frame_id];

    std::vector<vector<3>> gt_positions(0), in_positions(0);
    std::vector<std::pair<double, double>> depth_obs_vec;
    int max_inlier = 0;
    Pose best_pose;
    Frame tmp, tmp1;
    for (size_t i = 0; i < loop_info.cor_frame_ids_vec.size(); ++i) {
        const auto &cor_set = loop_info.cor_frame_ids_vec[i];
        // search for 2D-3D correspondences
        std::vector<vector2> cor_points2ds;
        std::vector<vector3> cor_points3ds;
        std::vector<std::pair<int, int>> cor_2d_3d_ids;
        map.SearchCorrespondences1(next_frame, cor_set, cor_points2ds,
                                   cor_points3ds, cor_2d_3d_ids, true);
        printf("next frame id: %d cor number: %d\n", _frame_id,
               (int)cor_2d_3d_ids.size());
        const double max_error = 8.0 / map.cameras_[next_frame.camera_id].fx();
        std::vector<char> inlier_mask;
        if (SolvePnP_colmap(cor_points2ds, cor_points3ds, max_error, tmp.Tcw,
                            inlier_mask)) {
            tmp.registered = true;
            // Continue tracks
            int count = 0;
            for (int id = 0; id < inlier_mask.size(); ++id) {
                if (!inlier_mask[id])
                    continue;
                count++;
                const auto &[p2d_id, track_id] = cor_2d_3d_ids[id];
                if (next_frame.track_ids_[p2d_id] != -1) {
                    const int old_track_id = next_frame.track_ids_[p2d_id];
                    auto &track1 = map.tracks_[old_track_id];
                    auto &track2 = map.tracks_[track_id];
                    vector3 p3d1 = tmp1.Tcw.q * track1.point3d_ + tmp1.Tcw.t;
                    vector3 p3d2 = tmp.Tcw.q * track2.point3d_ + tmp.Tcw.t;
                    depth_obs_vec.emplace_back(p3d1.z(), p3d2.z());
                    // printf("%lf %lf %lf\n", p3d1.z(), p3d2.z(), p3d1.z() /
                    // p3d2.z());
                    gt_positions.emplace_back(track1.point3d_);
                    in_positions.emplace_back(track2.point3d_);
                }

                next_frame.track_ids_[p2d_id] = track_id;
                map.tracks_[track_id].observations_[_frame_id] = p2d_id;
            }
            loop_info.twc_vec.push_back(tmp.Tcw.inverse());
            loop_info.num_inlier_vec.push_back(count);
            if (count > max_inlier) {
                max_inlier = count;
                best_pose = tmp.Tcw;
            }
            tmp1 = tmp;
            printf("PnP %d/%zu\n", count, cor_points2ds.size());
        }
    }
    next_frame.Tcw = best_pose;
    next_frame.registered = true;
    next_frame.is_keyframe = true;
    // TODO use ransace
    if (depth_obs_vec.size() >= 4) {
        double sum = 0;
        for (int i = 0; i < depth_obs_vec.size(); ++i) {
            sum += depth_obs_vec[0].first / depth_obs_vec[0].second;
        }
        loop_info.scale_obs = sum / depth_obs_vec.size();
        SRT srt = umeyama(gt_positions, in_positions, false);
        loop_info.scale_obs = srt.scale;
        printf("s12: %lf %lf\n", 1.0 / (sum / depth_obs_vec.size()), srt.scale);
        std::cout << srt.t.transpose() << std::endl;
        for (size_t i = 0; i < gt_positions.size(); ++i) {
            vector3 new_p =
                srt.q.inverse() * (srt.scale * gt_positions[i] - srt.t);
            // std::cout << gt_positions[i].transpose() << " - " <<
            // in_positions[i].transpose() << " - "
            //           << new_p.transpose() << std::endl;
        }
    } else {
        loop_info.scale_obs = -1;
    }
    return true;
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

int RegisterNextImage1(const int frame_id, Map &map) {
    Frame &next_frame = map.frames_[frame_id];
    // search for 2D-3D correspondences
    std::vector<vector2> cor_points2ds;
    std::vector<vector3> cor_points3ds;
    std::vector<std::pair<int, int>> cor_2d_3d_ids;
    map.SearchCorrespondences(next_frame, cor_points2ds, cor_points3ds,
                              cor_2d_3d_ids, true);
    printf("next frame id: %d cor number: %d\n", frame_id,
           (int)cor_2d_3d_ids.size());
    if (cor_2d_3d_ids.size() < 20)
        return 0;
    std::vector<char> inlier_mask;
    if (SolvePnP_colmap(cor_points2ds, cor_points3ds, 8.0 / 700, next_frame.Tcw,
                        inlier_mask)) {
        next_frame.registered = true;
        next_frame.is_keyframe = true;
        // Continue tracks
        int count = 0;
        for (int id = 0; id < inlier_mask.size(); ++id) {
            if (!inlier_mask[id])
                continue;
            count++;
            const auto &[p2d_id, track_id] = cor_2d_3d_ids[id];
            next_frame.track_ids_[p2d_id] = track_id;
            map.tracks_[track_id].observations_[frame_id] = p2d_id;
        }
        printf("PnP %d/%zu\n", count, cor_points2ds.size());
        return count;
    }
    LOG(WARNING) << "fail to register frame " << frame_id;
    return 0;
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
