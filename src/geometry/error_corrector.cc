#include "geometry/error_corrector.h"

#include "geometry/pnp.h"
#include "utility/global.h"
#include "utility/io_ecim.hpp"

namespace xrsfm {

bool ErrorCorrector::IsGoodRelativePose(const Map &map, const FramePair &fp,
                                        std::vector<char> &inlier_mask) {
    constexpr int num_min_matches = 100;
    constexpr double ratio_th = 0.8;
    constexpr double pure_rotation_th = 0.01;
    constexpr double sin_th = std::sin(2.0 * M_PI / 180);
    constexpr double cos_th = std::cos(2.0 * M_PI / 180);

    std::cout << fp.id1 << " " << fp.id2 << " " << fp.matches.size()
              << std::endl;
    if ((std::abs(fp.id1 - fp.id2) != 1) && fp.matches.size() < num_min_matches)
        return true;

    const auto &frame1 = map.frame(fp.id1), &frame2 = map.frame(fp.id2);
    const vector3 relative_motion = frame2.center() - frame1.center();
    const double distance = relative_motion.norm();
    const bool is_pure_rotation = distance < pure_rotation_th;
    if (is_pure_rotation)
        printf("pure rotation, distance: %lf\n", distance);
    const vector3 t12 = relative_motion.normalized();

    inlier_mask.clear();
    int num_matches = 0, num_inliers = 0;
    for (int i = 0; i < fp.matches.size(); ++i) {
        if (!fp.inlier_mask[i])
            continue;
        num_matches++;

        bool good_relative_pose = true;
        if (is_pure_rotation) { // error
                                // double cos_theta = ray1.dot(ray2);
                                // good_essential = cos_theta > cos_th;
        } else {
            const vector2 p2d1 =
                map.GetNormalizedPoint(fp.id1, fp.matches[i].id1);
            const vector2 p2d2 =
                map.GetNormalizedPoint(fp.id2, fp.matches[i].id2);
            const vector3 ray1 =
                (frame1.qwc() * p2d1.homogeneous()).normalized();
            const vector3 ray2 =
                (frame2.qwc() * p2d2.homogeneous()).normalized();

            const bool use_ray2 =
                std::abs(ray1.dot(t12)) > std::abs(ray2.dot(t12));
            // compute deg between ray2 and plane_ray1-t
            const vector3 n =
                (ray2.cross(t12)).normalized(); // normal line of plane_ray1-t
            const double sin_theta = std::abs(n.dot(ray1));
            const vector3 n1 = (ray1.cross(t12)).normalized();
            const double sin_theta1 = std::abs(n1.dot(ray2));

            good_relative_pose =
                use_ray2 ? sin_theta < sin_th : sin_theta1 < sin_th;

            if (ray1.dot(ray2) < 0 && ray1.dot(t12) < 0)
                good_relative_pose = false;
            if (ray1.dot(t12) < 0 && ray2.dot(t12) > ray1.dot(t12) + sin_th)
                good_relative_pose = false;
            if (ray2.dot(t12) > 0 && ray2.dot(t12) > ray1.dot(t12) + sin_th)
                good_relative_pose = false;

            if (good_relative_pose) {
                const double deg_r1_r2 = ToDeg(acos(ray1.dot(ray2)));
                if (deg_r1_r2 > 1.0) {
                    const double cos_1 = ray1.dot(t12);
                    const double cos_2 = ray2.dot(t12);
                    // std::cout<<cos_1<<" "<<cos_2<<std::endl;
                    if (abs(cos_1) > 1e-5 && abs(cos_2) > 1e-5) {
                        const double t1 = acos(cos_1);
                        const double t2 = acos(cos_2);
                        if (abs(1.0 / tan(t1) - 1.0 / tan(t2)) < 1e-5)
                            good_relative_pose = false;
                        const double h =
                            distance / abs(1.0 / tan(t1) - 1.0 / tan(t2));
                        if (h > 200 * sin(t1) || h > 200 * sin(t2))
                            good_relative_pose = false;
                    }
                }
            }
        }
        if (good_relative_pose)
            num_inliers++;
        inlier_mask.emplace_back(good_relative_pose);
    }

    const double ratio = 1.0 * num_inliers / num_matches;
    printf("%d %d %lf %d/%d\n", frame1.id, frame2.id, ratio, num_inliers,
           num_matches);

    if (ratio < ratio_th) {
        return false;
    }
    return true;
}

bool ErrorCorrector::CheckAllRelativePose(
    Map &map, int frame_id, std::set<int> &bad_matched_frame_ids) {
    bad_matched_frame_ids.clear();
    const auto &frame = map.frame(frame_id);
    std::map<int, int> id2num_covisible_obs;
    for (const auto &track_id : frame.track_ids_) {
        if (track_id == -1)
            continue;
        const auto &track = map.track(track_id);
        for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
            if (id2num_covisible_obs.count(t_frame_id) == 0) {
                id2num_covisible_obs[t_frame_id] = 1;
            } else {
                id2num_covisible_obs.at(t_frame_id)++;
            }
        }
    }

    int num_good = 0, num_all = 0;
    for (const auto id : map.frameid2framepairids_[frame_id]) {
        const auto &fp = map.frame_pairs_[id];
        const auto &frame1 = map.frame(fp.id1);
        const auto &frame2 = map.frame(fp.id2);
        // if (frame_id > 1550) {
        //     std::cout << fp.id2 << " " << fp.id1 << " "
        //               << (frame1.registered ? 1 : 0) << " "
        //               << (frame2.registered ? 1 : 0) << std::endl;
        // }
        if (!(frame1.registered && frame2.registered))
            continue;

        int num_covise = -1;
        if (fp.id1 == frame_id) {
            if (id2num_covisible_obs.count(fp.id2) != 0)
                num_covise = id2num_covisible_obs.at(fp.id2);
        } else {
            if (id2num_covisible_obs.count(fp.id1) != 0)
                num_covise = id2num_covisible_obs.at(fp.id1);
        }

        // if (frame_id > 1550) {
        //     std::cout << fp.id2 << " " << fp.id1 << " " << num_covise
        //               << std::endl;
        // }

        if (num_covise >= 10)
            continue;
        // matched but not covisible

        ++num_all;
        std::vector<char> inlier_mask;

        if (IsGoodRelativePose(map, fp, inlier_mask)) { // TODO use tri angle
            ++num_good;
        } else {
            int bad_neighbor_id = fp.id1 == frame_id ? fp.id2 : fp.id1;
            bad_matched_frame_ids.insert(bad_neighbor_id);
        }
    }

    if (num_good < num_all) {
        LOG(WARNING) << "Bad Relative Pose\n";
        return false;
    }
    return true;
}

inline std::vector<int> GetMatchedFrameIds(Map &map, int frame_id) {
    std::vector<int> matched_frame_ids;
    for (const int &id : map.frameid2matched_frameids_[frame_id]) {
        if (!map.frame(id).registered)
            continue;
        matched_frame_ids.emplace_back(id);
    }
    matched_frame_ids.emplace_back(frame_id);
    return matched_frame_ids;
}

std::vector<std::set<int>> DivideMatchedFrames(Map &map, Frame &frame,
                                               Frame &frame2) {
    const std::vector<int> matched_frame_ids =
        GetMatchedFrameIds(map, frame.id);

    std::map<int, int> id2num_covisible1, id2num_covisible2;
    for (auto &id : matched_frame_ids)
        if (id != frame.id)
            id2num_covisible1[id] = id2num_covisible2[id] = 0;
    for (auto &track_id : frame.track_ids_) {
        if (track_id == -1)
            continue;
        for (auto &[t_frame_id, t_p2d_id] : map.track(track_id).observations_) {
            if (id2num_covisible1.count(t_frame_id) != 0) {
                id2num_covisible1[t_frame_id]++;
            }
        }
    }
    for (auto &track_id : frame2.track_ids_) {
        if (track_id == -1)
            continue;
        for (auto &[t_frame_id, t_p2d_id] : map.track(track_id).observations_) {
            if (id2num_covisible2.count(t_frame_id) != 0) {
                id2num_covisible2[t_frame_id]++;
            }
        }
    }

    std::vector<std::set<int>> cor_frame_ids_vec(2);
    for (auto &id : matched_frame_ids) {
        if (id != frame.id) {
            // printf("%d %d %d\n", id, tmp_set[id], tmp_set1[id]);
            if (id2num_covisible1[id] > id2num_covisible2[id]) {
                cor_frame_ids_vec[0].insert(id);
                printf("0 %d\n", id);
            } else if (id2num_covisible2[id] > id2num_covisible1[id]) {
                cor_frame_ids_vec[1].insert(id);
                printf("1 %d\n", id);
            }
        }
    }
    // CHECK(cor_frame_ids_vec[0].size() > 0);
    // CHECK(cor_frame_ids_vec[1].size() > 0);
    return cor_frame_ids_vec;
}

LoopInfo GetLoopInfo(Map &map, Frame &frame1, Frame &frame2) {
    LoopInfo loop_info;
    loop_info.frame_id = frame1.id;
    loop_info.twc_vec = {frame1.Tcw.inverse(), frame2.Tcw.inverse()};
    loop_info.cor_frame_ids_vec = DivideMatchedFrames(map, frame1, frame2);

    // TODO It is a simple method to get scale observation
    int count = 0;
    double depth1 = 0, depth2 = 0;
    for (size_t i = 0; i < frame1.track_ids_.size(); ++i) {
        const int track_id1 = frame1.track_ids_[i],
                  track_id2 = frame2.track_ids_[i];
        if (track_id1 == -1 || track_id2 == -1)
            continue;
        const auto &track1 = map.track(track_id1),
                   &track2 = map.track(track_id2);

        vector3 p3d1 = frame1.Tcw.q * track1.point3d_ + frame1.Tcw.t;
        vector3 p3d2 = frame2.Tcw.q * track2.point3d_ + frame2.Tcw.t;
        depth1 += p3d1.z();
        depth2 += p3d2.z();
        count++;
    }
    if (count >= 4) {
        loop_info.scale_obs = depth2 / depth1;
    }
    printf("Set SCALE: %lf = %lf / %lf %d\n", loop_info.scale_obs, depth2,
           depth1, count);
    return loop_info;
}

bool CheckNegtiveDepth(const Map &map, const Frame &frame1,
                       const Frame &frame2) {
    for (auto &track_id : frame1.track_ids_) {
        if (track_id == -1)
            continue;
        auto &track = map.track(track_id);
        const vector3 p3d = frame2.Tcw.q * track.point3d_ + frame2.Tcw.t;
        if (p3d.z() < 0) {
            return true;
        }
    }
    for (auto &track_id : frame2.track_ids_) {
        if (track_id == -1)
            continue;
        auto &track = map.track(track_id);
        const vector3 p3d = frame1.Tcw.q * track.point3d_ + frame1.Tcw.t;
        if (p3d.z() < 0) {
            return true;
        }
    }
    return false;
}

inline bool TryLocate(Map &map, const int frame_id,
                      const std::set<int> &local_frame_ids,
                      Point3dProcessor *p3d_processor_) {
    bool reg_success = RegisterImageLocal(frame_id, local_frame_ids, map);

    if (!reg_success) {
        bool have_neighbor = false;
        std::vector<int> adjacent_frame_ids = {frame_id - 1, frame_id + 1};
        for (auto &id : adjacent_frame_ids) {
            if (local_frame_ids.count(id) != 0) {
                map.frame(frame_id).registered = false;
                p3d_processor_->TriangulateFramePoint(
                    map, id, p3d_processor_->th_rpe_lba_);
                map.frame(frame_id).registered = true;
                have_neighbor = true;
            }
        }
        if (have_neighbor)
            reg_success = RegisterImageLocal(frame_id, local_frame_ids, map);
    }
    return reg_success;
}

inline void MergeTrackLoop(Map &map, Frame &frame1, Frame &frame2) {
    for (size_t i = 0; i < frame1.track_ids_.size(); ++i) {
        // merge track in two local_map
        const int track_id = frame2.track_ids_[i];
        if (track_id == -1)
            continue;
        auto &track = map.track(track_id);
        if (track.observations_.count(frame1.id) != 0)
            continue;

        const int track_id1 = frame1.track_ids_[i];
        if (track_id1 != -1) { //  try merge track1 into track
            auto &track1 = map.track(track_id1);
            for (const auto &[t_frame_id, t_p2d_id] : track1.observations_) {
                if (track.observations_.count(t_frame_id) == 0) {
                    track.observations_[t_frame_id] = t_p2d_id;
                    map.frame(t_frame_id).track_ids_[t_p2d_id] = track_id;
                } else {
                    map.frame(t_frame_id).track_ids_[t_p2d_id] = -1;
                    map.DeleteNumCorHavePoint3D(t_frame_id, t_p2d_id);
                }
            }
            track1.outlier = true;
            continue;
        }

        frame1.track_ids_[i] = track_id;
        track.observations_[frame1.id] = i;
        map.AddNumCorHavePoint3D(frame1.id, i);
    }
}

bool ErrorCorrector::CheckAndCorrectPose(Map &map, int frame_id, int iter) {
    std::set<int> bad_matched_frame_ids;
    if (CheckAllRelativePose(map, frame_id, bad_matched_frame_ids))
        return false;
    if (!TryLocate(map, frame_id, bad_matched_frame_ids, p3d_processor_))
        return false;

    auto &frame = map.frame(frame_id);
    const std::vector<int> matched_frame_ids =
        GetMatchedFrameIds(map, frame_id);
    for (auto &id : matched_frame_ids) {
        std::cout << "|" << id << std::endl;
    }

    // PoseGraph
    const double dist = (frame.Tcw.center() - map.tmp_frame.center()).norm();
    const bool negtive_depth = CheckNegtiveDepth(map, map.tmp_frame, frame);
    std::cout << "DIST:  " << dist << std::endl;
    if (dist > 1.5 || negtive_depth) {
        KeyFrameSelection(map, matched_frame_ids, true);
        UpdateByRefFrame(map);
        LoopInfo loop_info = GetLoopInfo(map, frame, map.tmp_frame);
        if (loop_info.cor_frame_ids_vec[0].size() == 0 ||
            loop_info.cor_frame_ids_vec[1].size() == 0)
            return false;
        if (only_correct_with_sim3_ && loop_info.scale_obs == -1)
            return false;
        ba_solver_->ScalePoseGraphUnorder(loop_info, map, true);
    }

    MergeTrackLoop(map, frame, map.tmp_frame);

    // BA
    if (dist > 0.02) {
        printf("size: %d : ", matched_frame_ids.size());
        for (auto &id : matched_frame_ids) {
            printf("%d ", id);
        }
        printf("\n");
        ba_solver_->KGBA(map, matched_frame_ids, true);
        p3d_processor_->FilterPoints3d(map, p3d_processor_->th_rpe_gba_,
                                       p3d_processor_->th_angle_gba_);
    }

    CheckAllRelativePose(map, frame_id, bad_matched_frame_ids);
    map.tmp_frame.registered = false;

    return true;
}

bool ErrorCorrector::CheckAndCorrectPoseAll(Map &map, int frame_id) {
    std::set<int> bad_matched_frame_ids;
    if (CheckAllRelativePose(map, frame_id, bad_matched_frame_ids))
        return false;

    if (!TryLocate(map, frame_id, bad_matched_frame_ids, p3d_processor_))
        return false;

    WriteColMapDataBinary("./before/", map);

    auto &frame = map.frame(frame_id);
    const std::vector<int> matched_frame_ids =
        GetMatchedFrameIds(map, frame_id);
    for (auto &id : matched_frame_ids) {
        std::cout << "|" << id << std::endl;
    }

    // PoseGraph
    const double dist = (frame.Tcw.center() - map.tmp_frame.center()).norm();
    const bool negtive_depth = CheckNegtiveDepth(map, map.tmp_frame, frame);
    std::cout << "DIST:  " << dist << std::endl;
    if (dist > 1.5 || negtive_depth) {
        LoopInfo loop_info = GetLoopInfo(map, frame, map.tmp_frame);
        if (loop_info.cor_frame_ids_vec[0].size() == 0 ||
            loop_info.cor_frame_ids_vec[1].size() == 0)
            return false;
        if (only_correct_with_sim3_ && loop_info.scale_obs == -1)
            return false;
        ba_solver_->ScalePoseGraphUnorder(loop_info, map, false);
    }

    MergeTrackLoop(map, frame, map.tmp_frame);

    // BA
    if (dist > 0.02) {
        printf("size: %d : ", matched_frame_ids.size());
        for (auto &id : matched_frame_ids) {
            printf("%d ", id);
        }
        printf("\n");
        ba_solver_->GBA(map);
        p3d_processor_->FilterPoints3d(map, p3d_processor_->th_rpe_gba_,
                                       p3d_processor_->th_angle_gba_);
    }

    CheckAllRelativePose(map, frame_id, bad_matched_frame_ids);
    map.tmp_frame.registered = false;

    WriteColMapDataBinary("./after/", map);

    return true;
}

} // namespace xrsfm
