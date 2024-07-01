#include "incremental_mapper.h"

namespace xrsfm {
IncrementalMapper::IncrementalMapper() {}

void IncrementalMapper::Reconstruct(Map &map) {
    p3d_processor = Point3dProcessor(options.th_rpe_lba, options.th_angle_lba,
                                     options.th_rpe_gba, options.th_angle_lba);
    error_corrector.ba_solver_ = &ba_solver;
    error_corrector.p3d_processor_ = &p3d_processor;
    error_corrector.only_correct_with_sim3_ = options.only_with_sim3;

    timer.tot.resume();
    // 1. Map Initialization
    FramePair init_frame_pair;
    if (options.init_id1 != -1 && options.init_id2 != -1) {
        printf("Init with Given Frames %d %d\n", options.init_id1,
               options.init_id2);
        init_frame_pair =
            FindPair(map.frame_pairs_, options.init_id1, options.init_id2);
    } else {
        printf("Found Init Frame Pair %d %d\n", options.init_id1,
               options.init_id2);
        init_frame_pair.id1 = options.init_id1;
        init_frame_pair.id2 = options.init_id2;
        if (!FindInitFramePair(map, init_frame_pair)) {
            std::cout << "Fail to Find Frame Pair!!!" << std::endl;
            exit(-1);
        }
    }
    std::cout << "Found Init Frame Pair Done!" << std::endl;
    InitializeMap(map, init_frame_pair);
    ba_solver.GBA(map);

    // 2. Map Iterative Extension
    int num_image_reg = 2, num_image_reg_pre = 2;
    for (int iter = 0; iter < map.NumFrames(); iter++) {
        printf("-----------------------------------------------\n");
        // 1) Pose Estimation
        timer.reg.resume();
        const int frame_id = map.MaxPoint3dFrameId();
        if (frame_id == -1)
            break;
        printf("Iter %d %d %s\n", iter, frame_id,
               map.frame(frame_id).name.c_str());
        if (!RegisterImage(frame_id, map)) {
            if (options.stop_when_register_fail)
                break;
            map.frame(frame_id).registered_fail = true;
            continue;
        }
        timer.reg.stop();

        // 2) Check & Correct Frame Pose
        if (options.correct_pose)
            error_corrector.CheckAndCorrectPose(map, frame_id, iter);

        // 3) Point Estimation
        TIMING(timer.tri, p3d_processor.TriangulateFramePoint(
                              map, frame_id, options.th_angle_lba));
        TIMING(timer.fil,
               p3d_processor.FilterPointsFrame(
                   map, frame_id, options.th_rpe_lba, options.th_angle_lba));
        TIMING(timer.merge,
               p3d_processor.MergeTracks(map, frame_id, options.th_rpe_lba));
        if (p3d_processor.CheckFrameMeasurement(map, frame_id))
            continue;

        // 4) Local Optimization
        TIMING(timer.lba, ba_solver.LBA(frame_id, map));
        TIMING(timer.fil,
               p3d_processor.FilterPointsFrame(
                   map, frame_id, options.th_rpe_lba, options.th_angle_lba));

        // 5) Global Optimization
        if (num_image_reg++ > 1.2 * num_image_reg_pre) {
            TIMING(timer.che, p3d_processor.CheckTrackDepth(map));
            p3d_processor.CheckFramesMeasurement(map, options.th_rpe_lba,
                                                 options.th_angle_lba);
            TIMING(timer.gba, ba_solver.KGBA(map, std::vector<int>(0), true));
            // TIMING(timer.gba, ba_solver.GBA(map));
            TIMING(timer.fil,
                   p3d_processor.FilterPoints3d(map, options.th_rpe_gba,
                                                options.th_angle_gba));
            num_image_reg_pre = num_image_reg;
        }

        UpdateCovisiblity(map, frame_id);
    }
    timer.tot.stop();

    // ba_solver.GBA(map);

    for (auto &timer_ptr : timer.timer_vec) {
        timer_ptr->print();
    }
}

int get_num_matches(Map &map, int reference_id, int image_id) {
    int count = 0;
    for (int i = 0; i < map.frame(reference_id).points.size(); ++i) {
        const auto &corrs =
            map.corr_graph_.frame_node_vec_.at(reference_id).corrs_vector.at(i);
        bool has_corr = false;
        for (const auto &corr : corrs) {
            if (corr.first == image_id) {
                has_corr = true;
                break;
            }
        }
        if (!has_corr)
            continue;
        bool has_p3d = false;
        for (const auto &corr : corrs) {
            if (map.frame(corr.first).track_ids_.at(corr.second) != -1) {
                has_p3d = true;
                break;
            }
        }
        count++;
    }
    return count;
}

RegisterResult RegisterSequential(Map &map) {
    constexpr int num_max_gap_keyframe = 5;

    std::vector<int> image_ids;
    for (auto &frame : map.frames_) {
        image_ids.push_back(frame.id);
    }

    // find candidate
    struct ImageInfo {
        int image_index;
        int direct;
        int num_matches;
    };
    std::vector<ImageInfo> candidates;
    for (int i = 0; i < image_ids.size(); ++i) {
        const int image_id = image_ids.at(i);
        if (!map.frame(image_id).registered)
            continue;
        // we assert all reference images is registered
        for (int j = i - 1; j >= 0; --j) {
            const auto &last_image = map.frame(image_ids.at(j));
            if (last_image.registered)
                break;
            if (last_image.has_pose)
                continue;
            ImageInfo image_info;
            image_info.image_index = i;
            image_info.direct = -1;
            image_info.num_matches =
                get_num_matches(map, image_id, image_ids.at(j));
            candidates.emplace_back(image_info);
            break;
        }
        for (int j = i + 1; j < image_ids.size(); ++j) {
            const auto &next_image = map.frame(image_ids.at(j));
            if (next_image.registered)
                break;
            if (next_image.has_pose)
                continue;
            ImageInfo image_info;
            image_info.image_index = i;
            image_info.direct = 1;
            image_info.num_matches =
                get_num_matches(map, image_id, image_ids.at(j));
            candidates.emplace_back(image_info);
            break;
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](auto &p0, auto &p1) {
        return p0.num_matches > p1.num_matches;
    });

    // register
    RegisterResult reg_result;
    for (const auto &info : candidates) {
        const int reference_id = image_ids.at(info.image_index);
        const int direct = info.direct;
        const int image_id = image_ids.at(info.image_index + direct);
        std::cout << "reference:" << reference_id << " " << direct
                  << " image id:" << image_id << std::endl;
        const std::set<int> references = {reference_id};
        reg_result = RegisterImageLocal1(image_id, references, map);

        if (reg_result.success) {
            map.frame(image_id).has_pose = true;
        } else {
            continue;
        }

        // keyframe selection strategy for sequential images
        for (int i = 2; i <= num_max_gap_keyframe; ++i) {
            const int next_index = info.image_index + i * info.direct;
            if (!(next_index >= 0 && next_index < image_ids.size()))
                break;
            const int next_image_id = image_ids.at(next_index);
            if (map.frame(next_image_id).has_pose)
                break;
            const auto reg_next_result =
                RegisterImageLocal1(next_image_id, references, map);
            if (!reg_next_result.success)
                break;
            const int num_inlier = reg_next_result.num_inlier;
            const int num_corrs = reg_next_result.num_correspondences;

            if ((num_inlier >= 100 && num_inlier > 0.8 * num_corrs) ||
                (num_inlier >= 200 && num_inlier > 0.7 * num_corrs) ||
                (num_inlier >= 300 && num_inlier > 0.6 * num_corrs)) {
                map.frame(next_image_id).has_pose = true;
                reg_result = reg_next_result;
                if (next_image_id % 5 == 0) { // TODO remove it
                    break;
                }
            } else {
                break;
            }

            std::cout << "num_inliers: " << num_inlier << " " << num_corrs
                      << std::endl;
        }

        if (reg_result.success) {
            break;
        }
    }

    if (reg_result.success) {
        std::cout << "register : " << reg_result.frame_id << std::endl;
        auto &frame = map.frame(reg_result.frame_id);
        frame.registered = true;
        frame.Tcw = reg_result.tcw;

        for (const auto &[p2d_id, p3d_id] : reg_result.correspondences_2d3d) {
            auto &track = map.track(p3d_id);
            if (track.observations_.count(frame.id) == 0) {
                frame.track_ids_[p2d_id] = p3d_id;
                track.observations_[frame.id] = p2d_id;
                map.AddNumCorHavePoint3D(frame.id, p2d_id);
            } else {
                // if this track has been observed, compare rpe
            }
        }
    }

    return reg_result;
}

void IncrementalMapper::ReconstructKeyFrames(Map &map) {
    p3d_processor = Point3dProcessor(options.th_rpe_lba, options.th_angle_lba,
                                     options.th_rpe_gba, options.th_angle_lba);
    error_corrector.ba_solver_ = &ba_solver;
    error_corrector.p3d_processor_ = &p3d_processor;
    error_corrector.only_correct_with_sim3_ = options.only_with_sim3;

    timer.tot.resume();
    // 1. Map Initialization
    FramePair init_frame_pair;
    if (options.init_id1 != -1 && options.init_id2 != -1) {
        printf("Init with Given Frames %d %d\n", options.init_id1,
               options.init_id2);
        init_frame_pair =
            FindPair(map.frame_pairs_, options.init_id1, options.init_id2);
    } else {
        printf("Found Init Frame Pair %d %d\n", options.init_id1,
               options.init_id2);
        init_frame_pair.id1 = options.init_id1;
        init_frame_pair.id2 = options.init_id2;
        if (!FindInitFramePair(map, init_frame_pair)) {
            std::cout << "Fail to Find Frame Pair!!!" << std::endl;
            exit(-1);
        }
    }
    std::cout << "Found Init Frame Pair Done!" << std::endl;
    InitializeMap(map, init_frame_pair);
    ba_solver.GBA(map);

    // 2. Map Iterative Extension
    int num_image_reg = 2, num_image_reg_pre = 2;
    for (int iter = 0; iter < map.NumFrames(); iter++) {
        printf("-----------------------------------------------\n");
        // 1) Pose Estimation
        timer.reg.resume();
        auto reg_result = RegisterSequential(map);
        if (!reg_result.success) {
            break;
        }
        const int frame_id = reg_result.frame_id;

        // 2) Check & Correct Frame Pose
        if (options.correct_pose)
            error_corrector.CheckAndCorrectPoseAll(map, frame_id);

        // 3) Point Estimation
        TIMING(timer.tri, p3d_processor.TriangulateFramePoint(
                              map, frame_id, options.th_angle_lba));
        TIMING(timer.fil,
               p3d_processor.FilterPointsFrame(
                   map, frame_id, options.th_rpe_lba, options.th_angle_lba));

        // if (p3d_processor.CheckFrameMeasurement(map, frame_id))
        //     continue;

        // 4) Local Optimization
        for (int i = 0; i < 2; ++i) {
            TIMING(timer.lba, ba_solver.LBA(frame_id, map));
            TIMING(timer.merge, p3d_processor.MergeTracks(map, frame_id,
                                                          options.th_rpe_lba));
            TIMING(timer.fil, p3d_processor.FilterPointsFrame(
                                  map, frame_id, options.th_rpe_lba,
                                  options.th_angle_lba));
        }

        // 5) Global Optimization
        if (num_image_reg++ > 1.2 * num_image_reg_pre) {
            TIMING(timer.che, p3d_processor.CheckTrackDepth(map));
            p3d_processor.CheckFramesMeasurement(map, options.th_rpe_lba,
                                                 options.th_angle_lba);
            TIMING(timer.gba, ba_solver.GBA(map));
            // TODO merge points continue points
            TIMING(timer.fil,
                   p3d_processor.FilterPoints3d(map, options.th_rpe_gba,
                                                options.th_angle_gba));
            num_image_reg_pre = num_image_reg;
        }

        if (!UpdateCovisiblity(map, frame_id)) {
            break;
        }
        // if (frame_id > 1700)
        //     break;
    }
    timer.tot.stop();

    for (auto &timer_ptr : timer.timer_vec) {
        timer_ptr->print();
    }
}

} // namespace xrsfm
