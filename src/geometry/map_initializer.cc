//
// Created by SENSETIME\yezhichao1 on 2021/1/4.
//

#include "geometry/map_initializer.h"
#include "geometry/colmap/base/triangulation.h"
#include "geometry/essential.h"
#include "geometry/triangluate_svd.h"

namespace xrsfm {

bool CheckInitFramePair(const Map &map, FramePair &frame_pair, double min_angle = 16.0) {
    auto &frame1 = map.frames_[frame_pair.id1], &frame2 = map.frames_[frame_pair.id2];

    int inlier_num;
    std::vector<char> inlier_mask;
    std::vector<vector2> points1, points2;
    const int num_matches = frame_pair.matches.size();
    for (int i = 0; i < num_matches; ++i) {
        if (!frame_pair.inlier_mask[i]) continue;
        points1.push_back(frame1.points_normalized[frame_pair.matches[i].id1]);
        points2.push_back(frame2.points_normalized[frame_pair.matches[i].id2]);
    }
    if (points1.size() < 10) return false;
    const double th = 10.0 / map.cameras_[frame1.camera_id].fx();
    //add solve homograph
    xrsfm::solve_essential(points1, points2, th, frame_pair.E, inlier_num, inlier_mask);

    inlier_mask.clear();
    std::vector<vector3> point3ds;
    xrsfm::decompose_rt(frame_pair.E, points1, points2, frame_pair.R, frame_pair.t, point3ds, inlier_mask);
    Pose pose1 = Pose(quaternion::Identity(), vector3::Zero());
    Pose pose2 = Pose(quaternion(frame_pair.R), frame_pair.t);
    int id_p3d = 0, num_p3d = 0, num_p3d_valid = 0;
    for (size_t i = 0; i < num_matches; ++i) {
        if (frame_pair.inlier_mask[i]) { // assert track valid
            if (inlier_mask[id_p3d]) {
                double angle = colmap::CalculateTriangulationAngle(pose1.center(), pose2.center(), point3ds[id_p3d]);
                if (angle > DegToRad(min_angle)) {
                    num_p3d_valid++;
                }
                num_p3d++;
            }
            id_p3d++;
        }
    }

    if (num_p3d >= 0.5 * id_p3d && num_p3d_valid >= 0.5 * num_p3d && num_p3d_valid > 200) {
        printf("id %d-%d: %d %d %d\n", frame_pair.id1, frame_pair.id2, num_p3d_valid, num_p3d, id_p3d);
        return true;
    }
    return false;
}

bool FindInitFramePair(const Map &map, FramePair &init_frame_pair) {
    std::vector<int> init_id1_candidate;
    if (init_frame_pair.id1 == -1) {
        std::vector<std::pair<int, int>> frame_info_vec;
        for (const auto &frame : map.frames_) {
            // if (map.cameras_[frame.camera_id].distort_params[0] != 0) //TODO
            if (map.cameras_[frame.camera_id].valid())
                frame_info_vec.emplace_back(frame.id, map.frameid2matched_frameids_[frame.id].size());
        }
        std::sort(frame_info_vec.begin(), frame_info_vec.end(),
                  [](const std::pair<int, int> &a, const std::pair<int, int> &b) { return a.second > b.second; });
        for (const auto &[frame_id, num_matched_frame] : frame_info_vec) {
            init_id1_candidate.emplace_back(frame_id);
        }
    } else {
        init_id1_candidate.push_back(init_frame_pair.id1);
    }

    std::cout << init_id1_candidate.size() << std::endl;

    for (const auto &init_id1 : init_id1_candidate) {
        std::vector<int> init_id2_candidate;
        {
            std::unordered_map<int, int> num_correspondences;
            for (const auto &corrs : map.corr_graph_.frame_node_vec_[init_id1].corrs_vector) {
                for (const auto &[frame_id, p2d_id] : corrs) {
                    num_correspondences[frame_id]++;
                }
            }
            std::vector<std::pair<int, int>> frame_info_vec(num_correspondences.begin(), num_correspondences.end());
            std::sort(frame_info_vec.begin(), frame_info_vec.end(),
                      [](const std::pair<int, int> &a, const std::pair<int, int> &b) { return a.second > b.second; });
            for (const auto &[frame_id, num_matched_frame] : frame_info_vec) {
                init_id2_candidate.emplace_back(frame_id);
            }
        }

        for (const auto &init_id2 : init_id2_candidate) {
            // std::cout<<init_id2<<"\n";
            FramePair frame_pair;
            if (init_id1 < init_id2)
                FindPair(map.frame_pairs_, init_id1, init_id2, frame_pair);
            else
                FindPair(map.frame_pairs_, init_id2, init_id1, frame_pair);
            if (CheckInitFramePair(map, frame_pair, 16.0)) {
                init_frame_pair = frame_pair;
                return true;
            }
        }
    }
    return false;
}

void InitializeMap(Map &map, FramePair &frame_pair) {
    const int id1 = frame_pair.id1, id2 = frame_pair.id2;
    auto &frame1 = map.frames_[id1], &frame2 = map.frames_[id2];
    printf("Initialize id1: %d %s id2: %d %s\n", id1, frame1.name.c_str(), id2, frame2.name.c_str());

    int inlier_num;
    std::vector<char> inlier_mask;
    std::vector<vector2> points1, points2;
    const int num_matches = frame_pair.matches.size();
    for (int i = 0; i < num_matches; ++i) {
        if (!frame_pair.inlier_mask[i]) continue;
        points1.push_back(frame1.points_normalized[frame_pair.matches[i].id1]);
        points2.push_back(frame2.points_normalized[frame_pair.matches[i].id2]);
    }
    const double th = 10.0 / map.cameras_[frame1.camera_id].fx();
    xrsfm::solve_essential(points1, points2, th, frame_pair.E, inlier_num, inlier_mask);
    printf("Init essential %d/%zu\n", inlier_num, points1.size());

    inlier_mask.clear();
    std::vector<vector3> point3ds;
    xrsfm::decompose_rt(frame_pair.E, points1, points2, frame_pair.R, frame_pair.t, point3ds, inlier_mask);

    frame1.Tcw = Pose(quaternion::Identity(), vector3::Zero());
    frame1.registered = frame1.is_keyframe = true;
    frame1.hierarchical_level = 0;
    frame2.Tcw = Pose(quaternion(frame_pair.R), frame_pair.t);
    frame2.registered = frame2.is_keyframe = true;
    frame2.hierarchical_level = 0;

    map.tracks_.clear();
    Track track;
    int tri_num = 0, id_p3d = 0;
    for (size_t i = 0; i < num_matches; ++i) {
        if (frame_pair.inlier_mask[i]) { // assert track valid
            if (inlier_mask[id_p3d]) {
                const int ftr_id1 = frame_pair.matches[i].id1;
                const int ftr_id2 = frame_pair.matches[i].id2;
                const int track_id = static_cast<int>(map.tracks_.size());
                frame1.track_ids_[ftr_id1] = frame2.track_ids_[ftr_id2] = track_id;
                track.observations_[id1] = ftr_id1;
                track.observations_[id2] = ftr_id2;
                track.point3d_ = point3ds[id_p3d];
                track.is_keypoint = true;
                map.tracks_.emplace_back(track);
                tri_num++;

                map.AddNumCorHavePoint3D(id1, ftr_id1);
                map.AddNumCorHavePoint3D(id2, ftr_id2);
            }
            id_p3d++;
        }
    }

    map.init_id1 = id1;
    map.init_id2 = id2;
    map.frameid2covisible_frameids_[id1].emplace_back(id2);
    map.frameid2covisible_frameids_[id2].emplace_back(id1);
    printf("Initialize Triangulated num: %d/%zu\n", tri_num, points1.size());
}

} // namespace xrsfm
