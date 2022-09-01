//
// Created by SENSETIME\yezhichao1 on 2020/10/19.
//
#include "map.h"

#include <glog/logging.h>

#include "geometry/essential.h"
#include "geometry/track_processor.h"

namespace xrsfm {
int CorrespondenceGraph::GetMatch(int frame_id1, int frame_id2, std::vector<Match> &matches) {
    int count = 0;
    matches.resize(0);
    for (int ftr_id1 = 0; ftr_id1 < frame_node_vec_[frame_id1].corrs_vector.size(); ++ftr_id1) {
        for (auto &corr : frame_node_vec_[frame_id1].corrs_vector[ftr_id1]) {
            if (corr.first == frame_id2) {
                int ftr_id2 = corr.second;
                matches.emplace_back(ftr_id1, ftr_id2);
                count++;
            }
        }
    }
    return count;
};

void Map::Init() {
    frameid2pairid_.clear();
    for (int i = 0; i < frame_pairs_.size(); ++i) {
        assert(frames_.size() < 32768); // 2^15 INT_MAX=2147483647=2^31
        const auto &fp = frame_pairs_[i];
        int idpair = fp.id1 < fp.id2 ? fp.id1 * 32768 + fp.id2 : fp.id2 * 32768 + fp.id1;
        frameid2pairid_[idpair] = i;
    }

    frameid2framepairids_.clear();
    frameid2matched_frameids_.clear();
    frameid2covisible_frameids_.clear();
    for (const auto &frame : frames_) {
        frameid2framepairids_[frame.id] = std::vector<int>(0);
        frameid2matched_frameids_[frame.id] = std::vector<int>(0);
        frameid2covisible_frameids_[frame.id] = std::vector<int>(0);
    }
    for (int i = 0; i < frame_pairs_.size(); ++i) {
        frameid2framepairids_[frame_pairs_[i].id1].emplace_back(i);
        frameid2framepairids_[frame_pairs_[i].id2].emplace_back(i);
    }
    for (const auto &fp : frame_pairs_) {
        frameid2matched_frameids_[fp.id1].emplace_back(fp.id2);
        frameid2matched_frameids_[fp.id2].emplace_back(fp.id1);
    }
    corr_graph_.frame_node_vec_.resize(frames_.size() + 1); //TODO here a bug
    for (const auto &frame : frames_) {
        corr_graph_.frame_node_vec_[frame.id].num_observations = 0;
        corr_graph_.frame_node_vec_[frame.id].num_visible_point3d = 0;
        corr_graph_.frame_node_vec_[frame.id].num_correspondences = 0;
        corr_graph_.frame_node_vec_[frame.id].corrs_vector.assign(frame.points.size(), std::vector<std::pair<int, int>>(0));
    }
    for (const auto &frame_pair : frame_pairs_) {
        const int id1 = frame_pair.id1;
        const int id2 = frame_pair.id2;
        const auto &matches = frame_pair.matches;
        auto &image1 = corr_graph_.frame_node_vec_.at(id1);
        auto &image2 = corr_graph_.frame_node_vec_.at(id2);
        int num_inlier_matches = 0;
        assert(matches.size() == frame_pair.inlier_mask.size());
        for (int i = 0; i < matches.size(); ++i) {
            if (!frame_pair.inlier_mask[i]) continue;
            const auto &match = matches[i];
            auto &corrs_vector1 = image1.corrs_vector.at(match.id1);
            auto &corrs_vector2 = image2.corrs_vector.at(match.id2);
            corrs_vector1.emplace_back(id2, match.id2);
            corrs_vector2.emplace_back(id1, match.id1);
            num_inlier_matches++;
        }
        image1.num_correspondences += num_inlier_matches;
        image2.num_correspondences += num_inlier_matches;
    }

    for (auto &frame : frames_) {
        frame.num_correspondences_have_point3D_.assign(frame.points.size(), 0);
    }
}

void Map::RemoveRedundancyPoints() {
    Init();
    // remove unused frame points
    std::vector<std::vector<int>> id2nid_vec(frames_.size(), std::vector<int>(0));
    std::vector<std::vector<int>> nid2id_vec(frames_.size(), std::vector<int>(0));
    for (int i = 0; i < frames_.size(); ++i) {
        auto &frame = frames_[i];
        auto &id2nid = id2nid_vec[i];
        auto &nid2id = nid2id_vec[i];
        id2nid.assign(frame.points.size(), -1);
        int count = 0;
        for (int k = 0; k < frame.points.size(); ++k) {
            if (!corr_graph_.frame_node_vec_[i].corrs_vector[k].empty()) {
                nid2id.emplace_back(k);
                id2nid[k] = count;
                count++;
            }
        }

        for (int k = 0; k < nid2id.size(); ++k) {
            frame.points[k] = frame.points[nid2id[k]];
            frame.points_normalized[k] = frame.points_normalized[nid2id[k]];
        }
        frame.points.resize(nid2id.size());
        frame.points_normalized.resize(nid2id.size());
        frame.track_ids_.assign(nid2id.size(), -1);
    }
    for (auto &fp : frame_pairs_) {
        const auto &id2nid1 = id2nid_vec[fp.id1];
        const auto &id2nid2 = id2nid_vec[fp.id2];
        for (auto &m : fp.matches) {
            m.id1 = id2nid1[m.id1];
            m.id2 = id2nid2[m.id2];
        }
    }
}

void Map::RemoveRedundancyPoints(Map &tmp) {
    Init();
    // remove unused frame points
    std::vector<std::vector<int>> id2nid_vec(frames_.size(), std::vector<int>(0));
    std::vector<std::vector<int>> nid2id_vec(frames_.size(), std::vector<int>(0));
    for (int i = 0; i < frames_.size(); ++i) {
        auto &frame = frames_[i];
        auto &id2nid = id2nid_vec[i];
        auto &nid2id = nid2id_vec[i];
        id2nid.assign(frame.points.size(), -1);
        int count = 0;
        for (int k = 0; k < frame.points.size(); ++k) {
            if (!corr_graph_.frame_node_vec_[i].corrs_vector[k].empty()) {
                nid2id.emplace_back(k);
                id2nid[k] = count;
                count++;
            }
        }
        for (int k = 0; k < nid2id.size(); ++k) {
            frame.points[k] = frame.points[nid2id[k]];
            frame.points_normalized[k] = frame.points_normalized[nid2id[k]];
        }

        frame.points.resize(nid2id.size());
        frame.points_normalized.resize(nid2id.size());
        frame.track_ids_.assign(nid2id.size(), -1);

        if (tmp.frame_map_.count(i + 1)) {
            auto &frame1 = tmp.frame_map_[i + 1];
            frame1.points_normalized = frame1.points;
            for (int k = 0; k < nid2id.size(); ++k) {
                frame1.points[k] = frame1.points[nid2id[k]];
                const int track_id = frame1.track_ids_[nid2id[k]];
                frame1.track_ids_[k] = track_id;
                // CHECK(tmp.track_map_[track_id].observations_[frame1.id] == nid2id[k]);
                // tmp.track_map_[track_id].observations_[frame1.id] = k;
            }
            frame1.points.resize(nid2id.size());
            frame1.track_ids_.resize(nid2id.size());
        }
    }

    for (auto &fp : frame_pairs_) {
        const auto &id2nid1 = id2nid_vec[fp.id1];
        const auto &id2nid2 = id2nid_vec[fp.id2];
        for (auto &m : fp.matches) {
            m.id1 = id2nid1[m.id1];
            m.id2 = id2nid2[m.id2];
        }
    }
}

int Map::MaxPoint3dFrameId() {
    int best_id = -1, max_num_p3d = 0;
    for (const auto &frame : frames_) {
        if (frame.registered || frame.registered_fail) continue;
        // skip bad camera in unorder
        // if (cameras_[frame.camera_id].distort_params[0] == 0) continue;
        // only use sequence image
        // if (frame.camera_id != cameras_.back().id) continue;

        // if (bad_focal.count(frame.id) != 0) continue;
        int num_p3d = frames_[frame.id].num_visible_points3D_;
        if (num_p3d > max_num_p3d) {
            max_num_p3d = num_p3d;
            best_id = frame.id;
        }
    }

    // bool exit_good_neibor = false;
    // for (const auto id : frameid2framepairids_[best_id]) {
    //   auto &fp = frame_pairs_[id];
    //   const auto &frame1 = frames_[fp.id1];
    //   const auto &frame2 = frames_[fp.id2];
    //   if (fp.matches.size() < 100) continue;
    //   if ((fp.id1 == best_id && frame2.registered) || (fp.id2 == best_id && frame1.registered)) {
    //     exit_good_neibor = true;
    //     break;
    //   }
    // }

    // for (const auto &frame : frames_) {
    //   if (frame.registered) continue;
    //   int num_p3d = 0;
    //   const auto &corrs_vector = corr_graph_.frame_node_vec_[frame.id].corrs_vector;
    //   for (const auto &corrs : corrs_vector) {
    //     for (const auto &[t_frame_id, t_p2d_id] : corrs) {
    //       const auto &t_frame = frames_[t_frame_id];
    //       if (!t_frame.registered || t_frame.track_ids_[t_p2d_id] == -1) continue;
    //       const auto &track = tracks_[t_frame.track_ids_[t_p2d_id]];
    //       if (track.outlier) continue;
    //       num_p3d++;
    //       break;
    //     }
    //   }
    //   if (num_p3d > max_num_p3d1) {
    //     max_num_p3d1 = num_p3d;
    //     best_id1 = frame.id;
    //   }
    // }

    // for (const auto &frame : frames_) {
    //   if (frame.registered) continue;
    //   const auto &corrs_vector = corr_graph_.frame_node_vec_[frame.id].corrs_vector;
    //   for (int i = 0; i < corrs_vector.size(); ++i) {
    //     int num_p3d = 0;
    //     const auto &corrs = corrs_vector[i];
    //     for (const auto &[t_frame_id, t_p2d_id] : corrs) {
    //       const auto &t_frame = frames_[t_frame_id];
    //       if (!t_frame.registered || t_frame.track_ids_[t_p2d_id] == -1) continue;
    //       const auto &track = tracks_[t_frame.track_ids_[t_p2d_id]];
    //       if (track.outlier) continue;
    //       num_p3d++;
    //     }

    //     if (frame.num_correspondences_have_point3D_[i] != num_p3d) {
    //       printf("error %d %d %d %d %d\n", frame.id, i, frame.num_correspondences_have_point3D_[i], num_p3d);
    //       CHECK(frame.num_correspondences_have_point3D_[i] == num_p3d);
    //     }
    //   }
    // }
    // CHECK(max_num_p3d1 == frames_[best_id1].num_visible_points3D_);
    printf("Frame id: %d visible point3d num: %d \n", best_id, max_num_p3d);
    if (max_num_p3d < 20) return -1;
    return best_id;
}

int Map::MaxPoint3dFrameId1() {
    int best_id = -1, max_num_p3d = 0;
    int best_id1 = -1, max_num_p3d1 = 0;

    for (const auto &frame : frames_) {
        if (frame.registered) continue;
        if (cameras_[frame.camera_id].distort_params[0] == 0) continue;
        int num_p3d = 0, num_p3d_key = 0;
        const auto &corrs_vector = corr_graph_.frame_node_vec_[frame.id].corrs_vector;
        for (const auto &corrs : corrs_vector) {
            bool visit = false, visit_key = false;
            for (const auto &[t_frame_id, t_p2d_id] : corrs) {
                const auto &t_frame = frames_[t_frame_id];
                if (!t_frame.registered || t_frame.track_ids_[t_p2d_id] == -1) continue;
                const auto &track = tracks_[t_frame.track_ids_[t_p2d_id]];
                if (track.outlier) continue;
                visit = true;
                // if (track.is_keypoint) {
                //   visit_key = true;
                //   break;
                // }
                break;
            }
            if (visit) num_p3d++;
            if (visit_key) num_p3d_key++;
        }
        if (num_p3d > max_num_p3d) {
            max_num_p3d = num_p3d;
            best_id = frame.id;
        }
        if (num_p3d_key > max_num_p3d1) {
            max_num_p3d1 = num_p3d_key;
            best_id1 = frame.id;
        }
    }
    printf("Frame id: %d visible key point3d num: %d\n", best_id1, max_num_p3d1);
    printf("Frame id: %d visible point3d num: %d\n", best_id, max_num_p3d);
    // if (max_num_p3d1 >= 1000) return best_id1;
    return best_id;
}

int Map::get_num_p3d(const int frame_id) {
    int num_p3d = 0;
    const auto &corrs_vector = corr_graph_.frame_node_vec_[frame_id].corrs_vector;
    for (const auto &corrs : corrs_vector) {
        bool visit = false, visit_key = false;
        for (const auto &[t_frame_id, t_p2d_id] : corrs) {
            const auto &t_frame = frames_[t_frame_id];
            if (!t_frame.registered) continue;
            const int track_id = t_frame.track_ids_[t_p2d_id];
            if (track_id == -1 || tracks_[track_id].outlier) continue;
            visit = true;
            num_p3d++;
            break;
        }
    }
    return num_p3d;
}

std::pair<int, int> Map::MaxPoint3dFrameIdSeq() {
    // get seq head and tail
    int best_id = -1, max_num_p3d = 0;
    size_t num_frames = frames_.size();
    for (int i = 0; i < num_frames; ++i) {
        const auto &frame = frames_[i];
        if (frame.registered || frame.registered_fail) continue;
        if ((i - 1 >= 0 && frames_[i - 1].registered && frames_[i - 1].camera_id == frame.camera_id) || (i + 1 < num_frames && frames_[i + 1].registered && frames_[i + 1].camera_id == frame.camera_id)) {
            const int num_p3d = get_num_p3d(frame.id);
            int num_p3d2 = frame.num_visible_points3D_;
            CHECK(num_p3d == num_p3d2) << num_p3d << " " << num_p3d2 << "\n";
            if (num_p3d > max_num_p3d) {
                max_num_p3d = num_p3d;
                best_id = frame.id;
            }
        }
    }
    return std::pair<int, int>(best_id, max_num_p3d);
}

void Map::SearchCorrespondences(const Frame &frame, std::vector<vector2> &points2d,
                                std::vector<vector3> &points3d, std::vector<std::pair<int, int>> &cor_2d_3d_ids,
                                const bool use_p2d_normalized) {
    points2d.clear();
    points3d.clear();
    cor_2d_3d_ids.clear();

    auto &corrs_vector = corr_graph_.frame_node_vec_[frame.id].corrs_vector;
    for (int p2d_id = 0; p2d_id < corrs_vector.size(); ++p2d_id) {
        auto &corrs = corrs_vector[p2d_id];
        for (auto &corr : corrs) {
            int t_frame_id = corr.first;
            int t_p2d_id = corr.second;
            if (!frames_[t_frame_id].registered) continue;
            int p3d_id = frames_[t_frame_id].track_ids_[t_p2d_id];
            if (p3d_id != -1 && !tracks_[p3d_id].outlier) {
                if (use_p2d_normalized) {
                    points2d.emplace_back(frame.points_normalized[p2d_id]);
                } else {
                    points2d.emplace_back(frame.points[p2d_id]);
                }
                points3d.emplace_back(tracks_[p3d_id].point3d_);
                // std::cout << tracks_[p3d_id].point3d_.transpose() << " ";
                cor_2d_3d_ids.emplace_back(std::pair<int, int>(p2d_id, p3d_id));
                break;
            }
        }
    }
}

void Map::SearchCorrespondences1(const Frame &frame, const std::set<int> cor_frame_id,
                                 std::vector<vector2> &points2d, std::vector<vector3> &points3d,
                                 std::vector<std::pair<int, int>> &cor_2d_3d_ids, const bool use_p2d_normalized) {
    points2d.clear();
    points3d.clear();
    cor_2d_3d_ids.clear();

    auto &corrs_vector = corr_graph_.frame_node_vec_[frame.id].corrs_vector;
    for (int p2d_id = 0; p2d_id < corrs_vector.size(); ++p2d_id) {
        auto &corrs = corrs_vector[p2d_id];
        for (auto &corr : corrs) {
            int t_frame_id = corr.first;
            int t_p2d_id = corr.second;
            if (!frames_[t_frame_id].registered) continue;
            if (cor_frame_id.count(t_frame_id) == 0) continue;

            int p3d_id = frames_[t_frame_id].track_ids_[t_p2d_id];
            if (p3d_id != -1 && !tracks_[p3d_id].outlier) {
                if (use_p2d_normalized) {
                    points2d.emplace_back(frame.points_normalized[p2d_id]);
                } else {
                    points2d.emplace_back(frame.points[p2d_id]);
                }
                points3d.emplace_back(tracks_[p3d_id].point3d_);
                cor_2d_3d_ids.emplace_back(std::pair<int, int>(p2d_id, p3d_id));
                break;
            }
        }
    }
}

void Map::SearchCorrespondencesOrder(const Frame &frame, std::vector<vector2> &points2d,
                                     std::vector<vector3> &points3d,
                                     std::vector<std::pair<int, int>> &cor_2d_3d_ids) {
    points2d.clear();
    points3d.clear();
    cor_2d_3d_ids.clear();

    FramePair frame_pair;
    for (auto &t_frame_pair : frame_pairs_) {
        if (t_frame_pair.id1 == frame.id - 1 && t_frame_pair.id2 == frame.id) {
            frame_pair = t_frame_pair;
            break;
        }
    }

    auto &last_frame = frames_[frame.id - 1];
    for (int i = 0; i < frame_pair.matches.size(); ++i) {
        if (!frame_pair.inlier_mask[i]) continue;
        auto &match = frame_pair.matches[i];
        int track_id = last_frame.track_ids_[match.id1];
        if (track_id == -1) continue;
        points2d.emplace_back(frame.points[match.id2]);
        points3d.emplace_back(tracks_[track_id].point3d_);
        cor_2d_3d_ids.emplace_back(std::pair<int, int>(match.id2, track_id));
    }
}

void Map::LogFrameReprojectError1(int frame_id) {
    auto &frame = frames_[frame_id];
    double sre = 0;
    int count = 0;
    for (int i = 0; i < frame.track_ids_.size(); ++i) {
        if (frame.track_ids_[i] == -1) continue;
        Track &track = tracks_[frame.track_ids_[i]];
        if (track.outlier) continue;
        vector3 p_c = frame.Tcw.q * track.point3d_ + frame.Tcw.t;
        double z = p_c.z();
        vector2 res = p_c.head<2>() / z - frame.points_normalized[i];
        count++;
        sre += res.norm();
        // if (res.norm() * 885 > 10) printf("-track id %d %d re %lf\n", frame.track_ids_[i], track.ref_id, res.norm() *
        // 885);
    }
    printf("frame id %d cout %d re %lf\n", frame.id, count, sre * 885 / count);
}

void Map::LogFrameReprojectError() {
    for (auto &frame : frames_) {
        if (!frame.registered) continue;
        double sre = 0;
        int count = 0;
        for (int i = 0; i < frame.track_ids_.size(); ++i) {
            if (frame.track_ids_[i] == -1) continue;
            Track &track = tracks_[frame.track_ids_[i]];
            if (track.outlier) continue;
            vector3 p_c = frame.Tcw.q * track.point3d_ + frame.Tcw.t;
            double z = p_c.z();
            vector2 res = p_c.head<2>() / z - frame.points_normalized[i];
            count++;
            sre += res.norm();
        }
        printf("frame id %d cout %d re %lf", frame.id, count, sre * 885 / count);
        if (frame.is_keyframe) std::cout << "-";
        std::cout << std::endl;
    }
    printf("all frame num %zu\n", frames_.size());
}

bool FindPair(const std::vector<FramePair> &frame_pairs, const int id1, const int id2, FramePair &frame_pair) {
    for (auto &t_frame_pair : frame_pairs) {
        if (t_frame_pair.id1 == id1 && t_frame_pair.id2 == id2) {
            frame_pair = t_frame_pair;
            return true;
        }
    }
    return false;
}

FramePair FindPair(const std::vector<FramePair> &frame_pairs, const int id1, const int id2) {
    FramePair frame_pair;
    if (!FindPair(frame_pairs, id1, id2, frame_pair)) {
        std::cerr << "NO SUCH FRAME PAIR\n";
    }
    return frame_pair;
}

bool UpdateCovisiblity(Map &map, int frame_id) {
    auto &frame = map.frames_[frame_id];
    // add a covisible edge if two neighbor frame have 10+ covisible p3d
    std::map<int, int> id2num_covisible_pt;
    for (const auto &track_id : frame.track_ids_) {
        if (track_id == -1) continue;
        const auto &track = map.tracks_[track_id];

        for (const auto [t_frame_id, t_p2d_id] : track.observations_) {
            if (id2num_covisible_pt.count(t_frame_id) == 0) {
                id2num_covisible_pt[t_frame_id] = 1;
            } else {
                id2num_covisible_pt[t_frame_id]++;
            }
        }
    }

    int count_covisibile_images = 0;
    for (const auto &[t_frame_id, num_covisible_pt] : id2num_covisible_pt) {
        if (num_covisible_pt > 10) {
            count_covisibile_images++;
            map.frameid2covisible_frameids_[frame_id].emplace_back(t_frame_id);
            map.frameid2covisible_frameids_[t_frame_id].emplace_back(frame_id);
        }
    }

    if (count_covisibile_images == 0) {
        map.DeregistrationFrame(frame_id);
        map.frames_[frame_id].registered_fail = true;
        LOG(ERROR) << "Frame " << frame_id << ": fail to registered , no enough covisibility\n";
        return false;
    }

    // update number for registered neighbor frame
    int num_neighbors_registered = 0;
    for (const auto &id : map.frameid2matched_frameids_[frame_id]) {
        if (!map.frames_[id].registered) continue;
        num_neighbors_registered++;
        map.frames_[id].num_neighbors_registered++;
    }
    frame.num_neighbors_registered = num_neighbors_registered;

    return true;
}

void KeyFrameSelection(Map &map, std::vector<int> loop_matched_frame_id, const bool is_sequential_data) {
    constexpr int th_obs = 3;
    constexpr int num_min_obs = 200;
    constexpr double th_min_ratio = 0.6;

    // update keyframe
    for (auto &frame : map.frames_) {
        if (!frame.registered) continue;
        frame.tcw_old = frame.Tcw;
        if (!frame.is_keyframe) continue;
        if (frame.id == map.init_id1 || frame.id == map.init_id2) continue;

        // step1: enough redundant observations
        int num_p3d = 0, num_p3d_redundant = 0;
        for (const auto track_id : frame.track_ids_) {
            if (track_id == -1) continue;
            const auto &track = map.tracks_[track_id];
            num_p3d++;

            int count = 0;
            for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
                const auto &t_frame = map.frames_[t_frame_id];
                if (t_frame_id != frame.id && t_frame.is_keyframe) count++;
            }
            if (count >= th_obs) num_p3d_redundant++;
        }
        if (num_p3d_redundant < num_min_obs || num_p3d_redundant < th_min_ratio * num_p3d) continue;

        // step2: ensure at least one connect
        std::set<int> id_covisible_key;
        const auto &id_covisibility = map.frameid2covisible_frameids_[frame.id];
        for (const auto &id : id_covisibility) {
            if (map.frames_[id].is_keyframe && id != frame.id) id_covisible_key.insert(id);
        }
        if (id_covisibility.empty()) continue;

        // step3: ensure connect between sequential data
        if (is_sequential_data || frame.camera_id == map.cameras_.size() - 1) {
            int min_connect = INT_MAX;
            for (auto it = id_covisible_key.begin(); next(it) != id_covisible_key.end(); it++) {
                int id1 = *it, id2 = *next(it);
                if (id1 < frame.id && id2 > frame.id) {
                    int count = 0;
                    for (auto &track_id : map.frames_[id1].track_ids_) {
                        if (track_id == -1) continue;
                        const auto &track = map.tracks_[track_id];
                        if (track.observations_.count(id2) == 0) continue;
                        count++;
                    }
                    if (count < min_connect) {
                        min_connect = count;
                    }
                }
            }
            if (min_connect < num_min_obs) continue;
        }

        // step4: keep hierarchical structure
        // std::map<int, int> track2change_level;
        // for (const auto track_id : frame.track_ids_) {
        //   if (track_id == -1) continue;
        //   const auto &track = map.tracks_[track_id];
        //   if (track.hierarchical_level < frame.hierarchical_level) continue;

        //   int min_level1 = -1, min_level2 = -1;
        //   for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
        //     const auto &t_frame = map.frames_[t_frame_id];
        //     if (t_frame_id == frame.id || !t_frame.is_keyframe) continue;

        //     int level = t_frame.hierarchical_level;
        //     if (min_level1 == -1) {
        //       min_level1 = level;
        //     } else if (min_level2 == -1 || level < min_level2) {
        //       if (level < min_level1) {
        //         min_level2 = min_level1;
        //         min_level1 = level;
        //       } else {
        //         min_level2 = level;
        //       }
        //       if (min_level2 <= track.hierarchical_level) break;
        //     }
        //   }
        //   if (min_level2 > track.hierarchical_level) track2change_level[track_id] = min_level2;
        // }

        // bool keep_hierarchicy = true;
        // for (auto &id : id_covisible_key) {
        //   auto &cov_frame = map.frames_[id];
        //   if (cov_frame.hierarchical_level <= frame.hierarchical_level) continue;

        //   int count = 0, count1 = 0;
        //   const int level = cov_frame.hierarchical_level;
        //   for (auto &track_id : map.frames_[id].track_ids_) {
        //     if (track_id == -1) continue;
        //     const auto &track = map.tracks_[track_id];
        //     int track_level = track.hierarchical_level;
        //     if (track_level < level) {
        //       count1++;
        //     }
        //     if (track2change_level.count(track_id) != 0) {
        //       track_level = track2change_level[track_id];
        //     }
        //     if (track_level < level) {
        //       count++;
        //       if (count == MIN_OBS_NUM_LEVEL) break;
        //     }
        //   }
        //   std::cout << frame.id << " " << cov_frame.id << " " << count << " " << count1 << std::endl;
        //   if (count < MIN_OBS_NUM_LEVEL && count != count1) {
        //     keep_hierarchicy = false;
        //     break;
        //   }
        // }
        // if (!keep_hierarchicy) continue;

        frame.is_keyframe = false;
        printf("!!! init remove: %d %d %d\n", frame.id, num_p3d_redundant, num_p3d);
    }

    for (auto &frame_id : loop_matched_frame_id) {
        std::cout << "|" << frame_id << std::endl;
        map.frames_[frame_id].ref_id = -1; // fix bug
        map.frames_[frame_id].is_keyframe = true;
    }

    for (auto &frame : map.frames_) {
        std::map<int, int> covisiblity;
        if (!frame.registered) continue;
        if (frame.is_keyframe) continue;
        if (frame.ref_id != -1) continue;

        for (const auto &track_id : frame.track_ids_) {
            if (track_id == -1) continue;
            const auto &track = map.tracks_[track_id];
            if (track.outlier) continue;
            for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
                if (map.frames_[t_frame_id].is_keyframe && t_frame_id != frame.id) { // DIF(ORBSLAM use scalelevel)
                    if (covisiblity.count(t_frame_id) == 0)
                        covisiblity[t_frame_id] = 1;
                    else
                        covisiblity[t_frame_id]++;
                }
            }
        }

        if (covisiblity.empty()) {
            // todo fix it
            LOG(ERROR) << "no covisiblity key frame";
            for (int i = 1; i < map.frames_.size(); ++i) {
                if (frame.id + i < map.frames_.size()) {
                    if (map.frames_[frame.id + i].is_keyframe) {
                        frame.ref_id = frame.id + i;
                        break;
                    }
                }
                if (frame.id - i > 0) {
                    if (map.frames_[frame.id - i].is_keyframe) {
                        frame.ref_id = frame.id - i;
                        break;
                    }
                }
            }
        } else {
            std::vector<std::pair<int, int>> covisiblity_vec(covisiblity.begin(), covisiblity.end());
            std::sort(covisiblity_vec.begin(), covisiblity_vec.end(),
                      [](const std::pair<int, int> &a, const std::pair<int, int> &b) -> bool { return a.second > b.second; });
            frame.ref_id = covisiblity_vec[0].first;
            CHECK(covisiblity_vec[0].second > 0);
        }
    }

    // update keypoint
    for (auto &track : map.tracks_) {
        if (track.outlier) continue;
        int count = 0;
        for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
            if (map.frames_[t_frame_id].is_keyframe) {
                count++;
            }
        }
        track.is_keypoint = count >= 2;
    }
}

void UpdateByRefFrame(Map &map) {
    for (auto &frame : map.frames_) {
        if (!frame.registered) continue;
        if (frame.is_keyframe) continue;
        Frame *ref_frame = &map.frames_[frame.ref_id];
        int count = 0;
        while (!ref_frame->is_keyframe) {
            ref_frame = &map.frames_[ref_frame->ref_id];
            count++;
            if (count > 100) {
                LOG(ERROR) << "too many loop " << ref_frame->id << " " << ref_frame->ref_id;
                break;
            }
        }
        frame.ref_id = ref_frame->id;
        frame.Tcw = frame.tcw_old.mul(ref_frame->tcw_old.inverse().mul(ref_frame->Tcw));
    }
}

void Map::DeregistrationFrame(int frame_id) {
    auto &frame = frames_[frame_id];
    frame.registered = false;
    for (int i = 0; i < frame.track_ids_.size(); ++i) {
        auto &id = frame.track_ids_[i];
        // for (int &id : frame.track_ids_) {
        if (id == -1) continue;
        auto &track = tracks_[id];
        if (track.outlier) continue;
        track.observations_.erase(frame.id);
        DeleteNumCorHavePoint3D(frame.id, i);
        id = -1;
    }
}

void Map::LogReprojectError() {
    int count = 0, count_key = 0;
    double sre = 0, sre_key = 0;
    for (const auto &frame : frames_) {
        if (!frame.registered) continue;
        for (size_t i = 0; i < frame.track_ids_.size(); ++i) {
            const auto &track_id = frame.track_ids_[i];
            if (track_id == -1) continue;
            Track &track = tracks_[track_id];
            if (track.outlier) continue;
            const vector3 p_c = frame.Tcw.q * track.point3d_ + frame.Tcw.t;
            const vector2 res = p_c.hnormalized() - frame.points_normalized[i];
            const double focal = cameras_[frame.camera_id].fx();

            count++;
            sre += res.norm() * focal;
            if (frame.is_keyframe) {
                count_key++;
                sre_key += res.norm() * focal;
            }
        }
    }
    printf("%d avg rpe %lf %lf\n", count, sre / count, sre_key / count_key);
    sre_key_ = sre_key / count_key;

    // int count_tra = 0, count_len = 0;
    // for (const auto &track : tracks_) {
    //   if (track.outlier) continue;
    //   count_tra++;
    //   count_len += track.observations_.size();
    // }
    // avg_track_length_ = 1.0 * count_len / count_tra;
    // std::cout<<count_tra<<" "<<avg_track_length_<<std::endl;
}
} // namespace xrsfm
