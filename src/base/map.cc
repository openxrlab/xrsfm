//
// Created by SENSETIME\yezhichao1 on 2020/10/19.
//
#include "map.h"

#include <glog/logging.h>

#include "geometry/essential.h"
#include "geometry/track_processor.h"

namespace xrsfm {
int CorrespondenceGraph::GetMatch(int frame_id1, int frame_id2,
                                  std::vector<Match> &matches) {
    int count = 0;
    matches.resize(0);
    for (int ftr_id1 = 0;
         ftr_id1 < frame_node_vec_[frame_id1].corrs_vector.size(); ++ftr_id1) {
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
    assert(NumFrames() < 32768); // 2^15 INT_MAX=2147483647=2^31
    frameid2framepairids_.clear();
    frameid2matched_frameids_.clear();
    frameid2covisible_frameids_.clear();
    for (const auto &frame : frames_) {
        frameid2framepairids_[frame.id] = std::vector<int>(0);
        frameid2matched_frameids_[frame.id] = std::vector<int>(0);
        frameid2covisible_frameids_[frame.id] = std::vector<int>(0);
    }
    for (int i = 0; i < frame_pairs_.size(); ++i) {
        frameid2framepairids_.at(frame_pairs_[i].id1).emplace_back(i);
        frameid2framepairids_.at(frame_pairs_[i].id2).emplace_back(i);
    }
    for (const auto &fp : frame_pairs_) {
        frameid2matched_frameids_.at(fp.id1).emplace_back(fp.id2);
        frameid2matched_frameids_.at(fp.id2).emplace_back(fp.id1);
    }
    corr_graph_.frame_node_vec_.resize(NumFrames() + 1); // TODO here a bug
    for (const auto &frame : frames_) {
        corr_graph_.frame_node_vec_.at(frame.id).num_observations = 0;
        corr_graph_.frame_node_vec_.at(frame.id).num_visible_point3d = 0;
        corr_graph_.frame_node_vec_.at(frame.id).num_correspondences = 0;
        corr_graph_.frame_node_vec_.at(frame.id).corrs_vector.assign(
            frame.points.size(), std::vector<std::pair<int, int>>(0));
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
            if (!frame_pair.inlier_mask.at(i))
                continue;
            const auto &match = matches.at(i);
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
    std::vector<std::vector<int>> id2nid_vec(NumFrames(), std::vector<int>(0));
    std::vector<std::vector<int>> nid2id_vec(NumFrames(), std::vector<int>(0));
    for (int i = 0; i < NumFrames(); ++i) {
        auto &frame = this->frame(i);
        auto &id2nid = id2nid_vec.at(i);
        auto &nid2id = nid2id_vec.at(i);
        id2nid.assign(frame.points.size(), -1);
        int count = 0;
        for (int k = 0; k < frame.points.size(); ++k) {
            if (!corr_graph_.frame_node_vec_.at(i).corrs_vector.at(k).empty()) {
                nid2id.emplace_back(k);
                id2nid.at(k) = count;
                count++;
            }
        }

        for (int k = 0; k < nid2id.size(); ++k) {
            frame.points.at(k) = frame.points.at(nid2id.at(k));
        }
        frame.points.resize(nid2id.size());
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

int Map::MaxPoint3dFrameId() {
    int best_id = -1, max_num_p3d = 0;
    for (const auto &frame : frames_) {
        if (frame.registered || frame.registered_fail)
            continue;
        int num_p3d = this->frame(frame.id).num_visible_points3D_;
        if (num_p3d > max_num_p3d) {
            max_num_p3d = num_p3d;
            best_id = frame.id;
        }
    }

    printf("Frame id: %d visible point3d num: %d \n", best_id, max_num_p3d);
    if (max_num_p3d < 20)
        return -1;
    return best_id;
}

void Map::SearchCorrespondences(const Frame &frame,
                                std::vector<vector2> &points2d,
                                std::vector<vector3> &points3d,
                                std::vector<std::pair<int, int>> &cor_2d_3d_ids,
                                const bool use_p2d_normalized) {
    points2d.clear();
    points3d.clear();
    cor_2d_3d_ids.clear();

    auto &corrs_vector = corr_graph_.frame_node_vec_[frame.id].corrs_vector;
    for (int p2d_id = 0; p2d_id < corrs_vector.size(); ++p2d_id) {
        auto &corrs = corrs_vector[p2d_id];
        for (auto &[t_frame_id, t_p2d_id] : corrs) {
            if (!this->frame(t_frame_id).registered)
                continue;
            int p3d_id = this->frame(t_frame_id).track_ids_[t_p2d_id];
            if (p3d_id != -1 && !track(p3d_id).outlier) {
                points2d.emplace_back(frame.points[p2d_id]);
                points3d.emplace_back(track(p3d_id).point3d_);
                cor_2d_3d_ids.emplace_back(std::pair<int, int>(p2d_id, p3d_id));
                break;
            }
        }
    }

    if (use_p2d_normalized) {
        for (int i = 0; i < points2d.size(); ++i) {
            Eigen::Vector2d point2d_N;
            ImageToNormalized(Camera(frame.camera_id), points2d[i], point2d_N);
            points2d[i] = point2d_N;
        }
    }
}

void Map::SearchCorrespondences1(
    const Frame &frame, const std::set<int> cor_frame_id,
    std::vector<vector2> &points2d, std::vector<vector3> &points3d,
    std::vector<std::pair<int, int>> &cor_2d_3d_ids,
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
            if (!this->frame(t_frame_id).registered)
                continue;
            if (cor_frame_id.count(t_frame_id) == 0)
                continue;

            int p3d_id = this->frame(t_frame_id).track_ids_[t_p2d_id];
            if (p3d_id != -1 && !track(p3d_id).outlier) {
                points2d.emplace_back(frame.points[p2d_id]);
                points3d.emplace_back(track(p3d_id).point3d_);
                cor_2d_3d_ids.emplace_back(std::pair<int, int>(p2d_id, p3d_id));
                break;
            }
        }
    }

    if (use_p2d_normalized) {
        for (int i = 0; i < points2d.size(); ++i) {
            Eigen::Vector2d point2d_N;
            ImageToNormalized(Camera(frame.camera_id), points2d[i], point2d_N);
            points2d[i] = point2d_N;
        }
    }
}

bool FindPair(const std::vector<FramePair> &frame_pairs, const int id1,
              const int id2, FramePair &frame_pair) {
    for (auto &t_frame_pair : frame_pairs) {
        if (t_frame_pair.id1 == id1 && t_frame_pair.id2 == id2) {
            frame_pair = t_frame_pair;
            return true;
        }
    }
    return false;
}

FramePair FindPair(const std::vector<FramePair> &frame_pairs, const int id1,
                   const int id2) {
    FramePair frame_pair;
    if (!FindPair(frame_pairs, id1, id2, frame_pair)) {
        std::cerr << "NO SUCH FRAME PAIR\n";
    }
    return frame_pair;
}

bool UpdateCovisiblity(Map &map, int frame_id) {
    auto &frame = map.frame(frame_id);
    // add a covisible edge if two neighbor frame have 10+ covisible p3d
    std::map<int, int> id2num_covisible_pt;
    for (const auto &track_id : frame.track_ids_) {
        if (track_id == -1)
            continue;
        const auto &track = map.track(track_id);

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
        map.frame(frame_id).registered_fail = true;
        LOG(ERROR) << "Frame " << frame_id
                   << ": fail to registered , no enough covisibility\n";
        return false;
    }

    // update number for registered neighbor frame
    int num_neighbors_registered = 0;
    for (const auto &id : map.frameid2matched_frameids_[frame_id]) {
        if (!map.frame(id).registered)
            continue;
        num_neighbors_registered++;
        map.frame(id).num_neighbors_registered++;
    }
    frame.num_neighbors_registered = num_neighbors_registered;

    return true;
}

void KeyFrameSelection(Map &map, std::vector<int> loop_matched_frame_id,
                       const bool is_sequential_data) {
    constexpr int th_obs = 3;
    constexpr int num_min_obs = 200;
    constexpr double th_min_ratio = 0.6;

    // update keyframe
    for (auto &frame : map.frames_) {
        if (!frame.registered)
            continue;
        frame.tcw_old = frame.Tcw;
        if (!frame.is_keyframe)
            continue;
        if (frame.id == map.init_id1 || frame.id == map.init_id2)
            continue;

        // step1: enough redundant observations
        int num_p3d = 0, num_p3d_redundant = 0;
        for (const auto track_id : frame.track_ids_) {
            if (track_id == -1)
                continue;
            const auto &track = map.track(track_id);
            num_p3d++;

            int count = 0;
            for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
                const auto &t_frame = map.frame(t_frame_id);
                if (t_frame_id != frame.id && t_frame.is_keyframe)
                    count++;
            }
            if (count >= th_obs)
                num_p3d_redundant++;
        }
        if (num_p3d_redundant < num_min_obs ||
            num_p3d_redundant < th_min_ratio * num_p3d)
            continue;

        // step2: ensure at least one connect
        std::set<int> id_covisible_key;
        const auto &id_covisibility = map.frameid2covisible_frameids_[frame.id];
        for (const auto &id : id_covisibility) {
            if (map.frame(id).is_keyframe && id != frame.id)
                id_covisible_key.insert(id);
        }
        if (id_covisibility.empty())
            continue;

        // step3: ensure connect between sequential data
        if (is_sequential_data) {
            int min_connect = INT_MAX;
            for (auto it = id_covisible_key.begin();
                 next(it) != id_covisible_key.end(); it++) {
                int id1 = *it, id2 = *next(it);
                if (id1 < frame.id && id2 > frame.id) {
                    int count = 0;
                    for (auto &track_id : map.frame(id1).track_ids_) {
                        if (track_id == -1)
                            continue;
                        const auto &track = map.track(track_id);
                        if (track.observations_.count(id2) == 0)
                            continue;
                        count++;
                    }
                    if (count < min_connect) {
                        min_connect = count;
                    }
                }
            }
            if (min_connect < num_min_obs)
                continue;
        }

        frame.is_keyframe = false;
        printf("!!! init remove: %d %d %d\n", frame.id, num_p3d_redundant,
               num_p3d);
    }

    for (auto &frame_id : loop_matched_frame_id) {
        std::cout << "|" << frame_id << std::endl;
        map.frame(frame_id).ref_id = -1; // fix bug
        map.frame(frame_id).is_keyframe = true;
    }

    for (auto &frame : map.frames_) {
        std::map<int, int> covisiblity;
        if (!frame.registered)
            continue;
        if (frame.is_keyframe)
            continue;
        if (frame.ref_id != -1)
            continue;

        for (const auto &track_id : frame.track_ids_) {
            if (track_id == -1)
                continue;
            const auto &track = map.track(track_id);
            if (track.outlier)
                continue;
            for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
                if (map.frame(t_frame_id).is_keyframe &&
                    t_frame_id != frame.id) { // DIF(ORBSLAM use scalelevel)
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
            for (int i = 1; i < map.NumFrames(); ++i) {
                if (frame.id + i < map.NumFrames()) {
                    if (map.frame(frame.id + i).is_keyframe) {
                        frame.ref_id = frame.id + i;
                        break;
                    }
                }
                if (frame.id - i > 0) {
                    if (map.frame(frame.id - i).is_keyframe) {
                        frame.ref_id = frame.id - i;
                        break;
                    }
                }
            }
        } else {
            std::vector<std::pair<int, int>> covisiblity_vec(
                covisiblity.begin(), covisiblity.end());
            std::sort(covisiblity_vec.begin(), covisiblity_vec.end(),
                      [](const std::pair<int, int> &a,
                         const std::pair<int, int> &b) -> bool {
                          return a.second > b.second;
                      });
            frame.ref_id = covisiblity_vec[0].first;
            CHECK(covisiblity_vec[0].second > 0);
        }
    }

    // update keypoint
    for (auto &track : map.tracks_) {
        if (track.outlier)
            continue;
        int count = 0;
        for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
            if (map.frame(t_frame_id).is_keyframe) {
                count++;
            }
        }
        track.is_keypoint = count >= 2;
    }
}

void UpdateByRefFrame(Map &map) {
    for (auto &frame : map.frames_) {
        if (!frame.registered)
            continue;
        if (frame.is_keyframe)
            continue;
        Frame *ref_frame = &map.frame(frame.ref_id);
        int count = 0;
        while (!ref_frame->is_keyframe) {
            ref_frame = &map.frame(ref_frame->ref_id);
            count++;
            if (count > 100) {
                LOG(ERROR) << "too many loop " << ref_frame->id << " "
                           << ref_frame->ref_id;
                break;
            }
        }
        frame.ref_id = ref_frame->id;
        frame.Tcw =
            frame.tcw_old.mul(ref_frame->tcw_old.inverse().mul(ref_frame->Tcw));
    }
}

void Map::DeregistrationFrame(int frame_id) {
    auto &frame = this->frame(frame_id);
    frame.registered = false;
    for (int i = 0; i < frame.track_ids_.size(); ++i) {
        auto &id = frame.track_ids_[i];
        // for (int &id : frame.track_ids_) {
        if (id == -1)
            continue;
        auto &track = this->track(id);
        if (track.outlier)
            continue;
        track.observations_.erase(frame.id);
        DeleteNumCorHavePoint3D(frame.id, i);
        id = -1;
    }
}

} // namespace xrsfm
