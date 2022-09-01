#include "match_expansion.h"

namespace xrsfm {
void MatchMap::PatchInit(const Map &map) {
    m_patch_track_num = 0;
    m_patch_tracks.clear();

    m_patch_kpt_ids.resize(map.frames_.size());
    pt2patch_id_vec.resize(map.frames_.size());
    m_frames_patch_track_ids.resize(map.frames_.size());
    for (size_t i = 0; i < map.frames_.size(); ++i) {
        const int step_x = image_size[i].width / col_num;
        const int step_y = image_size[i].height / row_num;

        auto &kps = map.frames_[i].keypoints_;

        m_patch_kpt_ids[i].resize(patch_num);
        pt2patch_id_vec[i].resize(kps.size());
        m_frames_patch_track_ids[i].assign(patch_num, -1);
        for (size_t k = 0; k < patch_num; ++k) {
            m_patch_kpt_ids[i][k].clear();
        }
        for (size_t k = 0; k < kps.size(); ++k) {
            auto &kpt = kps[k].pt;
            int x = (int)(kpt.x) / step_x;
            int y = (int)(kpt.y) / step_y;
            if (x >= col_num) x = col_num - 1;
            if (y >= row_num) y = row_num - 1;
            int patch_id = y * col_num + x;
            pt2patch_id_vec[i][k] = patch_id;
            m_patch_kpt_ids[i][patch_id].push_back(k);
        }
    }
}

void MatchMap::PrintImageSize() {
    std::cout << "Image Size:\n";
    for (size_t i = 0; i < image_size.size(); ++i) {
        // int step_x  =image_size[i].width / col_num;
        // int step_y  =image_size[i].height / row_num;
        std::cout << "(" << i << "):(" << image_size[i].width << ", " << image_size[i].height << ")\n";
    }
}

void MatchMap::AddTrack(int frame_id1, int frame_id2, const Match &match,
                        const size_t num_keypoint1, const size_t num_keypoints2) {
    int idx1 = match.id1, idx2 = match.id2;
    auto &frame1_track = m_frames_track_ids[frame_id1];
    auto &frame2_track = m_frames_track_ids[frame_id2];
    if (idx1 > num_keypoint1 || idx2 > num_keypoints2) {
        printf("-----------------------------------error-------------------------------\n");
        std::cout << frame_id1 << " " << idx1 << " " << num_keypoint1 << std::endl;
        std::cout << frame_id2 << " " << idx2 << " " << num_keypoints2 << std::endl;
    }
    if (frame1_track[idx1] == -1 && frame2_track[idx2] == -1) {
        frame1_track[idx1] = m_track_num;
        frame2_track[idx2] = m_track_num;
        TrackExpansionWrapper track;
        track.invalid = false;
        // printf("%f %f %f\n", p(0), p(1), p(2));
        track.observations_.insert(std::pair<int, int>(frame_id1, idx1));
        track.observations_.insert(std::pair<int, int>(frame_id2, idx2));
        // printf("%d\n",m_tracks.size());
        m_tracks.push_back(track);
        m_track_num++;
    } else if (frame1_track[idx1] == -1) {
        int track_id = frame1_track[idx1] = frame2_track[idx2];
        TrackExpansionWrapper &track = m_tracks[track_id];
        track.observations_.insert(std::pair<int, int>(frame_id1, idx1));
    } else if (frame2_track[idx2] == -1) {
        int track_id = frame2_track[idx2] = frame1_track[idx1];
        TrackExpansionWrapper &track = m_tracks[track_id];
        track.observations_.insert(std::pair<int, int>(frame_id2, idx2));
    } else if (frame1_track[idx1] != frame2_track[idx2]) {
        // printf("Warning Merge Track\n");
        int track_id1 = frame1_track[idx1];
        int track_id2 = frame2_track[idx2];
        TrackExpansionWrapper &track1 = m_tracks[track_id1];
        TrackExpansionWrapper &track2 = m_tracks[track_id2];
        // printf("Merge Track %d %d\n", track_id1, track_id2);
        // move track2 into track1
        for (auto &iter : track2.observations_) {
            int frame_id = iter.first;
            size_t point_id = iter.second;
            if (track1.observations_.count(frame_id) == 0) {
                track1.observations_.insert(std::pair<int, int>(frame_id, point_id));
                m_frames_track_ids[frame_id][point_id] = track_id1;
            } else if (track1.observations_[frame_id] == point_id) {
                m_frames_track_ids[frame_id][point_id] = track_id1;
            } else {
                // printf("Warning Merge Track %d %d\n", track1.mObservations[frame_id], point_id);
                m_frames_track_ids[frame_id][point_id] = -1;
            }
        }
        track2.invalid = true;
    }
}

void MatchMap::MakeTrack(const Map &map) { // map::
    m_tracks.clear();
    m_frames_track_ids.resize(map.frames_.size());
    // for (size_t i = 0; i < map.frames_.size(); i++) {
    //   m_frames_track_ids[i].assign(map.frames_[i].keypoints_.size(), -1);
    // }
    // for (auto &frame_pair : m_frame_pairs) {
    //   const int id1 = frame_pair.id1, id2 = frame_pair.id2;
    //   for (auto &match : frame_pair.matches) {
    //     AddTrack(id1, id2, match, map.frames_[id1].keypoints_.size(),
    //              map.frames_[id2].keypoints_.size());
    //   }
    // }
    for (size_t i = 0; i < map.frames_.size(); i++) {
        m_frames_track_ids[i].assign(map.frames_[i].track_ids_.size(), -1);
    }
    for (auto &frame_pair : m_frame_pairs) {
        const int id1 = frame_pair.id1, id2 = frame_pair.id2;
        for (auto &match : frame_pair.matches) {
            AddTrack(id1, id2, match, map.frames_[id1].track_ids_.size(),
                     map.frames_[id2].track_ids_.size());
        }
    }
}

void MatchMap::MakeTrack(const std::vector<Frame> &frames) { // map::
    m_tracks.clear();
    m_frames_track_ids.resize(frames.size());
    for (size_t i = 0; i < frames.size(); i++) {
        m_frames_track_ids[i].assign(frames[i].track_ids_.size(), -1);
    }
    for (auto &frame_pair : m_frame_pairs) {
        const int id1 = frame_pair.id1, id2 = frame_pair.id2;
        for (auto &match : frame_pair.matches) {
            AddTrack(id1, id2, match, frames[id1].track_ids_.size(), frames[id2].track_ids_.size());
        }
    }
}

void MatchMap::PrintFramesTrack() {
    std::cout << "PrintFramesTrack:\n";
    for (int i = 0; i < m_frames_track_ids.size(); i++) {
        for (int j = 0; j < m_frames_track_ids[i].size(); j++) {
            std::cout << "Feature " << j << " of frame " << i << " is tracked in id "
                      << m_frames_track_ids[i][j] << "\n";
        }
    }
}

void MatchMap::PrintPatchIdOfFeatures() {
    std::cout << "PrintPatchIdOfFeatures:\n";
    for (int i = 0; i < pt2patch_id_vec.size(); i++) {
        for (int j = 0; j < pt2patch_id_vec[i].size(); j++) {
            std::cout << "Feature " << j << " of frame " << i << " is  in Patch " << pt2patch_id_vec[i][j]
                      << "\n";
        }
    }
}

void MatchMap::PrintFeaturesOfPatch() {
    std::cout << "PrintFeaturesOfPatch:\n";
    for (int i = 0; i < m_patch_kpt_ids.size(); i++) {
        for (int j = 0; j < m_patch_kpt_ids[i].size(); j++) {
            std::cout << "of frame " << i << "is  in Patch " << j << " includes: ";
            for (const auto &kpt_id : m_patch_kpt_ids[i][j]) {
                std::cout << kpt_id << " ";
            }
            std::cout << "\n";
        }
    }
}

void MatchMap::PrintIdPair() {
    for (size_t id1 = 0; id1 < id2pair_id.size(); id1++) {
        for (const auto &item : id2pair_id.at(id1)) {
            std::cout << id1 << " - " << item.first << ":" << item.second << "\n";
        }
    }
}

std::vector<std::map<int, int>> MatchMap::MakeIdPair() { // map::
    id2pair_id.resize(m_num_frame);
    for (int i = 0; i < (int)m_frame_pairs.size(); ++i) {
        int id1 = m_frame_pairs[i].id1;
        int id2 = m_frame_pairs[i].id2;
        id2pair_id[id1][id2] = i;
        id2pair_id[id2][id1] = i;
    }
    return id2pair_id;
}

// map::
void MatchMap::LoadRetrievalRank(const std::string &filename,
                                 const std::map<std::string, int> &name_map,
                                 std::map<int, std::vector<int>> &retrieval_rank_of_frames,
                                 bool skip_matches) {
    std::ifstream infile(filename.c_str());

    std::string line;
    std::set<std::string> missing_image_names;
    while (getline(infile, line)) {
        if (line == "") continue;
        FramePair fp;
        std::istringstream s1(line);
        std::string image_name1, image_name2;
        s1 >> image_name1 >> image_name2;
        if (name_map.count(image_name1) == 0) { //
            // printf("Warning : missing %s in name map\n",image_name1.c_str());
            missing_image_names.insert(image_name1);
            continue;
        }
        if (name_map.count(image_name2) == 0) {
            missing_image_names.insert(image_name2);
            continue;
        }
        fp.id1 = name_map[image_name1];
        fp.id2 = name_map[image_name2];

        fp.matches.resize(0);
        if (!skip_matches) {
            while (getline(infile, line)) {
                if (line == "")
                    break;
                else {
                    Match m;
                    std::istringstream s2(line);
                    s2 >> m.id1 >> m.id2;
                    fp.matches.push_back(m);
                }
            }
            if (fp.matches.size() == 0) {
                continue;
            } else if (fp.matches.size() < 15) {
                printf("%d inlier not enough\n", fp.matches.size());
            }
        }
        fp.inlier_num_f = fp.matches.size();
        // frame_pairs.push_back(fp);
        retrieval_rank_of_frames[fp.id1].emplace_back(fp.id2);
    }

    for (const auto name : missing_image_names) {
        printf("Warning : missing %s in name map\n", name.c_str());
    }
}

void MatchMap::LoadMatch(const Map &map, std::vector<FramePair> &frame_pairs) {
    frame_pairs.reserve(map.frame_pairs_.size());
    for (const auto &frame_pair : map.frame_pairs_) {
        frame_pairs.emplace_back(frame_pair);
        frame_pairs.back().inlier_num_f = frame_pair.matches.size();
    }
}

void MatchMap::MakeCorrGraph(const Map &map) { // map::
    MatchExpansionSolver::CheckConsistenceOfFrameIdAndIndex(map);
    if (corr_graph.frame_node_vec_.size() == m_num_frame) {
        for (const auto &frame : map.frames_) {
            corr_graph.frame_node_vec_[frame.id].num_observations = 0; //(TODO)Is it ok to use frame.id
            corr_graph.frame_node_vec_[frame.id].num_visible_point3d = 0;
        }
    } else {
        corr_graph.frame_node_vec_.resize(m_num_frame);
        for (const auto &frame : map.frames_) {
            corr_graph.frame_node_vec_[frame.id].num_observations = 0;
            corr_graph.frame_node_vec_[frame.id].num_visible_point3d = 0;
            corr_graph.frame_node_vec_[frame.id].num_correspondences = 0;
            corr_graph.frame_node_vec_[frame.id].corrs_vector.resize(frame.keypoints_.size());
        }
        for (int i = 0; i < m_frame_pairs.size(); ++i) {
            int id1 = m_frame_pairs[i].id1;
            int id2 = m_frame_pairs[i].id2;
            const auto &matches = m_frame_pairs[i].matches;
            auto &image1 = corr_graph.frame_node_vec_[id1];
            auto &image2 = corr_graph.frame_node_vec_[id2];
            image1.num_correspondences += matches.size();
            image2.num_correspondences += matches.size();

            for (const auto &match : matches) {
                auto &corrs_vector1 = image1.corrs_vector[match.id1];
                auto &corrs_vector2 = image2.corrs_vector[match.id2];

                corrs_vector1.emplace_back(id2, match.id2);
                corrs_vector2.emplace_back(id1, match.id1);
            }
        }
    }
}

int MatchMap::GetMatches(int frame_id1, int frame_id2, std::vector<Match> &matches) {
    const auto &track_ids = m_frames_track_ids[frame_id1];
    int count = 0;
    matches.resize(0);
    for (size_t i = 0; i < track_ids.size(); ++i) {
        if (track_ids[i] == -1) continue;
        TrackExpansionWrapper &track = m_tracks[track_ids[i]];
        if (track.invalid) continue;
        auto &obs_infos = track.observations_;
        for (auto &obs_info : obs_infos) {
            if (obs_info.first == frame_id2) {
                count++;
                matches.emplace_back(Match(i, obs_info.second));
                break;
            }
        }
    }
    return count;
}

int MatchMap::GetMatcheNum(int frame_id1, int frame_id2) {
    int count = 0;
    for (const auto &track_id : m_frames_track_ids[frame_id1]) {
        if (track_id == -1) continue;
        const auto &track = m_tracks[track_id];
        if (track.invalid) continue;
        if (track.observations_.count(frame_id2) != 0) {
            count++;
        }
    }
    return count;
}

/////////////////////////////////////////
/// MatchExpansionSolver Implementation
////////////////////////////////////////

void MatchExpansionSolver::CheckConsistenceOfFrameIdAndIndex(const Map &map) {
    for (int i = 0; i < map.frames_.size(); i++) {
        CHECK(i == map.frames_[i].id) << "Error: frame id" << map.frames_[i].id << " != " << i << "\n";
    }
}

void MatchExpansionSolver::SetUp(const Map &map,
                                 const std::map<int, std::vector<int>> &id2retrieval_result,
                                 const std::vector<ImageSize> &image_size_vec,
                                 const size_t &initial_frame1, const size_t &initial_frame2) {
    m_initial_frame1 = initial_frame1;
    m_initial_frame2 = initial_frame2;

    m_matchmap.SetUpImageSize(image_size_vec);
    m_matchmap.SetUpFrameSize(map.frames_.size());
    m_matchmap.m_frame_pairs = map.frame_pairs_;
    for (auto &fp : m_matchmap.m_frame_pairs) {
        fp.inlier_num_f = fp.matches.size();
    }

    id2rank_vec = GetId2RankVec(id2retrieval_result);
    m_matchmap.verbose = verbose;
}

void SetSubtraction(const std::vector<int> &set1, const std::vector<int> &set2,
                    std::vector<int> &set3) {
    set3.resize(0);
    for (auto id : set1) {
        if (std::find(set2.begin(), set2.end(), id) == set2.end()) {
            set3.push_back(id);
        }
    }
}

void GetCandidateSimilarity(const int init_num, const std::vector<int> &can_reg,
                            const std::vector<int> &can_build,
                            const std::vector<std::map<int, int>> &id2rank_vec,
                            std::vector<std::set<int>> &matched_ids,
                            std::vector<FramePair> &frame_pairs) {
    for (const auto &id1 : can_reg) {
        for (const auto &id2 : can_build) {
            if (matched_ids[id1].count(id2) != 0) continue;
            if ((id2rank_vec[id1].count(id2) != 0 && id2rank_vec[id1].at(id2) <= 40) || (id2rank_vec[id2].count(id1) != 0 && id2rank_vec[id2].at(id1) <= 40)) {
                frame_pairs.emplace_back(id1, id2);
                matched_ids[id1].insert(id2);
                matched_ids[id2].insert(id1);
            }
        }
    }
}

void GetMayreg(const std::vector<int> &connect_frames, const std::vector<int> &reg_frames,
               const std::vector<std::map<int, int>> &frame2retrievalpairs,
               std::vector<int> &may_reg);

void MatchExpansionSolver::Run(const Map &map, std::vector<std::pair<int, int>> &image_pairs) {
    ///////////////////////////////////////////////////////////////////////////////////////
    // Get Potential Registered Frames
    //////////////////////////////////////////////////////////////////////////////////////
    m_matchmap.MakeIdPair();
    m_matchmap.MakeCorrGraph(map);
    std::vector<int> connect_frames(0), reg_frames(0), strong_reg_set(0);
    std::vector<int> may_reg_set(0);
    GetConnectedFrames(map, connect_frames);
    GetPotentialRegisteredFrames(map, 30, reg_frames);
    GetPotentialRegisteredFrames(map, 100, strong_reg_set);
    printf("frame number\n cov_1: %d  cov_30: %d cov_100:%d\n", connect_frames.size(), reg_frames.size(), strong_reg_set.size());
    m_matchmap.corr_graph.frame_node_vec_.clear();

    /////////////////////////////////////////////////
    // get candidate frame pairs(output)
    /////////////////////////////////////////////////
    std::vector<FramePair> candidates;
    // 在connect中,找到共视pair,pair中至少一帧为registered
    GetCandidateCovisibility(map, reg_frames, connect_frames, id2rank_vec, id2matched_ids,
                             candidates);
    std::cout << "#covisiblity candidates: " << candidates.size() << "\n";

    // 找到由registered与mayreg组成pair
    // SetSubtraction(connect_frames, strong_reg_set, can_reg);
    GetMayreg(connect_frames, reg_frames, id2rank_vec, may_reg_set);
    GetCandidateSimilarity(5, may_reg_set, reg_frames, id2rank_vec, id2matched_ids, candidates);
    std::cout << "#full candidates: " << candidates.size() << "\n";

    // remove duplicate frame pairs
    std::vector<std::set<int>> t_pair_list(map.frames_.size());
    for (const auto &fp : candidates) {
        if (fp.id1 < fp.id2)
            t_pair_list[fp.id1].insert(fp.id2);
        else
            t_pair_list[fp.id2].insert(fp.id1);
    }
    image_pairs.reserve(candidates.size());
    for (int i = 0; i < t_pair_list.size(); ++i) {
        for (auto it : t_pair_list[i]) {
            image_pairs.emplace_back(i, it);
        }
    }
    std::cout << "#filtered candidates: " << image_pairs.size() << " \n";
}

void MatchExpansionSolver::Print(const std::vector<int> &x_set) {
    std::cout << "(" << x_set.size() << "):";
    for (const auto &value : x_set) {
        std::cout << value << " ";
    }
    std::cout << "\n";
}

std::vector<std::map<int, int>> MatchExpansionSolver::GetId2RankVec(
    const std::map<int, std::vector<int>> &retrieval_rank_of_frames) {
    std::vector<std::map<int, int>> fp(m_matchmap.m_num_frame);
    for (size_t i = 0; i < m_matchmap.m_num_frame; ++i) {
        fp[i].clear();
        CHECK(retrieval_rank_of_frames.count(i))
            << "Error: frame " << i << " is missing in retrieval_rank_of_frames.\n";
        for (int k = 0; k < retrieval_rank_of_frames[i].size(); ++k) {
            fp[i][retrieval_rank_of_frames[i][k]] = k + 1;
        }
    }
    return fp;
}

void MatchExpansionSolver::GetConnectedFrames(const Map &map, std::vector<int> &set) {
    const auto &id_pair = m_matchmap.id2pair_id;
    const auto &frame_pairs = m_matchmap.m_frame_pairs;
    if (verbose) {
        m_matchmap.PrintIdPair();
    }
    // match 关联性 保证最小inliers
    std::set<int> candidate;
    std::set<int> new_candidate;
    std::vector<bool> registered(m_matchmap.m_num_frame);

    int id = GetInitFramePairId(id_pair);
    int image_id1 = frame_pairs[id].id1, image_id2 = frame_pairs[id].id2;

    registered[image_id1] = true;
    registered[image_id2] = true;
    candidate.insert(image_id1);
    candidate.insert(image_id2);
    set.push_back(image_id1);
    set.push_back(image_id2);

    int count = 2;
    while (!candidate.empty()) {
        new_candidate.clear();
        for (auto &image_id : candidate) {
            for (auto &id : id_pair[image_id]) {
                if (!registered[id.first]) {
                    new_candidate.insert(id.first);
                    registered[id.first] = true;
                    set.push_back(id.first);
                }
            }
        }
        candidate = new_candidate;
    }
}

void MatchExpansionSolver::PrintRetrievalMap() {
    std::cout << "retrievalframepairs_map";
    for (size_t i = 0; i < id2rank_vec.size(); i++) {
        for (const auto &item : id2rank_vec[i]) {
            std::cout << i << " - " << item.first << ":" << item.second << "\n";
        }
    }
}

int MatchExpansionSolver::GetInitFramePairId(const std::vector<std::map<int, int>> &id_pair) {
    CHECK(id_pair[m_initial_frame1].count(m_initial_frame2) != 0) << "error 21\n";
    int id = id_pair[m_initial_frame1].at(m_initial_frame2);
    return id;
}

void MatchExpansionSolver::GetPotentialRegisteredFrames(const Map &map, int num,
                                                        std::vector<int> &set) {
    std::vector<std::vector<bool>> potential_tri_mask; // frame feature
    SimulationSfM(map, num, potential_tri_mask);

    set.resize(0);
    for (size_t i = 0; i < potential_tri_mask.size(); ++i) {
        int count = 0;
        for (auto inlier : potential_tri_mask[i]) {
            if (inlier) count++;
        }
        if (count != 0 && count >= num) {
            set.push_back(i);
        }
    }
}

void MatchExpansionSolver::SimulationSfM(const Map &map, int num,
                                         std::vector<std::vector<bool>> &tri) {
    const auto &id_pair = m_matchmap.id2pair_id;
    auto &corr_graph = m_matchmap.corr_graph;
    const auto &frame_pairs = m_matchmap.m_frame_pairs;
    // match 关联性 保证最小inliers
    std::set<int> candidate_images;
    tri.resize(m_matchmap.m_num_frame);
    std::vector<bool> registered(m_matchmap.m_num_frame);
    for (int i = 0; i < m_matchmap.m_num_frame; ++i) {
        const auto &image = corr_graph.frame_node_vec_[i];
        tri[i].resize(image.corrs_vector.size(), false);
    }
    { // #8101 and #8214
        //  #1381 and #1602
        int id = GetInitFramePairId(id_pair); //#1730 and #1575
        int image_id1 = frame_pairs[id].id1, image_id2 = frame_pairs[id].id2;
        registered[image_id1] = true;
        registered[image_id2] = true;
        tri_points(image_id1, image_id2, corr_graph, tri);
    }
    int count = 2;
    while (1) {
        //  find next images
        std::vector<int> next_images(0);
        for (int i = 0; i < m_matchmap.m_num_frame; ++i) {
            if (registered[i]) continue;
            if (corr_graph.frame_node_vec_[i].num_visible_point3d >= num) {
                next_images.push_back(i);
                registered[i] = true;
            }
        }
        // printf("%d+%d/%d\n", count, next_images.size(), m_matchmap.m_num_frame);
        count += next_images.size();
        if (next_images.empty()) break;
        //  triangulate
        for (const auto &image_id1 : next_images) {
            for (const auto &it : id_pair[image_id1]) {
                int image_id2 = it.first;
                if (!registered[image_id2]) continue;
                auto &image1 = corr_graph.frame_node_vec_[image_id1];
                tri_points(image_id1, image_id2, corr_graph, tri);
            }
        }
    }
}

void MatchExpansionSolver::tri_points(int image_id1, int image_id2, CorrespondenceGraph &corr_graph,
                                      std::vector<std::vector<bool>> &tri) {
    auto &cor_vec = corr_graph.frame_node_vec_[image_id1].corrs_vector;
    for (const auto &cor_ftr : cor_vec) {
        bool find_b = false;
        for (const auto cor : cor_ftr) {
            if (cor.first == image_id2) {
                find_b = true;
            }
        }
        if (find_b) {
            for (const auto cor : cor_ftr) {
                if (!tri[cor.first][cor.second]) {
                    tri[cor.first][cor.second] = true;
                    corr_graph.frame_node_vec_[cor.first].num_visible_point3d++;
                }
            }
        }
    }
}

void GetMayreg(const std::vector<int> &connect_frames, const std::vector<int> &reg_frames,
               const std::vector<std::map<int, int>> &frame2retrievalpairs,
               std::vector<int> &may_reg) {
    size_t num_frame = frame2retrievalpairs.size();
    std::vector<bool> mask_reg(num_frame, false), mask_con(num_frame, false);
    for (const int &id : reg_frames) {
        mask_reg[id] = true;
    }
    for (const int &id : connect_frames) {
        mask_con[id] = true;
    }

    for (size_t id = 0; id < num_frame; ++id) {
        if (mask_con[id]) continue;
        int count1 = 0, count2 = 0, count3 = 0;
        for (auto &[id2, rank] : frame2retrievalpairs[id]) {
            if (mask_reg[id2]) {
                if (rank <= 5) { // 5
                    count1++;
                } else if (rank <= 25) { // 20 25/15
                    count2++;
                } else if (rank <= 50) { // 25 50/35
                    count3++;
                }
            }
        }

        if (count2 >= 15 || count2 + count3 >= 35) {
            may_reg.push_back(id);
        }
    }
}

void MatchExpansionSolver::GetCandidateCovisibility(
    const Map &map, const std::vector<int> &registered_frames,
    const std::vector<int> &connected_frames,
    const std::vector<std::map<int, int>> &retrieval_rank_vec,
    std::vector<std::set<int>> &matched_ids, std::vector<FramePair> &frame_pairs) {
    const size_t num_frame = retrieval_rank_vec.size();
    std::vector<bool> mask_registered(num_frame, false), mask_connected(num_frame, false);
    for (const int &id : registered_frames) {
        mask_registered[id] = true;
    }
    for (const int &id : connected_frames) {
        mask_connected[id] = true;
    }
    m_matchmap.MakeTrack(map);
    m_matchmap.PatchInit(map);
    // printf("Init ok\n");

    // 计算每个frame的关联frame_patch的共视number
    // frame_id => （covisible_num_in_patch)_vec
    std::vector<std::map<int, std::vector<int>>> covisibility_info_vec(num_frame);
    for (size_t i = 0; i < num_frame; ++i) {
        if (mask_connected[i]) covisibility_info_vec[i] = GetCovisibilityInfo(i);
    }

    // select pair
    const auto &id2pair_id = m_matchmap.id2pair_id;
    std::vector<std::vector<bool>> visit(num_frame, std::vector<bool>(num_frame, false));
    for (size_t id1 = 0; id1 < num_frame; ++id1) {
        for (const auto &[id2, rank] : retrieval_rank_vec[id1]) {
            // skip self_match & visited & matched & shared enough matches
            if (id1 == id2) continue;
            if (visit[id1][id2]) continue;
            if (matched_ids[id1].count(id2) != 0) continue;
            // 两个connected组成pair并且中间至少一个registered frame
            if (!mask_registered[id1] && !mask_registered[id2]) continue;
            if (!mask_connected[id1] || !mask_connected[id2]) continue;
            if (m_matchmap.GetMatcheNum(id1, id2) > 50) continue;

            // check covisible
            int num_covisibility_patch = 0;
            auto &set1 = covisibility_info_vec[id1];
            auto &set2 = covisibility_info_vec[id2];
            // frame_id1 -- frame_id -- frame_id2
            for (const auto &[frame_id, patch_state1] : set1) {
                if (set2.count(frame_id) == 0) continue;
                const auto &patch_state2 = set2[frame_id];
                for (int i = 0; i < m_matchmap.patch_num; ++i) {
                    if (patch_state1[i] >= _T_ && patch_state2[i] >= _T_) num_covisibility_patch++;
                }
            }
            if (num_covisibility_patch >= 1) {
                frame_pairs.emplace_back(id1, id2);
                matched_ids[id1].insert(id2);
                matched_ids[id2].insert(id1);
                // update
                // visit[id1][id2] = true;
                // visit[id2][id1] = true;
                // for (auto &[id1_neibor, pair_id] : id2pair_id[id1]) {
                //     visit[id2][id1_neibor] = true;
                //     visit[id1_neibor][id2] = true;
                // }
                // for (auto &[id2_neibor, pair_id] : id2pair_id[id2]) {
                //     visit[id1][id2_neibor] = true;
                //     visit[id2_neibor][id1] = true;
                // }
            }
        }
    }
}

std::map<int, std::vector<int>> MatchExpansionSolver::GetCovisibilityInfo(int frame_id) {
    std::map<int, std::vector<int>> frame_patch_set;
    const auto &id_pair = m_matchmap.id2pair_id;
    for (int track_id : m_matchmap.m_frames_track_ids[frame_id]) {
        if (track_id == -1) continue;
        const auto &track = m_matchmap.m_tracks[track_id];
        if (track.invalid) continue;
        for (const auto &[t_frame_id, pt_id] : track.observations_) {
            if (frame_id == t_frame_id) continue;                   // self
            if (id_pair[frame_id].count(t_frame_id) != 0) continue; // matched
            if (frame_patch_set.count(t_frame_id) == 0)             // first visite
                frame_patch_set[t_frame_id].assign(m_matchmap.patch_num, 0);

            const int patch_id = m_matchmap.pt2patch_id_vec[t_frame_id][pt_id];
            frame_patch_set[t_frame_id][patch_id]++;
        }
    }
    return frame_patch_set;
}
} // namespace xrsfm
