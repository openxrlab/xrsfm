#pragma once
#include <fstream>

#include "base/map.h"
#include <glog/logging.h>

const int _T_ = 2;
const int _Np_ = 10;

class TrackExpansionWrapper : public Track {
 public:
  bool invalid;
};

class MatchMap {
 public:
  MatchMap() {
    m_track_num = 0;
    col_num = row_num = _Np_;
    patch_num = col_num * row_num;
  }

  void PatchInit(const Map &map);

  void AddTrack(int frame_id1, int frame_id2, const Match &match, const size_t num_keypoint1,
                const size_t num_keypoints2);

  void MakeTrack(const Map &map);

  void MakeTrack(const std::vector<Frame> &frames);

  std::vector<std::map<int, int>> MakeIdPair();  // map::

  int GetMatches(int frame_id1, int frame_id2, std::vector<Match> &matches);

  int GetMatcheNum(int frame_id1, int frame_id2);

  // map::
  static void LoadRetrievalRank(const std::string &filename,
                                const std::map<std::string, int> &name_map,
                                std::map<int, std::vector<int>> &retrieval_rank_of_frames,
                                bool skip_matches = true);

  static void LoadMatch(const Map &map, std::vector<FramePair> &frame_pairs);

  void MakeCorrGraph(const Map &map);

  void SetUpImageSize(const std::vector<ImageSize> &images) {
    image_size.assign(images.begin(), images.end());
  }

  void SetUpFrameSize(size_t num_frames) {
    m_num_frame = num_frames;
    assert(m_num_frame == image_size.size());
  }

  void PrintIdPair();
  void PrintFramesTrack();
  void PrintPatchIdOfFeatures();
  void PrintFeaturesOfPatch();
  void PrintImageSize();

  std::vector<ImageSize> image_size;

  std::vector<FramePair> m_frame_pairs;
  size_t m_num_frame;
  CorrespondenceGraph corr_graph;
  std::vector<std::vector<int>> m_frames_track_ids;
  std::vector<TrackExpansionWrapper> m_tracks;
  size_t m_track_num;

  struct PatchTrack {
    std::map<int, int> m_observations;
    Eigen::Vector3d m_point3d;
    bool invalid = false;
  };

  int col_num;
  int row_num;
  int patch_num;
  int m_patch_track_num;
  std::vector<PatchTrack> m_patch_tracks;
  std::vector<std::vector<int>> pt2patch_id_vec;
  std::vector<std::vector<std::vector<int>>> m_patch_kpt_ids;
  std::vector<std::vector<int>> m_frames_patch_track_ids;
  std::vector<std::map<int, int>> id2pair_id;

  bool verbose;
};

class MatchExpansionSolver {
 public:
  void SetUp(const Map &map, const std::map<int, std::vector<int>> &retrieval_rank_of_frames,
             const std::vector<ImageSize> &images, const size_t &initial_frame1,
             const size_t &initial_frame2);

  void Run(const Map &map, std::vector<std::pair<int, int>> &image_pairs);

  static void CheckConsistenceOfFrameIdAndIndex(const Map &map);

  std::vector<std::set<int>> id2matched_ids;

 private:
  void Print(const std::vector<int> &x_set);

  void PrintRetrievalMap();

  std::vector<std::map<int, int>> GetId2RankVec(
      const std::map<int, std::vector<int>> &retrieval_rank_of_frames);

  void GetConnectedFrames(const Map &map, std::vector<int> &set);

  int GetInitFramePairId(const std::vector<std::map<int, int>> &id_pair);

  void GetPotentialRegisteredFrames(const Map &map, int num, std::vector<int> &set);

  void SimulationSfM(const Map &map, int num, std::vector<std::vector<bool>> &tri);

  void tri_points(int image_id1, int image_id2, CorrespondenceGraph &corr_graph,
                  std::vector<std::vector<bool>> &tri);

  void GetCandidateCovisibility(const Map &map, const std::vector<int> &can_build,
                                const std::vector<int> &setcc,
                                const std::vector<std::map<int, int>> &cor_retrival,
                                std::vector<std::set<int>> &pair_list,
                                std::vector<FramePair> &frame_pairs);

  std::map<int, std::vector<int>> GetCovisibilityInfo(int frame_id);

  size_t m_initial_frame1, m_initial_frame2;
  std::vector<std::map<int, int>> id2rank_vec;  // retrivalframepairs_map[frame_i][frame_j] = the
                                                // rank of frame_j in candidates of frame_i
  MatchMap m_matchmap;

  const bool verbose = false;
};