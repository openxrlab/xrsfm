//
// Created by SENSETIME\yezhichao1 on 2020/10/26.
//

#ifndef ECIM_POINT3DPROCESSOR_H
#define ECIM_POINT3DPROCESSOR_H

#include "base/map.h"

class Point3dProcessor {
 public:
  Point3dProcessor(){};
  Point3dProcessor(double trl, double tal, double trg, double tag)
      : th_rpe_lba_(trl), th_angle_lba_(tal), th_rpe_gba_(trg), th_angle_gba_(tag){};

  bool log_state = false;
  double th_rpe_lba_ = 8, th_angle_lba_ = 2.0;
  double th_rpe_gba_ = 8, th_angle_gba_ = 2.0;

  void ReTriangulate(Map &map);
  void UpdateTrackInfo(Map &map);

  void CheckTrackDepth(const Map &map);
  bool CheckFrameMeasurement(Map &map, int frame_id);
  void CheckFramesMeasurement(Map &map, double th_rpe_lba, double th_angle_lba);

  void MergeTracks(Map &map, const int frame_id, double max_re);
  void ContinueFrameTracks(const int frame_id, const std::vector<std::pair<int, int>> &cor_2d_3d_ids, Map &map);

  int TriangulateFramePoint(Map &map, const int frame_id, const double deg_th);
  
  int FilterPoints3d(Map &map, double max_re, double deg);
  int FilterPointsFrame(Map &map, const int frame_id, const double max_re, const double deg);
};

#endif  // ECIM_POINT3DPROCESSOR_H