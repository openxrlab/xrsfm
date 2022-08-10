//
// Created by SENSETIME\yezhichao1 on 2021/1/4.
//

#ifndef XRSFM_SRC_GEOMETRY_MAP_INITIALIZER_H
#define XRSFM_SRC_GEOMETRY_MAP_INITIALIZER_H

#include "base/map.h"

namespace xrsfm {
bool FindInitFramePair(const Map &map, FramePair &init_frame_pair);

void InitializeMap(Map &map, FramePair &frame_pair);

void InitializeWithGT(Map &map, FramePair &frame_pair, Pose pose1, Pose pose2);
} // namespace xrsfm
#endif // XRSFM_SRC_GEOMETRY_MAP_INITIALIZER_H
