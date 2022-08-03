//
// Created by SENSETIME\yezhichao1 on 2021/1/4.
//

#ifndef ECIM_ECIM_MAP_INITIALIZER_H_
#define ECIM_ECIM_MAP_INITIALIZER_H_

#include "base/map.h"

namespace xrsfm{
bool FindInitFramePair(const Map &map, FramePair &init_frame_pair);

void InitializeMap(Map &map, FramePair &frame_pair);

void InitializeWithGT(Map &map, FramePair &frame_pair, Pose pose1, Pose pose2);
}
#endif  // ECIM_ECIM_MAP_INITIALIZER_H_
