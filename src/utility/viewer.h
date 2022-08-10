//
// Created by SENSETIME\yezhichao1 on 2020/10/18.
//

#ifndef ECIM_VIEWER_H
#define ECIM_VIEWER_H

#include <pangolin/pangolin.h>

#include <thread>

#include "base/map.h"
#include "utility/global.h"

namespace xrsfm {
class ViewerThread {
  public:
    void start();
    void worker_loop();
    void stop();

    void update_map(const Map &map);
    void update_map_colmap(const Map &map);
    void add_tag_points(const std::map<int, std::vector<vector3>> &pt_world_vec);

    double camera_size_ = 0.1;
    std::vector<Pose> cameras;
    std::vector<vector3> colors_cam;
    std::vector<vector3> points;
    std::vector<vector3> colors_pt;

    mutable std::mutex map_mutex;
    std::atomic<bool> worker_running;
    std::atomic<bool> worker_has_stop;
    std::thread worker_thread;
};
} // namespace xrsfm
#endif // ECIM_VIEWER_H
