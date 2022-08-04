//
// Created by SENSETIME\yezhichao1 on 2020/10/18.
//

#ifndef ECIM_VIEWER_H
#define ECIM_VIEWER_H

#include <pangolin/pangolin.h>

#include <thread>

#include "base/map.h"

namespace xrsfm {
class ViewerThread {
  public:
    void start();
    void worker_loop();
    void stop();

    void update_map(const Map &map);
    void update_map_colmap(const Map &map);
    void add_tag_points(const std::map<int, std::vector<Eigen::Vector3d>> &pt_world_vec);

    double camera_size_ = 0.1;
    std::vector<Pose> cameras;
    std::vector<Eigen::Vector3d> colors_cam;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> colors_pt;

    mutable std::mutex map_mutex;
    std::atomic<bool> worker_running = false;
    std::atomic<bool> worker_has_stop = false;
    std::thread worker_thread;
};
} // namespace xrsfm
#endif // ECIM_VIEWER_H
