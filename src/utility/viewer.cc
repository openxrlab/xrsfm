//
// Created by SENSETIME\yezhichao1 on 2020/10/19.
//
#include "viewer.h"

#include <pangolin/pangolin.h>
#include <unistd.h>

#include "view.h"
#include "viewer_handle.h"

#define OPEN_VIEW
constexpr int width = 1024, height = 768, focal = 500;
const Eigen::Vector3d red(1.0, 0, 0), green(0, 1.0, 0), blue(0, 0, 1.0), yellow(1, 1, 0), grey(0.5, 0.5, 0.5);

void ViewerThread::start() {
  worker_running = true;
  worker_has_stop = false;
  worker_thread = std::thread(&ViewerThread::worker_loop, this);
}

void ViewerThread::stop() {
  if (worker_running) {
    worker_running = false;
  }
  while (!worker_has_stop) {
    usleep(500);
  }
  worker_thread.join();
}

void ViewerThread::worker_loop() {
#ifdef OPEN_VIEW
  pangolin::CreateWindowAndBind("ECI-SfM", width, height);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(width, height, focal, focal, width / 2, height / 2, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));

  // Create Interactive View in window
  pangolin::MyHandler3D handler(s_cam);
  pangolin::View &d_cam =
      pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f).SetHandler(&handler);

  while (!pangolin::ShouldQuit() & worker_running) {
    if (handler.init_viewpoint) {
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));
      handler.init_viewpoint = false;
    }

    d_cam.Activate(s_cam);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    std::unique_lock lk(map_mutex);
    DrawCameras(cameras, colors_cam, camera_size_);
    DrawPoints(points);

    pangolin::FinishFrame();
    usleep(2000);
  }

  pangolin::DestroyWindow("ECI-SfM");
  pangolin::QuitAll();
  worker_has_stop = true;
#endif
}

void ViewerThread::update_map(const Map &map) {
  std::unique_lock lk(map_mutex);
  cameras.clear();
  colors_cam.clear();
  points.clear();
  colors_pt.clear();

  for (const auto &frame : map.frames_) {
    if (!frame.registered) continue;
    cameras.emplace_back(frame.Tcw);
    if (frame.flag_for_view) {
      colors_cam.emplace_back(red);
    } else if (frame.is_keyframe) {
      colors_cam.emplace_back(green);
    } else {
      colors_cam.emplace_back(grey);
    }
  }

  if (map.tmp_frame.registered) {
    cameras.emplace_back(map.tmp_frame.Tcw);
    colors_cam.emplace_back(yellow);
  }

  for (const auto &track : map.tracks_) {
    if (!track.outlier)  //&& track.is_keypoint) {
      points.emplace_back(track.point3d_);
  }
}

void ViewerThread::update_map_colmap(const Map &map) {
  std::unique_lock lk(map_mutex);
  cameras.clear();
  colors_cam.clear();
  points.clear();
  colors_pt.clear();

  for (const auto &[id,frame] : map.frame_map_) {
    if (!frame.registered) continue;
    cameras.emplace_back(frame.Tcw);
    if (frame.flag_for_view) {
      colors_cam.emplace_back(red);
    } else if (frame.is_keyframe) {
      colors_cam.emplace_back(green);
    } else {
      colors_cam.emplace_back(grey);
    }
  }

  for (const auto &[id,track] : map.track_map_) {
    if (!track.outlier)  //&& track.is_keypoint) {
      points.emplace_back(track.point3d_);
  }
}

void ViewerThread::update_cameras(const std::vector<Pose> &pose_vec) {
  cameras = pose_vec;
  colors_cam.assign(cameras.size(), red);
  int len = cameras.size()/2;
  for(int i = 0;i<len;++i){
    colors_cam[i] = green;
    double dist = (cameras[i].center()-cameras[len+i].center()).norm();
    // if(dist>1.0){
    //   colors_cam[i] = blue;
    // }
  }
}