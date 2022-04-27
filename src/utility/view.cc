//
// Created by SENSETIME\yezhichao1 on 2020/10/19.
//
#include "view.h"

#include <pangolin/pangolin.h>
#include <unistd.h>

#include "viewer_handle.h"

#define OPEN_VIEW
constexpr int width = 1024, height = 768, focal = 500;
const cv::Scalar cv_green(0, 255, 0), cv_gray(80, 80, 80), cv_blue(255, 0, 0), cv_yellow(0, 255, 255),
    cv_red(0, 0, 255), cv_brown(40, 140, 140);

void DrawMatchesColmap(const std::string dir_path, const FramePair &fp, const Map &map) {
  auto &f1 = map.frames_[fp.id1];
  auto &f2 = map.frames_[fp.id2];
  cv::Mat image1 = cv::imread(dir_path + f1.name);
  cv::Mat image2 = cv::imread(dir_path + f2.name);
  DrawFeatureMatches(image1, image2, f1.points, f2.points, fp.matches);
  cv::waitKey();
}

void DrawFrameMatches(const Map &map, const std::vector<cv::Mat> &images, int id) {
  for (auto &t_frame_pair : map.frame_pairs_) {
    if (t_frame_pair.id1 == id || t_frame_pair.id2 == id) {
      int id1 = t_frame_pair.id1;
      int id2 = t_frame_pair.id2;
      printf("Find Pair: %d %d %d\n", id1, id2, t_frame_pair.inlier_num);
      DrawFeatureMatches(images[id1], images[id2], map.frames_[id1].points, map.frames_[id2].points,
                         t_frame_pair.matches, t_frame_pair.inlier_mask);
      DrawFeatureMatches1(images[id1], map.frames_[id1].points, map.frames_[id2].points, t_frame_pair.matches,
                          t_frame_pair.inlier_mask);
    }
  }
}

void DrawFeature(const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints) {
  for (const auto &kpt : keypoints) {
    cv::circle(image, kpt.pt, 1, cv::Scalar(0, 255, 0), -1);
  }
  cv::imshow("", image);
  cv::waitKey(0);
}

void DrawFeatureMatches(const cv::Mat &img1, const cv::Mat &img2, const std::vector<cv::KeyPoint> &kpts1,
                        const std::vector<cv::KeyPoint> &kpts2, const std::vector<Match> &matches,
                        const std::vector<char> mask) {
  int rows = img1.rows > img2.rows ? img1.rows : img2.rows;
  cv::Mat img(rows, img1.cols + img2.cols, img1.type(), cv::Scalar(0, 0, 0));
  cv::Mat img1p = img(cv::Rect(0, 0, img1.cols, img1.rows));
  cv::Mat img2p = img(cv::Rect(img1.cols, 0, img2.cols, img2.rows));
  img1.copyTo(img1p);
  img2.copyTo(img2p);
  cv::Scalar color;
  for (size_t i = 0; i < matches.size(); ++i) {
    auto match = matches[i];
    color = mask.empty() || mask[i] ? cv_green : cv_red;
    if (!mask.empty() && !mask[i])
      line(img, kpts1[match.id1].pt, kpts2[match.id2].pt + cv::Point2f(img1.cols, 0), color, 1);
  }
  constexpr double scale = 1.0;
  resize(img, img, cv::Size(int(img.cols * scale), int(img.rows * scale))); /**/
  imshow("0", img);
}

void DrawFeatureMatches(const cv::Mat &img1, const cv::Mat &img2, const std::vector<Eigen::Vector2d> &kpts1,
                        const std::vector<Eigen::Vector2d> &kpts2, const std::vector<Match> &matches,
                        const std::vector<char> mask) {
  std::vector<cv::KeyPoint> kpts1_;
  for (auto &pt : kpts1) {
    cv::KeyPoint kpt;
    kpt.pt = cv::Point2f(pt.x(), pt.y());
    kpts1_.emplace_back(kpt);
  }
  std::vector<cv::KeyPoint> kpts2_;
  for (auto &pt : kpts2) {
    cv::KeyPoint kpt;
    kpt.pt = cv::Point2f(pt.x(), pt.y());
    kpts2_.emplace_back(kpt);
  }
  DrawFeatureMatches(img1, img2, kpts1_, kpts2_, matches, mask);
}

void DrawFeatureFlow(const cv::Mat &img1, const std::vector<Eigen::Vector2d> &pts1,
                     const std::vector<Eigen::Vector2d> &pts2, const std::vector<Match> &matches,
                     const std::vector<int> &states, const std::string window_name) {
  std::vector<cv::KeyPoint> kpts1;
  for (auto &pt : pts1) {
    kpts1.emplace_back(cv::KeyPoint(pt.x(), pt.y(), -1));
  }
  std::vector<cv::KeyPoint> kpts2;
  for (auto &pt : pts2) {
    ;
    kpts2.emplace_back(cv::KeyPoint(pt.x(), pt.y(), -1));
  }

  cv::Mat img = img1;

  for (size_t i = 0; i < matches.size(); ++i) {
    auto match = matches[i];
    cv::Scalar color, dst_color, src_color = cv_yellow;
    if (states[i] == 1)
      color = dst_color = cv_green;
    else if (states[i] == 0)
      color = dst_color = cv_red;
    else if (states[i] == -1) {
      color = dst_color = cv_brown;
    } else {
      std::cerr << "unexpected state";
    }
    cv::circle(img, kpts1[match.id1].pt, 2, src_color);
    line(img, kpts1[match.id1].pt, kpts2[match.id2].pt, color, 1);
    cv::circle(img, kpts2[match.id2].pt, 2, dst_color);
  }

  constexpr double scale = 1.0;
  resize(img, img, cv::Size(int(img.cols * scale), int(img.rows * scale)));
  imshow(window_name, img);
}

void DrawFeatureMatches1(const cv::Mat &img1, const std::vector<cv::KeyPoint> &kpts1,
                         const std::vector<cv::KeyPoint> &kpts2, const std::vector<Match> &matches,
                         const std::vector<char> mask) {
  cv::Mat img = img1.clone();
  cv::Scalar color;

  for (size_t i = 0; i < matches.size(); ++i) {
    auto match = matches[i];
    color = mask.empty() || mask[i] ? cv_green : cv_red;
    cv::circle(img, kpts1[match.id1].pt, 2, cv_blue);
    line(img, kpts1[match.id1].pt, kpts2[match.id2].pt, color, 1);
    cv::circle(img, kpts2[match.id2].pt, 2, cv_yellow);
  }

  constexpr double scale = 1.0;
  resize(img, img, cv::Size(int(img.cols * scale), int(img.rows * scale)));
  imshow("1", img);
}

void DrawFeatureMatches1(const cv::Mat &img1, const std::vector<Eigen::Vector2d> &kpts1,
                         const std::vector<Eigen::Vector2d> &kpts2, const std::vector<Match> &matches,
                         const std::vector<char> mask) {
  std::vector<cv::KeyPoint> kpts1_;
  for (auto &pt : kpts1) {
    cv::KeyPoint kpt;
    kpt.pt = cv::Point2f(pt.x(), pt.y());
    kpts1_.emplace_back(kpt);
  }
  std::vector<cv::KeyPoint> kpts2_;
  for (auto &pt : kpts2) {
    cv::KeyPoint kpt;
    kpt.pt = cv::Point2f(pt.x(), pt.y());
    kpts2_.emplace_back(kpt);
  }
  DrawFeatureMatches1(img1, kpts1_, kpts2_, matches, mask);
}

void DrawCameras(const std::vector<Pose> &cameras, const std::vector<Eigen::Vector3d> colors, double camera_size) {
  const float w = camera_size;
  const float h = w * 0.5f;
  const float z = w * 0.6f;
  const int num_camera = cameras.size();
  for (int i = 0; i < num_camera; ++i) {
    const auto &camera = cameras[i];
    glPushMatrix();
    const Eigen::Matrix3d R = camera.q.toRotationMatrix();
    const Eigen::Vector3d t = camera.t;
    Eigen::Matrix4d Tcw;
    Tcw << R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2), t(1), R(2, 0), R(2, 1), R(2, 2), t(2), 0, 0, 0,
        1;
    Eigen::Matrix4d Twc = Tcw.inverse();
    glMultMatrixd(Twc.data());
    glColor3d(colors[i](0), colors[i](1), colors[i](2));

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);
    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);
    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);

    //        glColor3f(1,0,0);
    //        glVertex3f(0,0,0);
    //        glVertex3f(2*w,0,0);
    //
    //        glColor3f(0,1,0);
    //        glVertex3f(0,0,0);
    //        glVertex3f(0,2*w,0);
    //
    //        glColor3f(0,0,1);
    //        glVertex3f(0,0,0);
    //        glVertex3f(0,0,2*w);

    glEnd();
    glPopMatrix();
  }
}

void DrawPoints(const std::vector<Eigen::Vector3d> &points) {
  glPointSize(1.0f);
  glBegin(GL_POINTS);
  for (const auto &pt : points) {
    glColor3f(0.0f, 0.0f, 0.0f);
    glVertex3d(pt(0), pt(1), pt(2));
  }
  glEnd();
}

void DrawPoints(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> colors) {
  glPointSize(1.0f);
  glBegin(GL_POINTS);
  for (int i = 0; i < points.size(); ++i) {
    glColor3d(colors[i](0), colors[i](1), colors[i](2));
    glVertex3d(points[i](0), points[i](1), points[i](2));
  }
  glEnd();
}

#include <thread>

void Viewer::Init() {
#ifdef OPEN_VIEW
  pangolin::CreateWindowAndBind("SfM: Map Viewer", width, height);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  s_cam = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(width, height, focal, focal, width / 2, height / 2, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));

  handler = std::make_unique<pangolin::MyHandler3D>(s_cam);

  d_cam = pangolin::CreateDisplay().SetBounds(0, 1.0, 0, 1.0, -1024.0f / 768.0f).SetHandler(handler.get());
#endif
}

void Viewer::Draw(const Map &map, bool stop) {
#ifdef OPEN_VIEW
  std::vector<Pose> cameras;
  std::vector<Eigen::Vector3d> colors;
  std::vector<Eigen::Vector3d> points;

  int camera_id = -1;
  for (const auto &frame : map.frames_) {
    if (frame.registered) {
      cameras.emplace_back(frame.Tcw);
    }
  }

  size_t num_camera = cameras.size();
  if (color_in_order_) {
    for (int i = 0; i < num_camera; ++i) {
      colors.emplace_back(1.0 * i / num_camera, 1.0 * (num_camera - i) / num_camera, 0);
    }
  } else {
    colors.assign(num_camera, Eigen::Vector3d(0, 1.0, 0));
  }

  int id_count = 0;
  for (const auto &frame : map.frames_) {
    if (frame.registered) {
      if (frame.id == map.current_frame_id_) {
        colors[id_count] = Eigen::Vector3d(0, 1.0, 0);
      } else if (frame.flag_for_view) {
        colors[id_count] = Eigen::Vector3d(1.0, 0, 0);
      } else {
        colors[id_count] = Eigen::Vector3d(0, 0, 1.0);
      }
      id_count++;
    }
  }

  if (map.tmp_frame.registered) {
    cameras.emplace_back(map.tmp_frame.Tcw);
    colors.emplace_back(Eigen::Vector3d(1.0, 1.0, 0));
  }

  for (const auto &track : map.tracks_) {
    if (track.outlier) continue;
    points.emplace_back(track.point3d_);
  }

  if (stop) {
    handler->draw_current = true;
    while (handler->draw_current) {
      if (handler->init_viewpoint) {
        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));
        handler->init_viewpoint = false;
      }

      d_cam.Activate(s_cam);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

      DrawCameras(cameras, colors, camera_size_);
      DrawPoints(points);
      pangolin::FinishFrame();
      usleep(500);
    }
  } else {
    if (handler->init_viewpoint) {
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));
      handler->init_viewpoint = false;
    }

    d_cam.Activate(s_cam);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    DrawCameras(cameras, colors, camera_size_);
    DrawPoints(points);
    pangolin::FinishFrame();
  }
#endif
}

void Viewer::DrawKey(const Map &map, bool stop) {
  if (handler->init_viewpoint) {
    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));
    handler->init_viewpoint = false;
  }

  std::vector<Pose> cameras;
  std::vector<Eigen::Vector3d> colors;
  std::vector<Eigen::Vector3d> points;
  for (const auto &frame : map.frames_) {
    if (frame.registered && frame.is_keyframe) {
      cameras.emplace_back(frame.Tcw);
    }
  }
  size_t num_camera = cameras.size();
  if (color_in_order_) {
    for (int i = 0; i < num_camera; ++i) {
      colors.emplace_back(1.0 * i / num_camera, 1.0 * (num_camera - i) / num_camera, 0);
    }
  } else {
    colors.assign(num_camera, Eigen::Vector3d(0, 1.0, 0));
  }
  for (const auto &track : map.tracks_) {
    if (!track.outlier && track.is_keypoint) {
      points.emplace_back(track.point3d_);
    }
  }
  printf("Viewer: %zu %zu\n", cameras.size(), points.size());
  if (stop) {
    handler->draw_current = true;
    while (handler->draw_current) {
      d_cam.Activate(s_cam);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

      DrawCameras(cameras, colors, camera_size_);
      DrawPoints(points);
      pangolin::FinishFrame();
      usleep(500);
    }
  } else {
    d_cam.Activate(s_cam);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    DrawCameras(cameras, colors, camera_size_);
    DrawPoints(points);
    pangolin::FinishFrame();
  }
}

void Viewer::DrawLoop(const Map &map, const LoopInfo loop_info) {
  std::vector<Pose> cameras;
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> colors;
  for (const auto &frame : map.frames_) {
    if (frame.registered) {
      cameras.emplace_back(frame.Tcw);
      if (frame.id == loop_info.frame_id) {
        colors.emplace_back(1, 0, 0);
      } else if (loop_info.cor_frame_ids_vec[0].count(frame.id) == 1) {
        colors.emplace_back(0, 0, 1);
      } else if (loop_info.cor_frame_ids_vec[1].count(frame.id) == 1) {
        colors.emplace_back(0.5, 0.5, 0);
      } else {
        colors.emplace_back(0, 1, 0);
      }
    }
  }
  for (const auto &track : map.tracks_) {
    if (track.outlier) continue;
    points.emplace_back(track.point3d_);
  }

  handler->draw_current = true;
  while (handler->draw_current) {
    if (handler->init_viewpoint) {
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));
      handler->init_viewpoint = false;
    }

    d_cam.Activate(s_cam);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    DrawCameras(cameras, colors, camera_size_);
    DrawPoints(points);
    pangolin::FinishFrame();
    usleep(500);
  }
}

void Viewer::DrawGraph(const Map &map, bool stop) {
#ifdef OPEN_VIEW
  if (handler->init_viewpoint) {
    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));
    handler->init_viewpoint = false;
  }

  std::vector<Pose> cameras;
  std::vector<Eigen::Vector3d> colors;
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> colors_pt;

  int max_level = -1;
  for (const auto &frame : map.frames_) {
    if (!frame.registered) continue;
    if (max_level < frame.hierarchical_level) {
      max_level = frame.hierarchical_level;
    }
  }

  int num_key = 0;
  for (const auto &frame : map.frames_) {
    if (!frame.registered) continue;

    double c = 1.0 * frame.hierarchical_level / max_level;
    if (frame.hierarchical_level == -1) {
      std::cout << "here\n";
      c == 0.0;
    } 
    int count = 0;
    for(auto&id:frame.track_ids_){
      if(id!=-1)count++;
    }
    // std::cout<<frame.id<<" "<<count<<" "<<frame.num_visible_points3D_<<std::endl;
    if(count<=50){ 
      // c = 0;
      std::cout<<frame.id<<" "<<frame.num_visible_points3D_<<std::endl;
    }else{
      cameras.emplace_back(frame.Tcw);
      colors.emplace_back(c, 1 - c, 0);
    }
    // if (frame.is_keyframe) {
    //   colors.emplace_back(1.0, 0, 0);
    // } else {
    //   colors.emplace_back(0, 1.0, 0);
    // }

    if (frame.is_keyframe) num_key++;
  }

  std::cout << "max level " << max_level << std::endl;
  std::cout << "keyframe num " << num_key << std::endl;

  for (const auto &track : map.tracks_) {
    if (!track.outlier && track.is_keypoint) {
      points.emplace_back(track.point3d_);
      // if (track.hierarchical_level == -1) std::cout << "here\n" ;
      // double c = 1.0 * track.hierarchical_level / max_distance;
      // colors_pt.emplace_back(c, 1 - c, 0);
    }
  }
  printf("Viewer: %zu %zu\n", cameras.size(), points.size());

  if (stop) {
    handler->draw_current = true;
    while (handler->draw_current) {
      d_cam.Activate(s_cam);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

      DrawCameras(cameras, colors, camera_size_);
      DrawPoints(points);
      pangolin::FinishFrame();
      usleep(500);
    }
  } else {
    d_cam.Activate(s_cam);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    DrawCameras(cameras, colors, camera_size_);
    DrawPoints(points);
    pangolin::FinishFrame();
  }
#endif
}

void Viewer::DrawColor(const Map &map, bool stop) {
#ifdef OPEN_VIEW
  if (handler->init_viewpoint) {
    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, -10, 0, 0, 0, pangolin::AxisNegY));
    handler->init_viewpoint = false;
  }

  std::vector<Pose> cameras;
  std::vector<Eigen::Vector3d> colors;
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> colors_pt;

  Eigen::Vector3d red(1.0, 0, 0), green(0, 1.0, 0), blue(0, 0, 1.0), yellow(1, 1, 0), grey(0.5, 0.5, 0.5),
      gb(100. / 255, 202. / 255, 249. / 255), gg(129. / 255, 200. / 255, 132. / 255),
      gr(240. / 255, 80. / 255, 80. / 255);

  auto in_range = [=](const int x, int min, int max) { return x <= max && x >= min; };
  for (const auto &frame : map.frames_) {
    if (!frame.registered) continue;
    cameras.emplace_back(frame.Tcw);
    bool is_seq = frame.camera_id == map.cameras_.back().id;
    if (is_seq) {
      if (in_range(frame.id, 0, 658)) {
        colors.emplace_back(gg);
      } else if (in_range(frame.id, 659, 2099)) {
        colors.emplace_back(gb);
      } else if (in_range(frame.id, 2100, 2666)) {
        colors.emplace_back(yellow);
      } else {
        colors.emplace_back(grey);
      }
    } else {
      colors.emplace_back(gr);
    }
  }

  for (const auto &track : map.tracks_) {
    if (!track.outlier && track.is_keypoint) {
      points.emplace_back(track.point3d_);
    }
  }
  printf("Viewer: %zu %zu\n", cameras.size(), points.size());

  handler->draw_current = true;
  while (handler->draw_current) {
    d_cam.Activate(s_cam);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    DrawCameras(cameras, colors, camera_size_);
    DrawPoints(points);
    pangolin::FinishFrame();
    usleep(500);
  }
#endif
}
