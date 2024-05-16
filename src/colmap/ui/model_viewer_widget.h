// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#ifndef COLMAP_SRC_UI_MODEL_VIEWER_WIDGET_H_
#define COLMAP_SRC_UI_MODEL_VIEWER_WIDGET_H_

#include <QtCore>
#include <QtOpenGL>

#include <QOpenGLFunctions_3_2_Core>

#include "base/map.h"
#include "ui/point_painter.h"

namespace xrsfm {

class ModelViewerWidget : public QOpenGLWidget,
                          protected QOpenGLFunctions_3_2_Core {
  public:
    const float kInitNearPlane = 1.0f;
    const float kMinNearPlane = 1e-3f;
    const float kMaxNearPlane = 1e5f;
    const float kNearPlaneScaleSpeed = 0.02f;
    const float kFarPlane = 1e5f;
    const float kInitFocusDistance = 100.0f;
    const float kMinFocusDistance = 1e-5f;
    const float kMaxFocusDistance = 1e8f;
    const float kFieldOfView = 25.0f;
    const float kFocusSpeed = 2.0f;
    const float kInitPointSize = 1.0f;
    const float kMinPointSize = 0.5f;
    const float kMaxPointSize = 100.0f;
    const float kPointScaleSpeed = 0.1f;
    const float kInitImageSize = 0.2f;
    const float kMinImageSize = 1e-6f;
    const float kMaxImageSize = 1e3f;
    const float kImageScaleSpeed = 0.1f;
    const int kDoubleClickInterval = 250;

    ModelViewerWidget(QWidget *parent);

    void ChangeFocusDistance(const float delta);
    void ChangeNearPlane(const float delta);
    void ChangePointSize(const float delta);
    void ChangeCameraSize(const float delta);

    void ResetView();
    void RotateView(const float x, const float y, const float prev_x,
                    const float prev_y);
    void TranslateView(const float x, const float y, const float prev_x,
                       const float prev_y);

    //   void ReloadReconstruction();
    //   void ClearReconstruction();
    //   void SelectObject(const int x, const int y);
    //   void SelectMoviewGrabberView(const size_t view_idx);

    //   void ShowPointInfo(const point3D_t point3D_id);
    //   void ShowImageInfo(const image_t image_id);

    Map *map = nullptr;

    //   QLabel *statusbar_status_label;

    // protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;

    void Upload();

  private:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

    void SetupPainters();
    void SetupView();

    void UploadCoordinateGridData();
    void UploadPointData(const bool selection_mode = false);
    //   void UploadPointConnectionData();
    void UploadImageData(const bool selection_mode = false);
    //   void UploadImageConnectionData();
    //   void UploadMovieGrabberData();

    void ComposeProjectionMatrix();

    float ZoomScale() const;
    float AspectRatio() const;
    //   float OrthographicWindowExtent() const;

    Eigen::Vector3f PositionToArcballVector(const float x, const float y) const;

    QMatrix4x4 model_view_matrix_;
    QMatrix4x4 projection_matrix_;

    PointPainter point_painter_;
    LinePainter image_line_painter_;
    TrianglePainter image_triangle_painter_;

    bool mouse_is_pressed_;
    QTimer mouse_press_timer_;
    QPoint prev_mouse_pos_;

    float focus_distance_;

    std::vector<std::pair<size_t, char>> selection_buffer_;
    int selected_image_id_;
    long int selected_point3D_id_;
    size_t selected_movie_grabber_view_;

    bool coordinate_grid_enabled_;

    float point_size_;
    float image_size_;
    float near_plane_;

    float background_color_[3];

    QOpenGLShaderProgram *shaderProgram;
};

} // namespace xrsfm

#endif // COLMAP_SRC_UI_MODEL_VIEWER_WIDGET_H_
