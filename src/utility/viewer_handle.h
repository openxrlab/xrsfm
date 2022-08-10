//
// Created by yzc on 19-5-14.
//

#ifndef WSFM_MHANDLER_H
#define WSFM_MHANDLER_H

#include <pangolin/handler/handler.h>

namespace pangolin {
struct MyHandler3D : Handler3D {
    MyHandler3D(OpenGlRenderState &cam_state, AxisDirection enforce_up = AxisNone, float trans_scale = 1.0f,
                float zoom_fraction = 3 * PANGO_DFLT_HANDLER3D_ZF) :
        Handler3D(cam_state, enforce_up, trans_scale, zoom_fraction){};

    MyHandler3D::~MyHandler3D();

    //        void Mouse(View &d, MouseButton button, int x, int y, bool pressed,
    //        int button_state);
    void Keyboard(View &d, unsigned char key, int x, int y, bool pressed);

  public:
    bool draw_current;
    std::atomic<bool> init_viewpoint;
};
} // namespace pangolin

#endif // WSFM_MHANDLER_H
