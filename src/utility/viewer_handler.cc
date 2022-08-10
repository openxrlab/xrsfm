//
// Created by yzc on 19-5-14.
//

#include "viewer_handle.h"

namespace pangolin {
MyHandler3D::~MyHandler3D() {
    init_viewpoint = false;
}
void MyHandler3D::Keyboard(View &d, unsigned char key, int x, int y, bool pressed) {
    if (key == 'n' && pressed) {
        draw_current = false;
    } else if (key == 'o' && pressed) {
        init_viewpoint = true;
    }
}

} // namespace pangolin