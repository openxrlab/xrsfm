
#include "geometry/epipolar_geometry.hpp"
#include "geometry/pnp.h"
// #include "optimization/lie_algebra.h"
#include <gtest/gtest.h>

using namespace xrsfm;

TEST(GeometryTest, Fundamental) {
    quaternion q(0.5, 0.5, 0.5, 0.5);
    vector3 t(13, 11, 14);

    std::vector<vector2> points1 = {
        vector2(1, 1), vector2(2, 1), vector2(3, 1), vector2(4, 1), vector2(5, 1),
        vector2(1, 2), vector2(1, 3), vector2(1, 4), vector2(1, 5),
        vector2(-1.23, -3.21), vector2(-3.14, -4.13), vector2(-2.35, -5.23)};
    std::vector<vector2> points2;

    for (int i = 0; i < points1.size(); ++i) {
        vector3 pw = points1[i].homogeneous();
        vector2 pt = (q * pw + t).hnormalized();
        points2.push_back(pt);
    }
    FramePair frame_pair;
    SolveFundamnetalCOLMAP(points1, points2, frame_pair);
    // matrix3 R = q.toRotationMatrix();
    // matrix3 F = R*skewSymmetric(t);
    // std::cout<<frame_pair.F<<std::endl;
    // std::cout<<F<<std::endl;

    // for(int i = 0;i<points1.size();++i){
    //     vector3 pt1 = points1[i].homogeneous();
    //     vector3 pt2 = points2[i].homogeneous();
    //     std::cout<<pt2.dot(F*pt1)<<" "<<pt2.dot(frame_pair.F*pt1)<<std::endl;
    // }
}

TEST(GeometryTest, PnP) {
    quaternion q(0.5, 0.5, 0.5, 0.5);
    vector3 t(13, 11, 14);
    std::vector<vector2> p2d_vec;
    std::vector<vector3> pw_vec = {
        vector3(1, 1, 1), vector3(2, 1, 1), vector3(3, 1, 1), vector3(4, 1, 1), vector3(5, 1, 1),
        vector3(1, 2, 1), vector3(1, 3, 1), vector3(1, 4, 1), vector3(1, 5, 1),
        vector3(-1.23, -3.21, 1), vector3(-3.14, -4.13, 1), vector3(-2.35, -5.23, 1)};
    for (int i = 0; i < pw_vec.size(); ++i) {
        vector3 pc = (q * pw_vec[i] + t);
        p2d_vec.push_back(pc.hnormalized());
    }
    Pose Tcw;
    std::vector<char> inlier_mask;
    SolvePnP_colmap(p2d_vec, pw_vec, 1.0, Tcw,
                    inlier_mask);
    // std::cout << Tcw.q.coeffs().transpose() << " " << Tcw.t.transpose() << std::endl;
    auto dq = Tcw.q * q.inverse();
    ASSERT_LT(abs(dq.w() - 1), 1e-10);
    ASSERT_LT((Tcw.t - t).norm(), 1e-10);
}

// TEST(GeometryTest, Triangulate) {
// }

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    return 0;
}