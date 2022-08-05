
#include <iostream>

#include "opencv2/opencv.hpp"
#include <experimental/filesystem>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

int main(int argc, char *argv[]) {
    std::string image_dir = argv[1];
    std::vector<std::string> image_vec;
    for (const auto &fe : std::experimental::filesystem::directory_iterator(image_dir)) {
        image_vec.emplace_back(fe.path().filename());
    }

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    std::sort(image_vec.begin(), image_vec.end());
    cv::Mat frame, gray;
    for (auto &image_name : image_vec) {
        std::string img_path = image_dir + image_name;
        frame = cv::imread(img_path.c_str());
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = {.width = gray.cols,
                         .height = gray.rows,
                         .stride = gray.cols,
                         .buf = gray.data};

        zarray_t *detections = apriltag_detector_detect(td, &im);
        const int num_detect = zarray_size(detections);
        if (num_detect == 0) continue;
        // std::cout<<img_path<<" "<<num_detect<<std::endl;
        std::cout << image_name << " " << num_detect << std::endl;
        for (int i = 0; i < num_detect; i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            // Do stuff with detections here.
            std::vector<cv::Point2f> p_vec(4);
            for (int k = 0; k < 4; ++k)
                p_vec[k] = cv::Point2f(det->p[k][0], det->p[k][1]);
            for (int k = 0; k < 4; ++k) {
                cv::line(frame, p_vec[k], p_vec[(k + 1) % 4], cv::Scalar(255, 0, 0));
                cv::circle(frame, p_vec[k], 2, cv::Scalar(255 * k / 4, 255 * (4 - k) / 4, 0), -1);
            }
            std::cout << det->id;
            for (int k = 0; k < 4; ++k)
                std::cout << " " << det->p[k][0] << " " << det->p[k][1];
            std::cout << std::endl;
        }
        // cv::imshow("",frame);
        // cv::waitKey(0);
    }

    // Cleanup.
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);

    return 0;
}
