#include <opencv2/opencv.hpp>
#include <typeinfo>
#include <iostream>
#include <fstream>

int main(int argc, char* argv[]) {
    std::string bin_path, output_dir_path;
    if (argc == 3) {
        bin_path = argv[1];
        output_dir_path = argv[2];
    } else {
        exit(-1);
    }

    std::vector<float> buffer(4);
    std::ifstream inFile(bin_path, std::ios::in | std::ios::binary);
    if (!inFile) {
        std::cout << "error\n";
        return 0;
    }

    std::ofstream outFile(output_dir_path + "/camera.txt", std::ios::out);
    double timestamp;
    std::cout << "Wait a sec...\n";
    while (inFile.read(reinterpret_cast<char*>(&timestamp), sizeof(timestamp))) {
        outFile << std::to_string(timestamp);
        outFile << " simple_radio";
        float matrix;
        inFile.read(reinterpret_cast<char*>(buffer.data()), sizeof(matrix) * 4);
        for (auto i : buffer) {
            outFile << " ";
            outFile << std::to_string(i);
        }
        outFile << " 0\n";
        uint32_t width, height;
        inFile.read(reinterpret_cast<char*>(&width), sizeof(width));
        inFile.read(reinterpret_cast<char*>(&height), sizeof(height));
        char* imageBuffer = new char[(width)*height];
        inFile.read(reinterpret_cast<char*>(imageBuffer), (width)*height * sizeof(u_int8_t));
        std::string imgName = output_dir_path + "/images/" + std::to_string(timestamp) + ".png";
        cv::Mat image = cv::Mat(cv::Size(width, height), CV_8UC1, imageBuffer);
        cv::Mat cropped_image = image(cv::Range(0, height), cv::Range(0, width));
        cv::rotate(cropped_image, cropped_image, cv::ROTATE_90_CLOCKWISE);
        cv::imwrite(imgName, cropped_image);
    }
    inFile.close();
    outFile.close();
    std::cout << "Done\n";
}