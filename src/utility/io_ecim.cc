//
// Created by SENSETIME\yezhichao1 on 2020/4/5.
//

#include "io_ecim.hpp"

namespace xrsfm {

void ReadCamerasBinary(const std::string &path, std::vector<Camera> &cameras) {
    std::ifstream file(path, std::ios::binary);
    CHECK(file.is_open()) << path;
    const uint64_t num_camera = read_data2<uint64_t>(file);
    cameras.resize(num_camera);
    for (auto &camera : cameras) {
        int camera_model = 2;
        uint64_t w = 2 * camera.cx(), h = 2 * camera.cy();
        read_data(file, camera.id_);
        read_data(file, camera_model);
        read_data(file, w);
        read_data(file, h);
        std::array<double, 4> param;
        read_data_vec(file, param.data(), 4);
        camera.params_ = std::vector<double>(param.begin(), param.end());
    }
}

void ReadImagesBinary(const std::string &path, std::map<int, Frame> &frames) {
    std::ifstream file(path, std::ios::binary);
    CHECK(file.is_open()) << path;
    const uint64_t num_frame = read_data2<uint64_t>(file);

    for (uint64_t i = 0; i < num_frame; ++i) {
        Frame frame;
        frame.id = read_data2<uint32_t>(file);
        Eigen::Vector4d q_vec;
        read_data(file, q_vec);
        frame.Tcw.q =
            Eigen::Quaterniond(q_vec[0], q_vec[1], q_vec[2], q_vec[3]);
        read_data(file, frame.Tcw.t);
        frame.camera_id = read_data2<uint32_t>(file);
        read_name(file, frame.name);

        uint64_t num_p2d = read_data2<uint64_t>(file);
        frame.points.resize(num_p2d);
        frame.points_normalized.resize(num_p2d);
        frame.track_ids_.assign(num_p2d, -1);
        for (size_t i = 0; i < num_p2d; ++i) {
            auto &p2d = frame.points[i];
            read_data(file, p2d);
            frame.track_ids_[i] = read_data2<uint64_t>(file);
        }
        frame.registered = true;
        frames[frame.id] = frame;
    }
}

void ReadPoints3DBinary(const std::string &path, std::map<int, Track> &tracks) {
    std::ifstream file(path, std::ios::in | std::ios::binary);
    CHECK(file.is_open()) << path;

    tracks.clear();
    uint64_t num_track = -1;
    read_data(file, num_track);
    for (uint64_t i = 0; i < num_track; ++i) {
        Track track;
        uint64_t id = read_data2<uint64_t>(file);
        read_data(file, track.point3d_);
        std::array<uint8_t, 3> color = {0, 0, 0};
        read_data_vec(file, color.data(), 3);
        read_data(file, track.error);
        uint64_t num_obs = -1;
        read_data(file, num_obs);
        int frame_id = -1, p2d_id = -1;
        for (int t = 0; t < num_obs; ++t) {
            read_data(file, frame_id);
            read_data(file, p2d_id);
            track.observations_[frame_id] = p2d_id;
        }
        tracks[id] = track;
    }
}

void ReadColMapDataBinary(const std::string &output_path, Map &map) {
    ReadCamerasBinary(output_path + "cameras.bin", map.cameras_);
    ReadImagesBinary(output_path + "images.bin", map.frame_map_);
    ReadPoints3DBinary(output_path + "points3D.bin", map.track_map_);
}

void ReadImagesBinaryForTriangulation(const std::string &path,
                                      std::map<int, Frame> &frames) {
    std::ifstream file(path, std::ios::binary);
    CHECK(file.is_open()) << path;
    const uint64_t num_frame = read_data2<uint64_t>(file);
    const uint32_t feature_dim = read_data2<uint32_t>(file);
    for (uint64_t i = 0; i < num_frame; ++i) {
        Frame frame;
        frame.id = read_data2<uint32_t>(file);
        read_name(file, frame.name);
        const int width = read_data2<uint64_t>(file);
        const int height = read_data2<uint64_t>(file);

        uint64_t num_p2d = read_data2<uint64_t>(file);
        frame.keypoints_.resize(num_p2d);
        for (size_t i = 0; i < num_p2d; ++i) {
            auto &pt = frame.keypoints_[i].pt;
            pt.x = read_data2<double>(file);
            pt.y = read_data2<double>(file);
        }
        for (size_t i = 0; i < num_p2d; ++i) {
            // Eigen::Vector<float, 256> desc;
            std::vector<float> desc(feature_dim);
            read_data_vec(file, desc.data(), feature_dim);
        }
        frames[frame.id] = frame;
        // std::cout<<frame.id<<" "<<frame.name<<"
        // "<<frame.points.size()<<std::endl;
    }
}

void ReadFramePairBinaryForTriangulation(const std::string &path,
                                         std::vector<FramePair> &frame_pairs) {
    std::ifstream file(path, std::ios::binary);
    CHECK(file.is_open()) << path;
    const uint64_t num_pair = read_data2<uint64_t>(file, num_pair);
    frame_pairs.reserve(num_pair);
    for (uint64_t i = 0; i < num_pair; ++i) {
        FramePair fp;
        fp.id1 = read_data2<uint32_t>(file);
        fp.id2 = read_data2<uint32_t>(file);
        uint64_t num_matches = read_data2<uint64_t>(file);
        fp.matches.resize(num_matches);
        fp.inlier_mask.resize(num_matches, true);
        for (size_t i = 0; i < num_matches; ++i) {
            auto &match = fp.matches[i];
            read_data(file, match.id1);
            read_data(file, match.id2);
        }
        frame_pairs.push_back(fp);
    }
}

void WriteCamerasBinary(const std::string &path,
                        const std::vector<Camera> &cameras) {
    std::ofstream file(path, std::ios::trunc | std::ios::binary);
    CHECK(file.is_open()) << path;
    uint64_t num_camera = cameras.size();
    write_data(file, num_camera);
    for (const auto &camera : cameras) {
        uint64_t w = 2 * camera.cx(), h = 2 * camera.cy();
        write_data(file, camera.id_);
        write_data(file, camera.model_id_);
        write_data(file, w);
        write_data(file, h);
        write_data_vec(file, camera.params_.data(), 4);
    }
}

void WriteImagesBinary(const std::string &path,
                       const std::vector<Frame> &frames) {
    std::ofstream file(path, std::ios::trunc | std::ios::binary);
    CHECK(file.is_open()) << path;
    uint64_t num_frame = 0;
    for (const auto &frame : frames) {
        if (!frame.registered)
            continue;
        num_frame++;
    }
    write_data(file, num_frame);

    for (const auto &frame : frames) {
        if (!frame.registered)
            continue;
        const uint32_t id = frame.id;
        const uint64_t num_p2d = frame.points.size();
        write_data(file, id);
        Eigen::Vector4d q_vec(frame.Tcw.q.w(), frame.Tcw.q.x(), frame.Tcw.q.y(),
                              frame.Tcw.q.z());
        write_data(file, q_vec); // todo
        write_data(file, frame.Tcw.t);
        write_data(file, frame.camera_id);
        write_name(file, frame.name);
        write_data(file, num_p2d);

        for (size_t i = 0; i < num_p2d; ++i) {
            const auto &p2d = frame.points[i];
            const uint64_t track_id = frame.track_ids_[i];
            write_data(file, p2d);
            write_data(file, track_id);
        }
    }
}

void WritePoints3DBinary(const std::string &path,
                         const std::vector<Track> tracks) {
    std::ofstream file(path, std::ios::trunc | std::ios::binary);
    CHECK(file.is_open()) << path;

    uint64_t num_track = 0;
    for (const auto &track : tracks) {
        if (track.outlier)
            continue;
        num_track++;
    }
    write_data(file, num_track);

    for (uint64_t i = 0, tracks_size = tracks.size(); i < tracks_size; ++i) {
        const auto &track = tracks[i];
        if (track.outlier)
            continue;
        write_data(file, i);
        write_data(file, track.point3d_);
        std::array<uint8_t, 3> color = {0, 0, 0};
        write_data_vec(file, color.data(), 3);
        write_data(file, track.error);
        const uint64_t num_obs = track.observations_.size();
        write_data(file, num_obs);
        for (const auto &[frame_id, p2d_id] : track.observations_) {
            write_data(file, frame_id);
            write_data(file, p2d_id);
        }
    }
}

void WriteColMapDataBinary(const std::string &output_path, const Map &map) {
    WriteCamerasBinary(output_path + "cameras.bin", map.cameras_);
    WriteImagesBinary(output_path + "images.bin", map.frames_);
    WritePoints3DBinary(output_path + "points3D.bin", map.tracks_);
}

void WriteImagesBinary2(const std::string &path,
                        const std::map<int, Frame> &frames) {
    std::ofstream file(path, std::ios::trunc | std::ios::binary);
    CHECK(file.is_open()) << path;
    const uint64_t num_frame = frames.size();
    write_data(file, num_frame);

    for (const auto &[frame_id, frame] : frames) {
        if (!frame.registered)
            continue;
        const uint32_t id = frame_id;
        const uint64_t num_p2d = frame.points.size();
        write_data(file, id);
        Eigen::Vector4d q_vec(frame.Tcw.q.w(), frame.Tcw.q.x(), frame.Tcw.q.y(),
                              frame.Tcw.q.z());
        write_data(file, q_vec); // todo
        write_data(file, frame.Tcw.t);
        write_data(file, frame.camera_id);
        const std::string name = frame.name + '\0';
        file.write(name.c_str(), name.size());
        write_data(file, num_p2d);

        for (size_t i = 0; i < num_p2d; ++i) {
            const auto &p2d = frame.points[i];
            const uint64_t track_id = frame.track_ids_[i];
            write_data(file, p2d);
            write_data(file, track_id);
        }
    }
}

void WritePoints3DBinary2(const std::string &path,
                          const std::map<int, Track> tracks) {
    std::ofstream file(path, std::ios::trunc | std::ios::binary);
    CHECK(file.is_open()) << path;
    const uint64_t num_track = tracks.size();
    write_data(file, num_track);

    for (const auto &[id, track] : tracks) {
        if (track.outlier)
            continue;
        const uint64_t track_id = id;
        write_data(file, track_id);
        write_data(file, track.point3d_);
        std::array<uint8_t, 3> color = {0, 0, 0};
        write_data_vec(file, color.data(), 3);
        write_data(file, track.error);
        const uint64_t track_length = track.observations_.size();
        // printf("%d %d\n", track_id, track_length);
        write_data(file, track_length);
        for (const auto &[frame_id, p2d_id] : track.observations_) {
            write_data(file, frame_id);
            write_data(file, p2d_id);
        }
    }
    // printf("%d\n", tracks.size());
}

void WriteColMapDataBinary2(const std::string &output_path, const Map &map) {
    WriteCamerasBinary(output_path + "cameras.bin", map.cameras_);
    WriteImagesBinary2(output_path + "images.bin", map.frame_map_);
    WritePoints3DBinary2(output_path + "points3D.bin", map.track_map_);
}

} // namespace xrsfm
