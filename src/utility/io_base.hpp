//
// Created by SENSETIME\yezhichao1 on 2020/4/5.
//

#pragma once

#include <fstream>
#include <iostream>

namespace xrsfm {

template <typename T>
inline void read_data(std::ifstream &file, T &data, bool log = false) {
    file.read(reinterpret_cast<char *>(&data), sizeof(T));
    //  if (log)
    //      std::cout << data << std::endl;
}

template <typename T>
inline T read_data2(std::ifstream &file, bool log = false) {
    T data;
    file.read(reinterpret_cast<char *>(&data), sizeof(T));
    //  if (log)
    //      std::cout << data << std::endl;
    return data;
}

template <typename T>
inline void read_data_vec(std::ifstream &file, T *data, int num, bool log = false) {
    file.read(reinterpret_cast<char *>(data), num * sizeof(T));
    //  if (log) {
    //      for (int i = 0; i < num; ++i)
    //          std::cout << data[i] << " ";
    //      std::cout << std::endl;
    //  }
}

template <typename T>
inline void write_data(std::ofstream &file, T &data, bool log = false) {
    file.write((char *)(&data), sizeof(T));
    //  if (log)
    //      std::cout << data << std::endl;
}

template <typename T>
inline void write_data_vec(std::ofstream &file, T *data, int num, bool log = false) {
    file.write((char *)(data), num * sizeof(T));
    //  if (log) {
    //      for (int i = 0; i < num; ++i)
    //          std::cout << data[i] << " ";
    //      std::cout << std::endl;
    //  }
}

template <typename T>
inline void write_data_txt(std::ofstream &file, T &data, bool b_newline = false) {
    file << data << " ";
    if (b_newline) {
        file << std::endl;
    }
}

template <typename T>
inline void write_data_vec_txt(std::ofstream &file, T *data, int num) {
    for (int i = 0; i < num; ++i) file << data[i] << " ";
    file << std::endl;
}

inline void read_name(std::ifstream &file, std::string &name) {
    name.clear();
    char name_char;
    do {
        file.read(&name_char, 1);
        if (name_char != '\0') {
            name += name_char;
        }
    } while (name_char != '\0');
}

inline void write_name(std::ofstream &file, const std::string &name) {
    const std::string str = name + '\0';
    file.write(str.c_str(), str.size());
}

} // namespace xrsfm
