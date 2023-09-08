//
// Created by xc on 2021/4/9.
//
#ifndef IMU_UWB_FILE_MANGER_H
#define IMU_UWB_FILE_MANGER_H
// boost
#include <boost/filesystem.hpp>
#include <string>
#include <iostream>
#include <fstream>
namespace IMU_UWB {
    class FileManager {
    public:
        FileManager();

        ~FileManager();

        bool CreatDirectory(std::string dir_path);

        bool CreatFile(std::ofstream &ofs, std::string file_path);

        bool IsExist(std::string file_path);

    };
}
#endif