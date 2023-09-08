#include "imu_uwb/utility/file_manger.hpp"

namespace IMU_UWB{
    FileManager::FileManager(){

    };
    FileManager::~FileManager(){

    };
    bool FileManager::CreatDirectory(std::string dir_path) {
        if(!boost::filesystem::is_directory(dir_path)){
            boost::filesystem::create_directories(dir_path);
        }
        if(!boost::filesystem::is_directory(dir_path)){
            std::cout << "无法创建文件夹： " << dir_path << std::endl;
            return  false;
        }
        return true;


    }

    bool FileManager::CreatFile(std::ofstream& ofs, std::string file_path) {
        ofs.open(file_path.c_str(),std::ios::app);
        if(!ofs){
            std::cout << "无法创建文件：　"<< file_path <<std::endl;
            return false;
        }
        return true;

    }

    bool FileManager::IsExist(std::string file_path) {
        if(boost::filesystem::is_regular_file(file_path)){
            return true;
        }
        std::cout << "file doesn't exits ...";
        return false;
    }
}