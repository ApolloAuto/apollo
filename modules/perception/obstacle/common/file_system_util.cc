#include "modules/perception/obstacle/common/file_system_util.h"
#include <boost/filesystem.hpp>

namespace apollo {
namespace perception {

bool is_exists(const std::string& path) {
    boost::filesystem::path p(path);
    return boost::filesystem::exists(p);
}

bool is_directory(const std::string& path) {
    boost::filesystem::path p(path);
    return boost::filesystem::is_directory(p);
}

bool create_directory(const std::string& path) {
    boost::filesystem::path p(path);
    return boost::filesystem::create_directory(p);
}

bool get_file_size(const std::string& path, unsigned int& size) {
    boost::filesystem::path p(path);
    if (boost::filesystem::exists(p)) {
        if (boost::filesystem::is_regular_file(p)) {
            size = boost::filesystem::file_size(p);
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

void get_files_in_folder_recursive(
        const std::string& folder, 
        const std::string& ext, 
        std::vector<std::string>& ret) {
    ret.clear();
    namespace fs = boost::filesystem;
    if (!fs::exists(folder) || !fs::is_directory(folder)) {
        return;
    }

    fs::recursive_directory_iterator it(folder);
    fs::recursive_directory_iterator endit;

    while (it != endit) {
        if (fs::is_regular_file(*it) && it->path().extension() == ext) {
            ret.push_back(it->path().string());
        }
        ++it;
    }
    std::sort(ret.begin(), ret.end());
}

void get_files_in_folder(const std::string& folder, 
    const std::string& ext, std::vector<std::string>& ret) {
    ret.clear();
    namespace fs = boost::filesystem;
    if (!fs::exists(folder) || !fs::is_directory(folder)) {
        return;
    }

    fs::directory_iterator it(folder);
    fs::directory_iterator endit;

    while (it != endit) {
        if (fs::is_regular_file(*it) && it->path().extension() == ext) {
            ret.push_back(it->path().string());
        }
        ++it;
    }
    std::sort(ret.begin(), ret.end());
}

void get_folders_in_folder(const std::string& folder, 
    std::vector<std::string>& ret) {
    
    ret.clear();
    namespace fs = boost::filesystem;
    if (!fs::exists(folder) || !fs::is_directory(folder)) {
        return;
    }

    fs::directory_iterator it(folder);
    fs::directory_iterator endit;

    while (it != endit) {
        if (fs::is_directory(*it)) {
            ret.push_back(it->path().string());
        }
        ++it;
    }
    std::sort(ret.begin(), ret.end());
}

void get_file_names_in_folder(const std::string& folder, 
    const std::string& ext, std::vector<std::string>& ret) {
    ret.clear();
    namespace fs = boost::filesystem;
    if (!fs::exists(folder) || !fs::is_directory(folder)) {
        return;
    }

    fs::directory_iterator it(folder);
    fs::directory_iterator endit;

    while (it != endit) {
        if (fs::is_regular_file(*it) && it->path().extension() == ext) {
            ret.push_back(it->path().filename().string());
        }
        ++it;
    }
    std::sort(ret.begin(), ret.end());
}

void get_file_names_in_folder_by_id(const std::string& folder, 
    const std::string& ext, std::vector<std::string>& ret) {
    std::vector<int> ret_id;
    ret.clear();
    namespace fs = boost::filesystem;
    if (!fs::exists(folder) || !fs::is_directory(folder)) {
        return;
    }

    fs::directory_iterator it(folder);
    fs::directory_iterator endit;

    while (it != endit) {
        if (fs::is_regular_file(*it) && it->path().extension() == ext) {
            std::string temp_path = it->path().filename().string();
            ret.push_back(temp_path);
            std::string temp_id_str = temp_path.substr(temp_path.rfind('_')+1,
                    temp_path.rfind('.')-temp_path.rfind('_')-1);
            int temp_id = std::atoi(temp_id_str.c_str());
            ret_id.push_back(temp_id);
        }
        ++it;
    }
    // sort
    int ret_size = ret.size();
    for (int i = 0; i < ret_size; ++i) {
        for (int j = i; j < ret_size; ++j) {
            if (ret_id[i] > ret_id[j]) {
                int temp_id = ret_id[i];
                ret_id[i] = ret_id[j];
                ret_id[j] = temp_id;
                std::string temp_path = ret[i];
                ret[i] = ret[j];
                ret[j] = temp_path;
            }
        }
    }
}

std::string get_directory_name(const std::string& path){
    std::string directory = path.substr(0,path.rfind('/'));
    if (directory == "" || directory == "."){
        directory = "./";
    }else {
        directory = directory + "/";
    }
    return directory;
}

std::string get_extension(const std::string& path){
    std::string filename = get_file_name(path);
    std::string::size_type loc = filename.rfind('.');
    if (loc == std::string::npos){
        return "";
    }else{
        return filename.substr(loc+1);
    }
}

std::string get_file_name(const std::string& path){
    std::string filename;
    std::string::size_type loc = path.rfind('/');
    if (loc == std::string::npos){
        filename = path;
    } else{
        filename = path.substr(loc+1);
    }
    return filename;
}

std::string get_file_name_without_extension(const std::string& path){
    std::string filename = get_file_name(path);
    std::string::size_type loc = filename.rfind('.');
    return filename.substr(0,loc);
}

bool has_extension(const std::string& path){
    std::string filename = get_file_name(path);
    std::string::size_type loc = filename.rfind('.');
    if (loc == std::string::npos){
        return false;
    }
    return true;
}

} //namespace perception
} //namespace apollo
