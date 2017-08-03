#ifndef ADU_PERCEPTION_OBSTACLE_COMMON_FILE_SYSTEM_UTIL_H
#define ADU_PERCEPTION_OBSTACLE_COMMON_FILE_SYSTEM_UTIL_H
#include <vector>
#include <string>

namespace apollo {
namespace perception {
    
/**@brief Determine if the file or directory exists. */
bool is_exists(const std::string& path);

/**@brief Determine if the path is a directory. */
bool is_directory(const std::string& path);

/**@brief Try to create a directory. */
bool create_directory(const std::string& path);

/**@brief Get the size of a file. */
bool get_file_size(
        const std::string& path, 
        unsigned int& size);

/**@brief get list of files end with ext in folder.
 * @param <ext> should be .jpg instead of jpg. **/
void get_files_in_folder_recursive(
        const std::string& folder, 
        const std::string& ext, 
        std::vector<std::string>& ret);

/**@brief get list of files end with ext in folder.
 * @param <ext> should be .jpg instead of jpg. **/
void get_files_in_folder(
        const std::string& folder, 
        const std::string& ext, 
        std::vector<std::string>& ret);

/**@brief Get list of folders in folder. */
void get_folders_in_folder(
        const std::string& folder, 
        std::vector<std::string>& ret);

/**@brief get list names of files end with ext in folder.
 * @param <ext> should be .jpg instead of jpg. **/
void get_file_names_in_folder(
        const std::string& folder, 
        const std::string& ext, 
        std::vector<std::string>& ret);
        
void get_file_names_in_folder_by_id(
        const std::string& folder, 
        const std::string& ext, 
        std::vector<std::string>& ret);

/**@brief return the directory information of the specified path string **/
std::string get_directory_name(const std::string& path);

/**@brief return the extension of the specified path string **/
std::string get_extension(const std::string& path);

/**@brief return the filename and extension of the specified path string **/
std::string get_file_name(const std::string& path);

/**@brief return the filename without extension of the specified path string **/
std::string get_file_name_without_extension(const std::string& path);

/**@brief return the full path of the specified path string **/
//std::string get_full_path(const std::string& path);

/**@determine whether a path includes a filename extension**/
bool has_extension(const std::string& path);

}//namespace perception
}//namespace apollo

#endif //namespace ADU_PERCEPTION_OBSTACLE_COMMON_FILE_SYSTEM_UTIL_H
