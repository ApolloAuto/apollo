#include "modules/localization/msf/local_map/base_map/base_map_node.h"
#include <cstdio>
#include "modules/localization/msf/common/util/system_utility.h"
#include "modules/localization/msf/local_map/base_map/base_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {

BaseMapNode::BaseMapNode(BaseMapMatrix* matrix, CompressionStrategy* strategy): 
        _map_config(NULL), _map_matrix(matrix), _compression_strategy(strategy) {
    _is_changed = false;
    _data_is_ready = false;
    _is_reserved = false;
    _min_altitude = 1e6;
}

BaseMapNode::~BaseMapNode() {
    delete _map_matrix;
    delete _compression_strategy;
}

void BaseMapNode::init(const BaseMapConfig* map_config,
        const MapNodeIndex& index, bool create_map_cells) {
    _map_config = map_config;
    _index = index;
    _left_top_corner = get_left_top_corner(*_map_config, _index);
    _is_reserved = false;
    _data_is_ready = false;
    _is_changed = false;
    if (create_map_cells) {
        init_map_matrix(_map_config);
    }
    return;
}

void BaseMapNode::init_map_matrix(const BaseMapConfig* map_config) {
    _map_config = map_config;
    _map_matrix->init(map_config);
}

void BaseMapNode::finalize() {
    if(_is_changed) {
        save();
        std::cerr << "Save Map Node to disk: " << _index << "." << std::endl;
    }
}

void BaseMapNode::reset_map_node() {
    _is_changed = false;
    _data_is_ready = false;
    _is_reserved = false;
    _map_matrix->reset(_map_config);
}

// void BaseMapNode::SetCompressionStrategy(compression::CompressionStrategy* strategy) {
//     _compression_strategy = strategy;
//     return;
// }

bool BaseMapNode::save() {
    save_intensity_image();
    char buf[1024];
    std::string path = _map_config->_map_folder_path;
    if (!create_map_directory(path)) {
        return false;
    }
    path = path + "/map";
    if (!create_map_directory(path)) {
        return false;
    }
    snprintf(buf, 1024, "/%03u", _index._resolution_id);
    path = path + buf;
    if (!create_map_directory(path)) {
        return false;
    }
    if (_index._zone_id > 0) {
        path = path + "/north";
    }
    else {
        path = path + "/south";
    }
    if (!create_map_directory(path)) {
        return false;
    }
    snprintf(buf, 1024, "/%02d", abs(_index._zone_id));
    path = path + buf;
    if (!create_map_directory(path)) {
        return false;
    }
    snprintf(buf, 1024, "/%08u", abs(_index._m));
    path = path + buf;
    if (!create_map_directory(path)) {
        return false;
    }
    snprintf(buf, 1024, "/%08u", abs(_index._n));
    path = path + buf;

    std::cout << "Save node: " << path << std::endl;

    FILE * file = fopen(path.c_str(), "wb");
    if (file) {
        create_binary(file);
        fclose(file);
        _is_changed = false;
        return true;
    }
    else {
        std::cerr << "Can't write to file: " << path << "." << std::endl;
        return false;
    }
}

bool BaseMapNode::load() {
    char buf[1024];
    std::string path = _map_config->_map_folder_path;
    if (!(system::is_exists(path)
            && system::is_directory(path))) {
        return false;
    }
    path = path + "/map";
    if (!(system::is_exists(path) 
            && system::is_directory(path))) {
        return false;
    }
    snprintf(buf, 1024, "/%03u", _index._resolution_id);
    path = path + buf;
    if (!(system::is_exists(path) 
            && system::is_directory(path))) {
        return false;
    }
    if (_index._zone_id > 0) {
        path = path + "/north";
    }
    else {
        path = path + "/south";
    }
    if (!(system::is_exists(path) 
            && system::is_directory(path))) {
        return false;
    }
    snprintf(buf, 1024, "/%02d", abs(_index._zone_id));
    path = path + buf;
    if (!(system::is_exists(path) 
            && system::is_directory(path))) {
        return false;
    }
    snprintf(buf, 1024, "/%08u", abs(_index._m));
    path = path + buf;
    if (!(system::is_exists(path) 
            && system::is_directory(path))) {
        return false;
    }
    snprintf(buf, 1024, "/%08u", abs(_index._n));
    path = path + buf;

    return load(path.c_str());
}

bool BaseMapNode::load(const char* filename) {
    _data_is_ready = false;
    // char buf[1024];

    FILE * file = fopen(filename, "rb");
    if (file) {
        load_binary(file);
        fclose(file);
        _is_changed = false;
        _data_is_ready = true;
        return true;
    }
    else {
        std::cerr << "Can't find the file: " << filename << std::endl;
        return false;
    }
}

unsigned int BaseMapNode::load_binary(FILE * file) {
    // Load the header
    unsigned int header_size = get_header_binary_size();
    std::vector<unsigned char> buf(header_size);
    size_t read_size = fread(&buf[0], 1, header_size, file);
    assert(read_size == header_size);
    unsigned int processed_size = load_header_binary(&buf[0]);
    assert(processed_size == header_size);

    // Load the body
    buf.resize(_file_body_binary_size);
    read_size = fread(&buf[0], 1, _file_body_binary_size, file);
    assert(read_size == _file_body_binary_size);
    processed_size += load_body_binary(buf);
    return processed_size;
}

unsigned int BaseMapNode::create_binary(FILE* file) const {
    unsigned int buf_size = get_binary_size();
    std::vector<unsigned char> buffer;
    buffer.resize(buf_size);

    unsigned int binary_size = 0;
    std::vector<unsigned char> body_buffer;
    unsigned int processed_size = create_body_binary(body_buffer);
    // assert(processed_size == buf_size);

    // Create header
    unsigned int header_size = get_header_binary_size();
    std::vector<unsigned char> header_buf(header_size);
    processed_size = create_header_binary(&buffer[0], buf_size);
    assert(processed_size == header_size);

    unsigned int buffer_bias = processed_size;
    buf_size -= processed_size;
    binary_size += processed_size;
    // Create body
    memcpy(&buffer[buffer_bias], &body_buffer[0], body_buffer.size());
    assert(buf_size >= body_buffer.size());
    binary_size += body_buffer.size();
    fwrite(&buffer[0], 1, binary_size, file);
    return binary_size;
}

unsigned int BaseMapNode::get_binary_size() const {
    // It is uncompressed binary size.
    return get_body_binary_size() + get_header_binary_size();
}

unsigned int BaseMapNode::load_header_binary(unsigned char * buf) {
    unsigned int target_size = get_header_binary_size();
    unsigned int * p = reinterpret_cast<unsigned int*>(buf);
    _index._resolution_id = *p;     
    ++p;
    int * pp = reinterpret_cast<int*>(p);
    _index._zone_id = *pp; 
    ++pp;
    p = reinterpret_cast<unsigned int*>(pp);
    _index._m = *p;
    ++p;
    _index._n = *p;
    ++p;
    // _left_top_corner = get_left_top_corner(*_map_config, _index);
    _left_top_corner = get_left_top_corner(*_map_config, _index);
    _file_body_binary_size = *p;
    return target_size;
}

unsigned int BaseMapNode::create_header_binary(
        unsigned char * buf, unsigned int buf_size) const {
    unsigned int target_size = get_header_binary_size();
    if (buf_size >= target_size) {
        unsigned int * p = reinterpret_cast<unsigned int*>(buf);
        *p = _index._resolution_id;
        ++p;
        int * pp = reinterpret_cast<int*>(p);
        *pp = _index._zone_id;
        ++pp;
        p = reinterpret_cast<unsigned int*>(pp);
        *p = _index._m;
        ++p;
        *p = _index._n;
        ++p;
        *p = _file_body_binary_size;  // Set it before call this function!
    }
    return target_size;
}

unsigned int BaseMapNode::get_header_binary_size() const {
    return sizeof(unsigned int)     // _index._resolution_id
        +sizeof(int)            // _index._zone_id
        +sizeof(unsigned int)   // _index._m
        +sizeof(unsigned int)   // _index._n
        +sizeof(unsigned int);  // the body size in file.
}

// unsigned int BaseMapNode::create_body_binary(
//         std::vector<unsigned char> &buf) const {
//     // Compute the binary body size
//     unsigned int body_size = get_body_binary_size();
//     buf.resize(body_size);
//     return _map_matrix->create_binary(&buf[0], body_size);
// }

// unsigned int BaseMapNode::get_body_binary_size() const {
//     return _map_matrix->get_binary_size();
// }

unsigned int BaseMapNode::load_body_binary(std::vector<unsigned char> &buf) {
    if(_compression_strategy == NULL) {
        return _map_matrix->load_binary(&buf[0]);
    }
    std::vector<unsigned char> buf_uncompressed;
    _compression_strategy->decode(buf, buf_uncompressed);
    std::cerr << "map node compress ratio: " <<
            (float)(buf.size())/buf_uncompressed.size() << std::endl;
    return _map_matrix->load_binary(&buf_uncompressed[0]);
}

unsigned int BaseMapNode::create_body_binary(
        std::vector<unsigned char> &buf) const {
    if(_compression_strategy == NULL) {
        unsigned int body_size = get_body_binary_size();
        buf.resize(body_size);
        _map_matrix->create_binary(&buf[0], body_size);
        _file_body_binary_size = buf.size();
        return buf.size();
    }
    std::vector<unsigned char> buf_uncompressed;
    // Compute the uncompression binary body size
    unsigned int body_size = get_body_binary_size();
    buf_uncompressed.resize(body_size);
    _map_matrix->create_binary(&buf_uncompressed[0], body_size);
    _compression_strategy->encode(buf_uncompressed, buf);
    _file_body_binary_size = buf.size();
    return buf.size();
}

unsigned int BaseMapNode::get_body_binary_size() const {
    return _map_matrix->get_binary_size();
}

// bool BaseMapNode::get_coordinate(const idl::car::core::numerical::Vector2D& coordinate,
//         unsigned int& x, unsigned int& y) const {
//     const idl::car::core::numerical::Vector2D& left_top_corner = get_left_top_corner();
//     int off_x = static_cast<int>((coordinate[0] - left_top_corner[0])/get_map_resolution());
//     int off_y = static_cast<int>((coordinate[1] - left_top_corner[1])/get_map_resolution());
//     if (off_x >= 0 && off_x < this->_map_config->_map_node_size_x &&
//         off_y >= 0 && off_y < this->_map_config->_map_node_size_y) {
//         x = static_cast<unsigned int>(off_x);
//         y = static_cast<unsigned int>(off_y);
//         return true;
//     }
//     else {
//         return false;
//     }
// }

bool BaseMapNode::get_coordinate(const Eigen::Vector2d& coordinate, 
        unsigned int& x, unsigned int& y) const {
    const Eigen::Vector2d& left_top_corner = get_left_top_corner();
    int off_x = static_cast<int>((coordinate[0] - left_top_corner[0])/get_map_resolution());
    int off_y = static_cast<int>((coordinate[1] - left_top_corner[1])/get_map_resolution());
    if (off_x >= 0 && off_x < int(this->_map_config->_map_node_size_x) &&
        off_y >= 0 && off_y < int(this->_map_config->_map_node_size_y)) {
        x = static_cast<unsigned int>(off_x);
        y = static_cast<unsigned int>(off_y);
        return true;
    }
    else {
        return false;
    }
}

// bool BaseMapNode::get_coordinate(const idl::car::core::numerical::Vector3D& coordinate,
//         unsigned int& x, unsigned int& y) const {
//     idl::car::core::numerical::Vector2D coord2d;
//     coord2d.init(coordinate.get_data());
//     return get_coordinate(coord2d, x, y);
// }

bool BaseMapNode::get_coordinate(const Eigen::Vector3d& coordinate, 
        unsigned int& x, unsigned int& y) const {
    Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
    return get_coordinate(coord2d, x, y);
}

// idl::car::core::numerical::Vector2D BaseMapNode::get_coordinate(
//         unsigned int x, unsigned int y) const {
//     const idl::car::core::numerical::Vector2D& left_top_corner = get_left_top_corner();
//     idl::car::core::numerical::Vector2D coord;
//     coord[0] = left_top_corner[0] + x * get_map_resolution();
//     coord[1] = left_top_corner[1] + y * get_map_resolution();
//     return coord;
// }

Eigen::Vector2d BaseMapNode::get_coordinate(unsigned int x, unsigned int y) const {
    const Eigen::Vector2d& left_top_corner = get_left_top_corner();
    Eigen::Vector2d coord(left_top_corner[0] + x * get_map_resolution(), 
            left_top_corner[1] + y * get_map_resolution());
    return coord;
}

// idl::car::core::numerical::Vector2D BaseMapNode::get_left_top_corner(
//     const BaseMapConfig& config, const MapNodeIndex& index) {
//     idl::car::core::numerical::Vector2D coord;
//     coord[0] = config._map_range.get_min_x() +
//             config._map_node_size_x*config._map_resolutions[index._resolution_id]*index._n;
//     coord[1] = config._map_range.get_min_y() +
//             config._map_node_size_y*config._map_resolutions[index._resolution_id]*index._m;
//     assert(coord[0] < config._map_range.get_max_x());
//     assert(coord[1] < config._map_range.get_max_y());
//     return coord;
// }

Eigen::Vector2d BaseMapNode::get_left_top_corner(
    const BaseMapConfig& config, const MapNodeIndex& index) {
    Eigen::Vector2d coord;
    coord[0] = config._map_range.get_min_x() +
            config._map_node_size_x*config._map_resolutions[index._resolution_id]*index._n;
    coord[1] = config._map_range.get_min_y() +
            config._map_node_size_y*config._map_resolutions[index._resolution_id]*index._m;
    assert(coord[0] < config._map_range.get_max_x());
    assert(coord[1] < config._map_range.get_max_y());
    return coord;
}

bool BaseMapNode::create_map_directory(const std::string& path) const {
    if (system::is_exists(path)) {
        if (!system::is_directory(path)) {
            return false;
        }
        else {
            return true;
        }
    }
    else {
        return system::create_directory(path);
    }
}

bool BaseMapNode::save_intensity_image() const {
    char buf[1024];
    std::string path = _map_config->_map_folder_path;
    if (!create_map_directory(path)) {
        return false;
    }
    path = path + "/image";
    if (!create_map_directory(path)) {
        return false;
    }
    snprintf(buf, 1024, "/%03u", _index._resolution_id);
    path = path + buf;
    if (!create_map_directory(path)) {
        return false;
    }
    if (_index._zone_id > 0) {
        path = path + "/north";
    }
    else {
        path = path + "/south";
    }
    if (!create_map_directory(path)) {
        return false;
    }
    snprintf(buf, 1024, "/%02d", abs(_index._zone_id));
    path = path + buf;
    if (!create_map_directory(path)) {
        return false;
    }
    snprintf(buf, 1024, "/%08u", abs(_index._m));
    path = path + buf;
    if (!create_map_directory(path)) {
        return false;
    }
    snprintf(buf, 1024, "/%08u.png", abs(_index._n));
    path = path + buf;
    bool success0 = save_intensity_image(path);
    return success0;
}

bool BaseMapNode::save_intensity_image(const std::string& path) const {
    cv::Mat image;
    _map_matrix->get_intensity_img(image);
    bool success = cv::imwrite(path, image);
    return success;
}

} // namespace msf
} // namespace localization
} // namespace apollo
