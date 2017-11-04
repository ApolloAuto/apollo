#include "modules/localization/msf/local_map/lossless_map/lossless_map_matrix.h"
#include <iostream>

namespace apollo {
namespace localization {
namespace msf {
//======================LosslessMapSingleCell===========================
LosslessMapSingleCell::LosslessMapSingleCell():
    intensity(0.0), intensity_var(0.0), 
    altitude(0.0), altitude_var(0.0), count(0) {
}

void LosslessMapSingleCell::reset(){
    intensity = 0.0;
    intensity_var = 0.0;
    altitude = 0.0;
    count = 0;
}

LosslessMapSingleCell& LosslessMapSingleCell::operator = (
            const LosslessMapSingleCell& ref) {
    intensity = ref.intensity;
    intensity_var = ref.intensity_var;  
    altitude = ref.altitude;
    count = ref.count;
    return *this;
}

void LosslessMapSingleCell::add_sample(const float new_altitude, 
                    const float new_intensity) {

    ++count;
    float v1 = new_intensity - intensity;
    float value = v1 / count;
    intensity += value;
    float v2 = new_intensity - intensity;
    intensity_var = (static_cast<float>(count - 1) * 
                    intensity_var + v1 * v2) / count;
    
    v1 = new_altitude - altitude;
    value = v1 / count;
    altitude += value;
    v2 = new_altitude - altitude;
    altitude_var = (static_cast<float>(count - 1) * 
                    altitude_var + v1 * v2) / count;
}

unsigned int LosslessMapSingleCell::load_binary(unsigned char *buf) {
    float *p = reinterpret_cast<float *>(buf);
    intensity = *p;
    ++p;
    intensity_var = *p;
    ++p;
    altitude = *p;
    ++p;
    altitude_var = *p;
    ++p;
    unsigned int *pp = reinterpret_cast<unsigned int*>(p);
    count = *pp;
    return get_binary_size();
}

unsigned int LosslessMapSingleCell::create_binary(unsigned char *buf, 
                                        unsigned int buf_size) const {
    unsigned int target_size = get_binary_size();
    if (buf_size >= target_size) {
        float *p = reinterpret_cast<float *>(buf);
        *p = intensity;
        ++p;
        *p = intensity_var;
        ++p;
        *p = altitude;
        ++p;
        *p = altitude_var;
        ++p;
        unsigned int *pp = reinterpret_cast<unsigned int*>(p);
        *pp = count;
    }
    return target_size;
}

unsigned int LosslessMapSingleCell::get_binary_size() const {
    return sizeof(float)*4+sizeof(unsigned int);
}

//======================LosslessMapCell===========================
LosslessMapCell::LosslessMapCell() {
    _layer_num = 1;
}

void LosslessMapCell::reset() {
    for (unsigned int i = 0; i < IDL_CAR_NUM_RESERVED_MAP_LAYER; ++i) {
        _map_cells[i].reset(); 
    }
    _layer_num = 1;
}

void LosslessMapCell::set_value_layer(double altitude, unsigned char intensity,
                         double altitude_thres) {
    assert(_layer_num <= IDL_CAR_NUM_RESERVED_MAP_LAYER);

    unsigned int best_layer_id = get_layer_id(altitude);
    assert(best_layer_id < _layer_num);
    if (best_layer_id == 0) {
        if (_layer_num < IDL_CAR_NUM_RESERVED_MAP_LAYER){
            // No layer yet, create a new one
            LosslessMapSingleCell& cell = _map_cells[_layer_num++];
            cell.add_sample((float)altitude, static_cast<float>(intensity));
        }
        else {
            // No enough reserved map layers.
            std::cerr << 
                "[Warning] There are no enough reserved map cell layers. Please increase IDL_CAR_NUM_RESERVED_MAP_LAYER." 
                << std::endl;
        }
    }
    else {
        // There is a best layer
        double layer_alt_dif = fabs(_map_cells[best_layer_id].altitude - altitude);
        if (layer_alt_dif < altitude_thres) {
            // Still a good one
            LosslessMapSingleCell& cell = _map_cells[best_layer_id];
            cell.add_sample((float)altitude, static_cast<float>(intensity));
        }
        else {
            // Should create a new one
            if (_layer_num < IDL_CAR_NUM_RESERVED_MAP_LAYER) {
                LosslessMapSingleCell& cell = _map_cells[_layer_num++];
                cell.add_sample((float)altitude, static_cast<float>(intensity));
            }
            else {
                // No enough reserved map layers.
                std::cerr << 
                    "[Warning] There are no enough reserved map cell layers. Please increase IDL_CAR_NUM_RESERVED_MAP_LAYER." 
                    << std::endl;
            }
        }
    }    
}

void LosslessMapCell::set_value(double altitude, unsigned char intensity) {
    assert(_layer_num <= IDL_CAR_NUM_RESERVED_MAP_LAYER);
    LosslessMapSingleCell& cell = _map_cells[0];
    cell.add_sample((float)altitude, static_cast<float>(intensity));
}

unsigned int LosslessMapCell::load_binary(unsigned char *buf) {
    unsigned int *p = reinterpret_cast<unsigned int*>(buf);
    unsigned int size = *p;
    ++p;
    _layer_num = size;
    unsigned char * pp = reinterpret_cast<unsigned char *>(p);
    for (unsigned int i = 0; i < size; ++i) {
        LosslessMapSingleCell& cell = _map_cells[i];
        unsigned int processed_size = cell.load_binary(pp);
        pp += processed_size;
    }
    return get_binary_size();
}

unsigned int LosslessMapCell::create_binary(unsigned char *buf, unsigned int buf_size) const {
    unsigned int target_size = get_binary_size();
    if (buf_size >= target_size) {
        unsigned int *p = reinterpret_cast<unsigned int*>(buf);
        *p = _layer_num;
        ++p;
        buf_size -= sizeof(unsigned int);
        unsigned char * pp = reinterpret_cast<unsigned char *>(p);
        for (size_t i = 0; i < _layer_num; ++i) {
            const LosslessMapSingleCell& cell = _map_cells[i];
            unsigned int processed_size = cell.create_binary(pp, buf_size);
            assert(buf_size >= processed_size);
            buf_size -= processed_size;
            pp += processed_size;
        }
    }
    return target_size;
}

unsigned int LosslessMapCell::get_binary_size() const {
    unsigned int target_size = sizeof(unsigned int); // The size of the variable for the number of layers.
    for (size_t i = 0; i < _layer_num; ++i) {
        const LosslessMapSingleCell& cell = _map_cells[i];
        target_size += cell.get_binary_size();
    }
    return target_size;
}

unsigned int LosslessMapCell::get_layer_id(double altitude) const {
    unsigned int best_layer_id = 0;
    double best_layer_alt_dif = 1e10;
    for (unsigned int i = 1; i < _layer_num; ++i) {
        const LosslessMapSingleCell& cell = _map_cells[i];
        double layer_alt_dif = fabs(cell.altitude - altitude);
        if (layer_alt_dif < best_layer_alt_dif) {
            best_layer_alt_dif = layer_alt_dif;
            best_layer_id = i;
        }
    }
    return best_layer_id;
}

void LosslessMapCell::get_value(std::vector<unsigned char>& values) const {
    values.clear();
    for (unsigned int i = 1; i < _layer_num; ++i) {
        const LosslessMapSingleCell& cell = _map_cells[i];
        values.push_back(static_cast<unsigned char>(cell.intensity));
    }
}

void LosslessMapCell::get_var(std::vector<float>& vars) const {
    vars.clear();
    for (unsigned int i = 1; i < _layer_num; ++i) {
        const LosslessMapSingleCell& cell = _map_cells[i];
        vars.push_back(cell.intensity_var);
    }
}

void LosslessMapCell::get_alt(std::vector<float>& alts) const {
    alts.clear();
    for (unsigned int i = 1; i < _layer_num; ++i) {
        const LosslessMapSingleCell& cell = _map_cells[i];
        alts.push_back(cell.altitude);
    }
}

void LosslessMapCell::get_alt_var(std::vector<float>& alt_vars) const {
    alt_vars.clear();
    for (unsigned int i = 1; i < _layer_num; ++i) {
        const LosslessMapSingleCell& cell = _map_cells[i];
        alt_vars.push_back(cell.altitude_var);
    }
}

void LosslessMapCell::get_count(std::vector<unsigned int>& counts) const {
    counts.clear();
    for (unsigned int i = 1; i < _layer_num; ++i) {
        const LosslessMapSingleCell& cell = _map_cells[i];
        counts.push_back(cell.count);
    }
}

//======================LosslessMapMatrix===========================
LosslessMapMatrix::LosslessMapMatrix() {
    _rows = 0;
    _cols = 0;
    _map_cells = NULL;
}

LosslessMapMatrix::~LosslessMapMatrix() {
    if(_map_cells) {
        delete[] _map_cells;
    }
    _rows = 0;
    _cols = 0;
}

LosslessMapMatrix::LosslessMapMatrix(const LosslessMapMatrix& matrix): 
        BaseMapMatrix(matrix) {
    init(matrix._rows, matrix._cols);
    for (unsigned int y = 0; y < _rows; ++y) {
        for (unsigned int x = 0; x < _cols; ++x) {
            _map_cells[y * _cols + x] = matrix[y][x];
        }
    }
}

void LosslessMapMatrix::init(const BaseMapConfig* config) {
    unsigned int rows = config->_map_node_size_y;
    unsigned int cols = config->_map_node_size_x;
    if (_rows == rows && _cols == cols) {
        return;
    }
    init(rows, cols);
    return;
}

void LosslessMapMatrix::reset(const BaseMapConfig* config) {
    reset(config->_map_node_size_y, config->_map_node_size_x);
    return;
}

void LosslessMapMatrix::init(unsigned int rows, unsigned int cols) {
    if (_map_cells) {
        delete[] _map_cells;
        _map_cells = NULL;
    }
    _map_cells = new LosslessMapCell[rows * cols];
    _rows = rows;
    _cols = cols;
}

void LosslessMapMatrix::reset(unsigned int rows, unsigned int cols) {
    unsigned int length = rows * cols;
    for (unsigned int i = 0; i < length; ++i) {    
        _map_cells[i].reset();
    }
}

unsigned int LosslessMapMatrix::load_binary(unsigned char * buf) {
    unsigned int *p = reinterpret_cast<unsigned int*>(buf);
    _rows = *p;
    ++p;
    _cols = *p;
    ++p;
    init(_rows, _cols);

    unsigned char * pp = reinterpret_cast<unsigned char *>(p);
    for (unsigned int y = 0; y < _rows; ++y) {
        for (unsigned int x = 0; x < _cols; ++x) {
            LosslessMapCell& cell = get_map_cell(y, x);
            unsigned int processed_size = cell.load_binary(pp);
            pp += processed_size;
        }
    }
    return get_binary_size();
}

unsigned int LosslessMapMatrix::create_binary(unsigned char * buf, 
                                    unsigned int buf_size) const {
    unsigned int target_size = get_binary_size();
    if (buf_size >= target_size) {
        unsigned int *p = reinterpret_cast<unsigned int*>(buf);
        *p = _rows;
        ++p;
        *p = _cols;
        ++p;
        buf_size -= (sizeof(unsigned int)*2);
        unsigned char *pp = reinterpret_cast<unsigned char *>(p);
        for (unsigned int y = 0; y < _rows; ++y) {
            for (unsigned int x = 0; x < _cols; ++x) {
                const LosslessMapCell& cell = get_map_cell(y, x);
                unsigned int processed_size = cell.create_binary(pp, buf_size);
                assert(buf_size >= processed_size);
                buf_size -= processed_size;
                pp += processed_size;
            }
        }
    }
    return target_size;
}

unsigned int LosslessMapMatrix::get_binary_size() const {
    //default binary size
    unsigned int target_size = sizeof(unsigned int)*2;    // rows and cols
    for (unsigned int y = 0; y < _rows; ++y) {
        for (unsigned int x = 0; x < _cols; ++x) {
            const LosslessMapCell& cell = get_map_cell(y, x);
            target_size += cell.get_binary_size();
        }
    }
    return target_size;
}

void LosslessMapMatrix::get_intensity_img(cv::Mat &intensity_img) const {
    intensity_img = cv::Mat(cv::Size(_cols, _rows), CV_8UC1);

    for (int y = 0; y < _rows; ++y) {
        for (int x = 0; x < _cols; ++x) {
            intensity_img.at<unsigned char>(y, x) = get_map_cell(y, x).get_value();
        }
    }
}

} // namespace msf
} // namespace localization
} // namespace apollo
