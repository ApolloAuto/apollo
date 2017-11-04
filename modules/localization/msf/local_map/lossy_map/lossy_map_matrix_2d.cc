#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"

namespace apollo {
namespace localization {
namespace msf {

LossyMapCell2D::LossyMapCell2D(): 
        count(0), intensity(0.0), intensity_var(0.0), 
        altitude(0.0), altitude_ground(0.0), is_ground_useful(false) {
}

void LossyMapCell2D::reset() {
    intensity = 0.0;
    intensity_var = 0.0;
    altitude = 0.0;
    altitude_ground = 0.0;
    is_ground_useful = false;
    count = 0;
}

LossyMapCell2D& LossyMapCell2D::operator = (const LossyMapCell2D& ref) {
    altitude = ref.altitude;
    altitude_ground = ref.altitude_ground;
    is_ground_useful = ref.is_ground_useful;
    count = ref.count;
    intensity = ref.intensity;
    intensity_var = ref.intensity_var;  
    return *this;
}

LossyMapMatrix2D::LossyMapMatrix2D() {
    _rows = 0;
    _cols = 0;
    _map_cells = NULL;
}

LossyMapMatrix2D::~LossyMapMatrix2D() {
    if(_map_cells) {
        delete[] _map_cells;
    }
    _rows = 0;
    _cols = 0;
}

LossyMapMatrix2D::LossyMapMatrix2D(const LossyMapMatrix2D& matrix): 
        BaseMapMatrix(matrix) {
    init(matrix._rows, matrix._cols);
    for (unsigned int y = 0; y < _rows; ++y) {
        for (unsigned int x = 0; x < _cols; ++x) {
            _map_cells[y * _cols + x] = matrix[y][x];
        }
    }
}

void LossyMapMatrix2D::init(const BaseMapConfig* config) {
    unsigned int rows = config->_map_node_size_y;
    unsigned int cols = config->_map_node_size_x;
    if (_rows == rows && _cols == cols) {
        return;
    }
    init(rows, cols);
    return;
}

void LossyMapMatrix2D::init(unsigned int rows, unsigned int cols) {
    if (_map_cells) {
        delete[] _map_cells;
        _map_cells = NULL;
    }
    _map_cells = new LossyMapCell2D[rows * cols];
    _rows = rows;
    _cols = cols;
}

void LossyMapMatrix2D::reset(const BaseMapConfig* config) {
    reset(config->_map_node_size_y, config->_map_node_size_x);
    return;
}

void LossyMapMatrix2D::reset(unsigned int rows, unsigned int cols) {
    unsigned int length = rows * cols;
    for (unsigned int i = 0; i < length; ++i) {    
        _map_cells[i].reset();
    }
}

unsigned char LossyMapMatrix2D::encode_intensity(const LossyMapCell2D& cell) const {
    int intensity = cell.intensity;
    if (intensity > 255) {
        intensity = 255;
    }
    if (intensity < 0) {
        intensity = 0;
    }
    return intensity;
}

void LossyMapMatrix2D::decode_intensity(unsigned char data, 
        LossyMapCell2D& cell) const {
    cell.intensity = data;
}

unsigned short LossyMapMatrix2D::encode_var(const LossyMapCell2D& cell) const {
    float var = cell.intensity_var;
    var = std::sqrt(var);
    int intensity_var = _var_range / (var * _var_ratio + 1.0);
    if (intensity_var > _var_range) {
        intensity_var = _var_range;
    }
    if (intensity_var < 1) {
        intensity_var = 1;
    }
    return intensity_var;
}

void LossyMapMatrix2D::decode_var(unsigned short data, LossyMapCell2D& cell) const {
    float var = data;
    var = (_var_range / var - 1.0) / _var_ratio;
    cell.intensity_var = var * var;
}

unsigned short LossyMapMatrix2D::encode_altitude_ground(
        const LossyMapCell2D& cell) const {
    float delta_alt = cell.altitude_ground - _alt_ground_min;
    delta_alt /= _alt_ground_interval;
    int ratio = delta_alt + 0.5;
    if (ratio >= _ground_void_flag) {
        ratio = _ground_void_flag - 1;
    }
    if (ratio < 0) {
        ratio = 0;
    }
    return ratio;     
}

void LossyMapMatrix2D::decode_altitude_ground(
        unsigned short data, LossyMapCell2D& cell) const {
    float ratio = data;
    cell.altitude_ground = _alt_ground_min + ratio * _alt_ground_interval;
    return;
}

unsigned short LossyMapMatrix2D::encode_altitude_avg(
        const LossyMapCell2D& cell) const {
    float delta_alt = cell.altitude - _alt_avg_min;
    delta_alt /= _alt_avg_interval;
    int ratio = delta_alt + 0.5;
    if (ratio > 0xffff) {
        ratio = 0xffff;
    }
    if (ratio < 0) {
        ratio = 0;
    }
    return ratio;
}

void LossyMapMatrix2D::decode_altitude_avg(
        unsigned short data, LossyMapCell2D& cell) const {
    float ratio = data;
    cell.altitude = _alt_avg_min + ratio * _alt_avg_interval;
    return;
}

unsigned char LossyMapMatrix2D::encode_count(const LossyMapCell2D& cell) const {
    int count_exp = 0;
    int count_tmp = cell.count;
    while (count_tmp > 0) {
        ++count_exp;
        count_tmp /= 2;
    }
    if (count_exp > _count_range) {
        count_exp = _count_range;
    }
    return count_exp;
}

void LossyMapMatrix2D::decode_count(unsigned char data, LossyMapCell2D& cell) const {
    int count_exp = data;
    if (count_exp == 0) {
        cell.count = count_exp;
    } else {
        cell.count = 1 << (count_exp - 1);
    }
}

unsigned int LossyMapMatrix2D::load_binary(unsigned char * buf) {
    unsigned int * p = reinterpret_cast<unsigned int*>(buf);
    _rows = *p;
    ++p;
    _cols = *p;
    ++p;
    // std::cerr << "rows: " << _rows << ", clos: " << _cols << std::endl;
    float* pf = reinterpret_cast<float*>(p);
    _alt_avg_min = *pf;
    ++pf;
    _alt_avg_max = *pf;
    ++pf;
    // std::cerr << "alt_min: " << _alt_avg_min << ", alt_max: " << _alt_avg_max << std::endl;
    _alt_ground_min = *pf;
    ++pf;
    _alt_ground_max = *pf;
    ++pf;
    // std::cerr << "alt_min: " << _alt_min << ", alt_max: " << _alt_max << std::endl;

    init(_rows, _cols);

    unsigned char * pp = reinterpret_cast<unsigned char *>(pf);
    //count
    for (unsigned int row = 0; row < _rows; ++row) {
        for (unsigned int col = 0; col < _cols; ++col) {
            decode_count(pp[row * _cols + col], _map_cells[row * _cols + col]);
        }
    }
    pp += _rows * _cols;

    //intensity
    for (unsigned int row = 0; row < _rows; ++row) {
        for (unsigned int col = 0; col < _cols; ++col) {
            decode_intensity(pp[row * _cols + col], _map_cells[row * _cols + col]);
        }
    }
    pp += _rows * _cols;

    //intensity_var
    unsigned char * pp_low = pp + _rows * _cols;
    unsigned char * pp_high = pp;
    for (unsigned int row = 0; row < _rows; ++row) {
        for (unsigned int col = 0; col < _cols; ++col) {
            unsigned short var = pp_high[row * _cols + col];
            var = var * 256 + pp_low[row * _cols + col];
            decode_var(var, _map_cells[row * _cols + col]);
        }
    }
    pp += 2 * _rows * _cols;

    //altitude_avg 
    pp_low = pp + _rows * _cols;
    pp_high = pp;
    for (unsigned int row = 0; row < _rows; ++row) {
        for (unsigned int col = 0; col < _cols; ++col) {
            unsigned short alt = pp_high[row * _cols + col];
            alt = alt * 256 + pp_low[row * _cols + col];
            LossyMapCell2D& cell = _map_cells[row * _cols + col];
            if (cell.count > 0) {
                decode_altitude_avg(alt, cell);
            } else {
                cell.altitude = 0.0;
            }
            
        }
    }
    pp += 2 * _rows * _cols;

    //altitude_ground
    pp_low = pp + _rows * _cols;
    pp_high = pp;
    for (unsigned int row = 0; row < _rows; ++row) {
        for (unsigned int col = 0; col < _cols; ++col) {
            unsigned short alt = pp_high[row * _cols + col];
            alt = alt * 256 + pp_low[row * _cols + col];
            LossyMapCell2D& cell = _map_cells[row * _cols + col];
            if (alt == _ground_void_flag) {
                cell.is_ground_useful = false;
                cell.altitude_ground = 0.0; 
                } else {
                cell.is_ground_useful = true;
                decode_altitude_ground(alt, cell);
            }
        }
    }
    pp += 2 * _rows * _cols;
    
    return get_binary_size();
}

unsigned int LossyMapMatrix2D::create_binary(unsigned char * buf,
                                    unsigned int buf_size) const {
    unsigned int target_size = get_binary_size();
    if (buf_size >= target_size) {
        unsigned int * p = reinterpret_cast<unsigned int*>(buf);
        *p = _rows;
        ++p;
        *p = _cols;
        ++p;
        buf_size -= sizeof(unsigned int)*2;

        float *pf = reinterpret_cast<float*>(p);
        _alt_avg_min = 1e8;
        _alt_avg_max = -1e8;
        for (unsigned int y = 0; y < _rows; ++y) {
            for (unsigned int x = 0; x < _cols; ++x) {
                const LossyMapCell2D& cell = _map_cells[y * _cols + x];
                if (cell.count == 0) {
                    continue;
                }
                if (cell.altitude > _alt_avg_max) {
                    _alt_avg_max = cell.altitude;
                }
                if (cell.altitude < _alt_avg_min) {
                    _alt_avg_min = cell.altitude;
                }
            }
        }
        *pf = _alt_avg_min;
        ++pf;
        *pf = _alt_avg_max;
        ++pf;
        buf_size -= sizeof(float)*2;

        _alt_ground_min = 1e8;
        _alt_ground_max = -1e8;
        for (unsigned int y = 0; y < _rows; ++y) {
            for (unsigned int x = 0; x < _cols; ++x) {
                const LossyMapCell2D& cell = _map_cells[y * _cols + x];
                if (cell.is_ground_useful == false) {
                    continue;
                }
                if (cell.altitude_ground > _alt_ground_max) {
                    _alt_ground_max = cell.altitude_ground;
                }
                if (cell.altitude_ground < _alt_ground_min) {
                    _alt_ground_min = cell.altitude_ground;
                }
            }
        }
        *pf = _alt_ground_min;
        ++pf;
        *pf = _alt_ground_max;
        ++pf;
        buf_size -= sizeof(float)*2;
        
        unsigned char * pp = reinterpret_cast<unsigned char *>(pf);
        // count
        for (unsigned int row = 0; row < _rows; ++row) {
            for (unsigned int col = 0; col < _cols; ++col) {
                pp[row * _cols + col] = encode_count(_map_cells[row * _cols + col]);
            }
        }
        pp += _rows * _cols;

        // intensity
        for (unsigned int row = 0; row < _rows; ++row) {
            for (unsigned int col = 0; col < _cols; ++col) {
                pp[row * _cols + col] = encode_intensity(_map_cells[row * _cols + col]);
            }
        }
        pp += _rows * _cols;

        // intensity_var
        unsigned char * pp_low = pp + _rows * _cols;
        unsigned char * pp_high = pp;
        for (unsigned int row = 0; row < _rows; ++row) {
            for (unsigned int col = 0; col < _cols; ++col) {
                unsigned short var = encode_var(_map_cells[row * _cols + col]);
                pp_high[row * _cols + col] = var / 256;
                pp_low[row * _cols + col] = var % 256;
            }
        }
        pp += 2 * _rows * _cols;

        // altitude_avg 
        pp_low = pp + _rows * _cols;
        pp_high = pp;
        for (unsigned int row = 0; row < _rows; ++row) {
            for (unsigned int col = 0; col < _cols; ++col) {
                unsigned short altitude = 0.0;
                if (_map_cells[row * _cols + col].count > 0) {
                    altitude = encode_altitude_avg(_map_cells[row * _cols + col]);
                }
                pp_high[row * _cols + col] = altitude / 256;
                pp_low[row * _cols + col] = altitude % 256;
            }
        }
        pp += 2 * _rows * _cols;

        // altitude_ground
        pp_low = pp + _rows * _cols;
        pp_high = pp;
        for (unsigned int row = 0; row < _rows; ++row) {
            for (unsigned int col = 0; col < _cols; ++col) {
                unsigned short altitude = _ground_void_flag;
                if (_map_cells[row * _cols + col].is_ground_useful) {
                    altitude = encode_altitude_ground(_map_cells[row * _cols + col]);
                }
                pp_high[row * _cols + col] = altitude / 256;
                pp_low[row * _cols + col] = altitude % 256;
            }
        }
        pp += 2 * _rows * _cols;
    }
    return target_size;
}

unsigned int LossyMapMatrix2D::get_binary_size() const {
    unsigned int target_size = sizeof(unsigned int)*2 + sizeof(float)*4; // rows and cols and alts
    // count, intensity, intensity_var, altitude_avg, altitude_ground
    target_size += _rows * _cols * 
            (sizeof(unsigned char) + sizeof(unsigned char) + sizeof(unsigned short) 
            + sizeof(unsigned short) + sizeof(unsigned short));
    return target_size;
}

void LossyMapMatrix2D::get_intensity_img(cv::Mat &intensity_img) const {
    intensity_img = cv::Mat(cv::Size(_cols, _rows), CV_8UC1);

    for (int y = 0; y < _rows; ++y) {
        for (int x = 0; x < _cols; ++x) {
            unsigned int id = y * _cols + x;
            intensity_img.at<unsigned char>(y, x) = (unsigned char)(_map_cells[id].intensity);
        }
    }
}

} // namespace msf
} // namespace localization
} // namespace apollo
