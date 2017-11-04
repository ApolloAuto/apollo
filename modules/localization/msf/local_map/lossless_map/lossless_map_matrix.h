#ifndef BAIDU_ADU_LOSSLESS_MAP_MATRIX_H
#define BAIDU_ADU_LOSSLESS_MAP_MATRIX_H

#include "modules/localization/msf/local_map/base_map/base_map_matrix.h"
#include "modules/localization/msf/local_map/base_map/base_map_node.h"
#include <vector>

namespace apollo {
namespace localization {
namespace msf {

/**@brief The first layer (layer 0) includes all the intensities from any layers.
 * Other layers only include the samples from a layer. */
#define IDL_CAR_NUM_RESERVED_MAP_LAYER 2

struct LosslessMapSingleCell {
    /**@brief The default constructor. */
    LosslessMapSingleCell();
    /**@brief Reset to default value. */
    inline void reset();
    /**@brief Add a sample.*/
    void add_sample(const float new_altitude, 
                    const float new_intensity);
    /**@brief Overloading the assign operator. */
    LosslessMapSingleCell& operator = (const LosslessMapSingleCell& ref);
    /**@brief Load the map cell from a binary chunk.
     * @param <return> The size read (the real size of object).
     */
    unsigned int load_binary(unsigned char *buf);
    /**@brief Create the binary. Serialization of the object.
     * @param <buf, buf_size> The buffer and its size.
     * @param <return> The required or the used size of is returned.
     */
    unsigned int create_binary(unsigned char *buf, unsigned int buf_size) const;
    /**@brief Get the binary size of the object. */
    unsigned int get_binary_size() const;

    /**@brief The average intensity value. */
    float intensity;
    /**@brief The variance intensity value. */
    float intensity_var;
    /**@brief The average altitude of the cell. */
    float altitude;
    /**@brief The variance altitude value of the cell. */
    float altitude_var;
    /**@brief The number of samples in the cell. */
    unsigned int count;
};

/**@brief The multiple layers of the cell. */
struct LosslessMapCell {
    /**@brief The default constructor. */
    LosslessMapCell();
    /**@brief Reset to default value. */
    void reset();
    /**@brief Set the value of a layer that layer_id > 0. 
     * The target layer is found according to the altitude. */
    void set_value_layer(double altitude, unsigned char intensity,
                         double altitude_thres = 10.0);
    /**@brief Set the value.
     * @param <altitude> The altitude of the cell.
     * @param <intensity> The reflectance intensity.
     */
    void set_value(double altitude, unsigned char intensity);
    /**@brief Load the map cell from a binary chunk.
     * @param <return> The size read (the real size of object).
     */
    unsigned int load_binary(unsigned char *buf);
    /**@brief Create the binary. Serialization of the object.
     * @param <buf, buf_size> The buffer and its size.
     * @param <return> The required or the used size of is returned.
     */
    unsigned int create_binary(unsigned char *buf, unsigned int buf_size) const;
    /**@brief Get the binary size of the object. */
    unsigned int get_binary_size() const;

    /**@brief Match a layer in the map cell given a altitude.
     * @return The valid layer ID is 1 ~ N (The layer 0 is the layer includes all the samples).
     *  If there is no existing layer, return 0. */
    unsigned int get_layer_id(double altitude) const;
    /**@brief Load the map cell from a binary chunk.
     * @param <return> The size read (the real size of object).
     */
    /**@brief Get the average intensity of all layers in the map cell. */
    void get_value(std::vector<unsigned char>& values) const;
    /**@brief Get the variance of the intensity of all layers in the map cell. */
    void get_var(std::vector<float>& vars) const;
    /**@brief Get the average altitude of all layers in the map cell. */
    void get_alt(std::vector<float>& alts) const;
    /**@brief Get the variance of the altitude of all layers in the map cell. */
    void get_alt_var(std::vector<float>& alt_vars) const;
    /**@brief Get the count of the samples of all layers in the map cell. */
    void get_count(std::vector<unsigned int>& counts) const;
    /**@brief Get the average intensity of the map cell. */
    inline unsigned char get_value() const {
        return static_cast<unsigned char>(_map_cells[0].intensity);
    }
    /**@brief Get the variance of the intensity of the map cell. */
    inline float get_var() const {
        return _map_cells[0].intensity_var;
    }
    /**@brief Get the average altitude of the map cell. */
    inline float get_alt() const {
        return _map_cells[0].altitude;
    }
    /**@brief Get the variance of the altitude of the map cell. */
    inline float get_alt_var() const {
        return _map_cells[0].altitude_var;
    }
    /**@brief Get the count of the samples in the map cell. */
    inline unsigned int get_count() const {
        return _map_cells[0].count;
    }
    /**@brief Get a perticular layer in the map cell. The layer 0 is the layer includes all the samples. */
    LosslessMapSingleCell& get_layer(unsigned int layer_id) {
        assert(layer_id < _layer_num);
        return _map_cells[layer_id];
    }
    /**@brief Get a perticular layer in the map cell. The layer 0 is the layer includes all the samples. */
    const LosslessMapSingleCell& get_layer(unsigned int layer_id) const {
        assert(layer_id < _layer_num);
        return _map_cells[layer_id];
    }

    /**@brief The layers of the cell. */
    unsigned int _layer_num;
    /**@brief The multiple layers of the cell. 
     * The first layer (layer 0) includes all the intensities from any layers.
     * Other layers only include the samples from a layer. */
    LosslessMapSingleCell _map_cells[IDL_CAR_NUM_RESERVED_MAP_LAYER];
};

class LosslessMapMatrix: public BaseMapMatrix {
public:
    LosslessMapMatrix();
    ~LosslessMapMatrix();
    LosslessMapMatrix(const LosslessMapMatrix& matrix);

    virtual void init(const BaseMapConfig* config);
    /**@brief Reset map cells data. */
    virtual void reset(const BaseMapConfig* config);

    void init(unsigned int rows, unsigned int cols);
    void reset(unsigned int rows, unsigned int cols);

    /**@brief Load the map cell from a binary chunk.
     * @param <return> The size read (the real size of object).
     */
    virtual unsigned int load_binary(unsigned char * buf);
    /**@brief Create the binary. Serialization of the object.
     * @param <buf, buf_size> The buffer and its size.
     * @param <return> The required or the used size of is returned.
     */
    virtual unsigned int create_binary(unsigned char * buf, unsigned int buf_size) const;
    /**@brief Get the binary size of the object. */
    virtual unsigned int get_binary_size() const;
    /**@brief get intensity image of node. */
    virtual void get_intensity_img(cv::Mat &intensity_img) const;

    /**@brief Get a map cell. */
    inline const LosslessMapCell& get_map_cell(unsigned int row, unsigned int col) const {
        assert(row < _rows);
        assert(col < _cols);
        return _map_cells[row*_cols + col];
    }
    /**@brief Get a map cell. */
    inline LosslessMapCell& get_map_cell(unsigned int row, unsigned int col) {
        assert(row < _rows);
        assert(col < _cols);
        return _map_cells[row*_cols + col];
    }

    inline LosslessMapCell* operator [] (int row) {
        return _map_cells + row * _cols;
    }
    inline const LosslessMapCell* operator [] (int row) const {
        return _map_cells + row * _cols;
    }

protected:
    /**@brief The number of rows. */
    unsigned int _rows;
    /**@brief The number of columns. */
    unsigned int _cols;
    /**@brief The matrix data structure. */
    LosslessMapCell* _map_cells;
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif //BAIDU_ADU_LOSSLESS_MAP_MATRIX_H