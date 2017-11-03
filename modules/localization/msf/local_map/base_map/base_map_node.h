#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_NODE_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_NODE_H

#include <Eigen/Core>
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/common/util/compression.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The data structure of a Node in the map. */
class BaseMapNode {
public:
    /**@brief Construct a map node. */
    explicit BaseMapNode(BaseMapMatrix* matrix, CompressionStrategy* strategy);
    /**@brief Destruct a map node. */
    virtual ~BaseMapNode();

    /**@brief Initialize the map node. Call this function first before use it! */
    virtual void init(const BaseMapConfig* map_config,
        const MapNodeIndex& index,
        bool create_map_cells = true);
    /**@brief Initialize the map matrix. */
    virtual void init_map_matrix(const BaseMapConfig* map_config);
    /**@brief call before deconstruction or reset. */
    virtual void finalize();
    /**@brief Reset map cells data. */
    virtual void reset_map_node();

    /**@brief Save the map node to the disk. */
    bool save();
    /**@brief Load the map node from the disk. */
    bool load();
    bool load(const char* filename);

    // /**@brief Set compression strategy. */
    // void SetCompressionStrategy(compression::CompressionStrategy* strategy);
    /**@brief Get map cell matrix. */
    inline const BaseMapMatrix& get_map_cell_matrix() const {
        return *_map_matrix;
    }

    inline BaseMapMatrix& get_map_cell_matrix() {
        return *_map_matrix;
    }

    /**@brief Get the map settings. */
    inline const BaseMapConfig& get_map_config() const {
        return *_map_config;
    }
    /**@brief Set the map node index. */
    inline void set_map_node_index(const MapNodeIndex& index) {
        _index = index;
    }
    /**@brief Get the map node index. */
    inline const MapNodeIndex& get_map_node_index() const {
        return _index;
    }
    /**@brief Set if the map node is reserved. */
    inline void set_is_reserved(bool is_reserved) {
        _is_reserved = is_reserved;
    }
    /**@brief Get if the map node is reserved. */
    inline bool get_is_reserved() const {
        return _is_reserved;
    }
    /**@brief Get if the map data has changed. */
    inline bool get_is_changed() const {
        return _is_changed;
    }
    /**@brief Set if the map node data has changed. */
    inline void set_is_changed(bool is) {
        _is_changed = is;
    }
    /**@brief Get if the map node data is ready*/
    inline bool get_is_ready() const {
        return _data_is_ready;
    } 
    /**@brief Get the left top corner of the map node. */
    // inline const idl::car::core::numerical::Vector2D& get_left_top_corner() const {
    //     return _left_top_corner;
    // }
    
    inline const Eigen::Vector2d& get_left_top_corner() const {
        return _left_top_corner;
    }

    inline void set_left_top_corner(double x, double y) {
        // _left_top_corner[0] = x;
        // _left_top_corner[1] = y;

        _left_top_corner[0] = x;
        _left_top_corner[1] = y;
    }
    /**@brief Get the resolution of this map nodex. */
    inline float get_map_resolution() const {
        return this->_map_config->_map_resolutions[this->_index._resolution_id];
    }
    /**@brief Given the global coordinate, get the local 2D coordinate of the map cell matrix.
     * <return> If global coordinate (x, y) belongs to this map node. */
    // bool get_coordinate(const idl::car::core::numerical::Vector2D& coordinate,
    //                     unsigned int& x, unsigned int& y) const;
    // bool get_coordinate(const idl::car::core::numerical::Vector3D& coordinate,
    //                     unsigned int& x, unsigned int& y) const;
    /**@brief Given the local 2D coordinate, return the global coordinate. */
    // idl::car::core::numerical::Vector2D get_coordinate(unsigned int x, unsigned int y) const;

    /**@brief Given the global coordinate, get the local 2D coordinate of the map cell matrix.
     * <return> If global coordinate (x, y) belongs to this map node, eigen version. */
    bool get_coordinate(const Eigen::Vector2d& coordinate, unsigned int& x, unsigned int& y) const;
    bool get_coordinate(const Eigen::Vector3d& coordinate, unsigned int& x, unsigned int& y) const;
    /**@brief Given the local 2D coordinate, return the global coordinate, eigen version. */
    Eigen::Vector2d get_coordinate(unsigned int x, unsigned int y) const;

    // static idl::car::core::numerical::Vector2D get_left_top_corner(const BaseMapConfig& option,
    //                                                      const MapNodeIndex& index);
    static Eigen::Vector2d get_left_top_corner(const BaseMapConfig& option, 
                                                    const MapNodeIndex& index);
protected:
    /**@brief Load the map cell from a binary chunk.
     * @param <return> The size read (the real size of object).
     */
    virtual unsigned int load_binary(FILE* file);
    /**@brief Create the binary. Serialization of the object.
     * @param <return> The the used size of binary is returned.
     */
    virtual unsigned int create_binary(FILE* file) const;
    /**@brief Get the binary size of the object. */
    virtual unsigned int get_binary_size() const;
    /**@brief Try to create the map directory. */
    bool create_map_directory(const std::string& path) const;
    /**@brief Load the map node header from a binary chunk.
     * @param <return> The size read (the real size of header).
     */
    virtual unsigned int load_header_binary(unsigned char * buf);
    /**@brief Create the binary header.
     * @param <buf, buf_size> The buffer and its size.
     * @param <return> The required or the used size of is returned.
     */
    virtual unsigned int create_header_binary(unsigned char * buf, unsigned int buf_size) const;
    /**@brief Get the size of the header in bytes. */
    virtual unsigned int get_header_binary_size() const;
    /**@brief Load the map node body from a binary chunk.
     * @param <return> The size read (the real size of body).
     */
    virtual unsigned int load_body_binary(std::vector<unsigned char> &buf);
    /**@brief Create the binary body.
     * @param <buf, buf_size> The buffer and its size.
     * @param <return> The required or the used size of is returned.
     */
    virtual unsigned int create_body_binary(std::vector<unsigned char> &buf) const;
    /**@brief Get the size of the body in bytes. */
    virtual unsigned int get_body_binary_size() const;

    /**@brief The map settings. */
    const BaseMapConfig* _map_config;
    /**@brief The index of this node. */
    MapNodeIndex _index;
    /**@brief The left top corner of the map node in the global coordinate system. */
    // idl::car::core::numerical::Vector2D _left_top_corner;
    Eigen::Vector2d _left_top_corner;
    /**@brief The data structure of the map datas, which is a matrix. */
    BaseMapMatrix* _map_matrix;
    /**@brief If the node is reserved in map. */
    bool _is_reserved;
    /**@brief Has the map node been changed. */
    bool _is_changed;
    /* *@brief Indicate map node data is ready*/
    bool _data_is_ready;
    /**@brief The body binary size in file. It's useful when reading and writing files. */
    mutable unsigned int _file_body_binary_size;
    /**@bried The compression strategy. */
    CompressionStrategy* _compression_strategy;
    /**@brief The min altitude of point cloud in the node. */
    float _min_altitude;
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_NODE_H_