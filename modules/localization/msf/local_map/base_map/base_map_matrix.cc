#include "modules/localization/msf/local_map/base_map/base_map_matrix.h"
#include <assert.h>

namespace apollo {
namespace localization {
namespace msf {

BaseMapMatrix::BaseMapMatrix() {
}

BaseMapMatrix::~BaseMapMatrix() {
}

BaseMapMatrix::BaseMapMatrix(const BaseMapMatrix& cells) {
}

unsigned int BaseMapMatrix::load_binary(unsigned char * buf) {
    return 0;
}

unsigned int BaseMapMatrix::create_binary(unsigned char * buf,
                                            unsigned int buf_size) const {
    return 0;
}

unsigned int BaseMapMatrix::get_binary_size() const {
    return 0;
}

} // namespace msf
} // namespace localization
} // namespace apollo
