#ifndef MODULES_PERCEPTION_OBSTACLE_COMMON_CIRCULAR_ARRAY_H_
#define MODULES_PERCEPTION_OBSTACLE_COMMON_CIRCULAR_ARRAY_H_
#include <vector>

namespace apollo {
namespace perception {

template <class Data>
class CircularArray {
public:
    explicit CircularArray(unsigned int max_count);
    CircularArray(const CircularArray<Data>& rhs);
    CircularArray<Data>& operator=(const CircularArray<Data>& rhs);
    void push_back(const Data& data);
    void get_array(std::vector<Data>& array);
    void clear();

private:
    CircularArray();

private:
    std::vector<Data>          _data;
    unsigned int               _max_count;
    unsigned int               _count;
};

template <class Data>
CircularArray<Data>::CircularArray(unsigned int max_count): _max_count(max_count),
    _count(0) {
    _data.resize(_max_count);
}

template <class Data>
CircularArray<Data>::CircularArray(const CircularArray<Data>& rhs) {
    _data.assign(rhs._data.begin(), rhs._data.end());
    _max_count = rhs._max_count;
    _count = rhs._count;
}

template <class Data>
CircularArray<Data>& CircularArray<Data>::operator=(const CircularArray<Data>& rhs) {
    _data.assign(rhs._data.begin(), rhs._data.end());
    _max_count = rhs._max_count;
    _count = rhs._count;
    return (*this);    
}

template <class Data>
void CircularArray<Data>::push_back(const Data& data) {
    int next = _count % _max_count;
    _data[next] = data;
    _count++;
}

template <class Data>
void CircularArray<Data>::get_array(std::vector<Data>& array) {
    if (_count > _max_count) {
        int start = _count % _max_count;
        array.resize(_max_count);
        for (int i = start; i < _max_count; i++) {
            array[i - start] = _data[i];
        }
        int offset = _max_count - start;
        for (int i = 0; i < start; i++) {
            array[offset + i] = _data[i];
        }
    } else {
        array.assign(_data.begin(), _data.begin() + _count);
    }
}

template <class Data>
void CircularArray<Data>::clear() {
    _count = 0;
}

} //namespace perception
} //namespace apollo
#endif // MODULES_PERCEPTION_OBSTACLE_COMMON_CIRCULAR_ARRAY_H_
