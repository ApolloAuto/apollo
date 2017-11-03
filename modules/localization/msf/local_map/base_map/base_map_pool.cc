#include "modules/localization/msf/local_map/base_map/base_map_pool.h"
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_map/base_map/base_map_node.h"

namespace apollo {
namespace localization {
namespace msf {

BaseMapNodePool::BaseMapNodePool(unsigned int pool_size,
                               unsigned int thread_size) 
                               : _pool_size(pool_size),
                                 _node_reset_workers(thread_size) {
}

BaseMapNodePool::~BaseMapNodePool() {
    release();
}

void BaseMapNodePool::initial(const BaseMapConfig* map_config, bool is_fixed_size) {
    _is_fixed_size = is_fixed_size;
    _map_config = map_config;
    for (unsigned int i = 0; i < _pool_size; ++i) {
        BaseMapNode* node = alloc_new_map_node();
        init_new_map_node(node);
        _free_list.push_back(node);
    }
}

void BaseMapNodePool::release() {
    _node_reset_workers.wait();
    typename std::list<BaseMapNode*>::iterator i = _free_list.begin();
    while (i != _free_list.end()) {
        finalize_map_node(*i);
        delloc_map_node(*i);
        i++;
    }
    _free_list.clear();
    typename std::set<BaseMapNode*>::iterator j = _busy_nodes.begin();
    while (j != _busy_nodes.end()) {
        finalize_map_node(*j);
        delloc_map_node(*j);
        j++;
    }
    _busy_nodes.clear();
    _pool_size = 0;
}

BaseMapNode* BaseMapNodePool::alloc_map_node() {
    if (_free_list.empty()) {
        _node_reset_workers.wait();
    }
    boost::unique_lock<boost::mutex> lock(_mutex);    
    if (_free_list.empty()) {
        if (_is_fixed_size) {
            return NULL;
        }
        BaseMapNode* node = alloc_new_map_node();
        init_new_map_node(node);
        _pool_size++;
        _busy_nodes.insert(node);
        return node;
    } else {
        BaseMapNode* node = _free_list.front();
        _free_list.pop_front();
        _busy_nodes.insert(node);
        return node;
    }
}

void BaseMapNodePool::free_map_node(BaseMapNode* map_node) {
    _node_reset_workers.schedule(boost::bind(
        &BaseMapNodePool::free_map_node_task, this, map_node));
}

void BaseMapNodePool::free_map_node_task(BaseMapNode* map_node) {
    finalize_map_node(map_node);
    reset_map_node(map_node);
    {
        boost::unique_lock<boost::mutex> lock(_mutex);
        typename std::set<BaseMapNode*>::iterator f = _busy_nodes.find(map_node);
        assert(f != _busy_nodes.end());
        _free_list.push_back(*f);
        _busy_nodes.erase(f);
    }
}

void BaseMapNodePool::init_new_map_node(BaseMapNode* node) {
    node->init_map_matrix(_map_config);
    return;
}

void BaseMapNodePool::finalize_map_node(BaseMapNode* node) {
    node->finalize();
}

void BaseMapNodePool::delloc_map_node(BaseMapNode* node) {
    delete node;
}

void BaseMapNodePool::reset_map_node(BaseMapNode* node) {
    node->reset_map_node();
}

} // namespace msf
} // namespace localization
} // namespace apollo
