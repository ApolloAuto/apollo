# adapter设计架构代码分析
## 代码分析：
主要文件：adapter_manager.cc, adapter_manager.h。

### 类AdapterManager分析：
 * Init(AdapterManagerConfig &configs)函数：根据传入的AdapterManagerConfig类，配置ros节点，包括创建Nodehandle，创建订阅或者发布话题。
 * Init(const std::string &adapter_config_filename)函数：根据传入的配置文件路径，通过调用Init(AdapterManagerConfig &configs)函数，配置ros节点。
 * Initialized()函数：返回此ros nodehandle是否已经初始化，上两个函数中有设置。
 * CreateTimer()函数创建一个周期timer，以一个类成员函数作为超时函数，在RTK localization函数中有使用到。
 * node_handle_指针，指向node handle指针，后续话题的发布都是基于此。
 * Enable##name()函数：创建nodehandle和topic，也就是init调用的函数。
 * Add##name##Callback函数：添加回调函数。name换成相应的模块名字。
 * observers_变量和Observe()函数：//TODO
### 类AdapterManager使用：
 * adapter可用于创建node handle及其topic，可通过配置文件配置。
 * 使用时只需要创建相应的配置文件（可参考/apollo/modules/perception/conf/adapter.conf，及相应的BUILD文件），
 * 调用接口（可参考/apollo/modules/perception/perception.cc中的Init()函数）。
