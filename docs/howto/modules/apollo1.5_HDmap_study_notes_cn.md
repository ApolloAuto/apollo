# HD map 使用
## 生成HD map:
由于apollo的hd map制作没有开放，所以目前hd map的生成是需要向百度提需求的。
如果想自己制作的话，apollo有提供建议如下：
 * 原始数据采集(视觉、激光雷达、GPS等)以及处理。
 * 地图数据生成。从步骤一生成的数据通过算法或者人工的方式获取地图数据。
 * 地图格式组织。将地图数据转换为Apollo的高精度地图格式（可以参照base_map.xml格式，其他的地图都可以从base_map.xml生成）。
 * 注意：这三个步骤的工具均需要自己开发，如果只是小规模的简单测试，也可以参照base_map.xml格式手工组织数据。

## 将HD map加入apollo1.5：
有两个方法，一个是通过添加一个新的目录，使用apollo系统；一个是替换原有目录下的地图文件。
### 新加一个hd map：
 * 在/apollo/modules/map/data目录下，创建一个目录new_map。
 * 将生成的hd map放入new_map中，如有配置文件，可以参考sunnyvale_office目录下的配置文件。
 * 编译，执行bash apollo.sh build。
 * 然后执行bash scripts/hmi.sh。
 * 打开ip:8887，在选择地图的下拉框中就可以看到新加入的hd map了。
 * 直接copy new_garage 重命名为new_garage_2测试的，测试通过。
 * 注1：编译的时候，应该相当于将/apollo/modules/map/data/new_map注册到系统中去，以便启动hmi时，前端网页可以定位到/apollo/modules/map/data/new_map目录，进而加载其中的文件。也因此，可以有第二个方法加入hd map。

### 利用现有的地图目录，加入地图：
 * 假设apollo1.5中，已经添加了new_map，此时只需要替换目录下的hd map所有的文件，这样不需要编译，即可使用新的hd map。
