一、资源文件配置
1.将AutoMap目录下的内容放到"/opt/hmc/engines/HNavigationEngine/data"目录下，也可根据实际情况将资源目录通过new HMapViewer()指定
eg: #define CFG_DATA_CONFIG "/opt/share/BDMapAuto"
    m_pMap = new HMapViewer(HMapType::FRONT,  DEV_WINDOW_WIDTH, DEV_WINDOW_HEIGHT, CFG_DATA_CONFIG);
2.将hmc文件放到/opt/目录下
将以上目录设置为可读写权限

二、程序使用流程说明
1.开发环境：Ubuntu14.04 + Qt 5.7.0，使用qml开发UI, 工程名字automap-linux-qml.pro
2.引擎使用流程简介：（请结合BaiduMapAutoQmlInterface.cpp）
   按功能模块划分为mapView、search、routeGenerator、routeGuide；
   每个模块都要创建新实例，调用相应的API接口，API详见doc;
   a.mapView模块
     mapView实例创建一定放在在SurfaceView创建好之后进行
	 在绘制线程以40ms左右周期调用 mapView->draw()，可参考BaiduMapAutoQmlInterface::Render()接口
	 
   b.search模块
     设置filter
	 调用search()， 参考BaiduMapAutoQmlInterface::doSearch()接口
	 监听检索结果， 参考BaiduMapAutoQmlInterface::onSearchResultUpdate接口
   
   c.routeGenerator模块
     设置startPosition
	 设置destPosition
	 generate()	 具体参考BaiduMapAutoQmlInterface::startRoutePlan()
     监听算路结果参考BaiduMapAutoQmlInterface::onGenerateStatusUpdated()
    
   d.routeGuide模块
     调用startSimulation或者startRouteGuidance()
	 startSimulation模拟导航接口
	 startRouteGuidance真实导航接口
	 诱导信息回调实现BaiduMapAutoQmlInterface::onTurnInfoUpdated()
	 

三、Linux环境配置
	1.拷贝依赖库
	新建$HOME/.jumbo/lib目录，并将jumbo_lib.tar.gz解压到lib目录中
	
	2.安装编译工具集
	安装命令：
	sudo apt-get install build-essential
	安装完成后验证gcc／g++版本
	PS: if meet ” E: Unable to locate package essential” ,please update apt-get
	sudo apt-get update
	
	3.安装OpenGL工具集
	安装opengl 3命令：
	sudo apt-get install freeglut3-dev
	安装opengl es命令：
	sudo apt-get install libgles2-mesa-dev

	4.安装freetype
    apt-get update更新索引列表
    sudo apt-get install libfreetype6-dev
	wqy-microhei.ttc 拷贝到/usr/share/fonts/truetype/wqy/wqy-microhei.ttc
