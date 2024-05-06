export const guide = {
    welcome: '欢迎',
    viewLoginSteps: '查看登陆步骤',
    modeSelectDesc: '我们为你提供以下可视化模板，选择一款作为默认打开界面吧～',
    defaultMode: '默认模式',
    perceptionMode: '感知模式',
    enterThisMode: '进入该模式',
    modules: '模块',
    panel: '面板',

    DefaultDesc: '默认模式沿用旧版dreamview布局，适用于所有开始调试的场景。',
    DefaultModules: '包含全部模式',
    DefaultPanel: '车辆可视化、车辆仪表盘、模块延时、控制台',
    PerceptionDesc:
        '感知模式适用于感知算法的开发调试场景，在该模式下，开发者可以直观查看传感器和点云的原始数据，支持多个传感器同步，可直观查看感知输出的障碍物结果。',
    PerceptionModules:
        'Prediction、Perception、Transform、Lidar、Radar、CameraSigleStage、CameraMultiStage、Lane、TrafficLight',
    PerceptionPanel: '车辆可视化、相机视图、点云、控制台、模块延时',
    PncDesc:
        'PnC开发调试模式适用于进行规划与控制模块开发的开发人员，提供PnC开发调试相关的数据操作流程选项、可视化数据展示面板与调试信息面板。',
    PncModules: 'planning、prediction、planning、control、routing、task manager、recorder',
    PncPanel: '车辆可视化、控制台、模块延时、Pnc 监控、车辆仪表盘',
    'Vehicle TestDesc':
        '实车路测模式适用于基于真实车辆的开发调试场景，在该模式下，开发者可以方便的监控车辆的底盘、定位设备等关键设备的状态，以可视化的方式查看自动驾驶系统的运营情况，并完成实车的数据采集、循迹、自动驾驶演示等操作。',
    'Vehicle TestModules':
        'Prediction、Camera 2D、Camera 3D、Perception、Traffic Light、Lane、Task Manager、Planning、Control、LiDAR、Radar、Canbus、GNSS、Localization、TF、Guardian',
    'Vehicle TestPanel': '车辆可视化、车辆仪表盘、控制台、模块延时、监控组件',

    skip: '跳过',
    back: '上一步',
    next: '下一步',
    close: '关闭',

    perceptionSelectModule: '选择模块',
    perceptionSelectModuleDesc:
        '您当前在感知模式。我们已为您挑选出感知模式常用的模块，您可以根据需要开启或者关闭模块。',
    perceptionSelectOperations: '选择操作',
    perceptionSelectOperationsDesc: '选择当前模式下相应的操作流程，感知模式下提供播包操作。',
    perceptionProfileManager: '资源管理',
    perceptionProfileManagerDesc:
        '资源管理是Apollo提供的车云一体化资源管理中心，提供车辆、模型、数据包和仿真场景等素材的下载功能。您可以点击需要的数据包，车辆、模型、仿真场景等元素进行下载。',
    perceptionSelectResources: '选择资源',
    perceptionSelectResourcesDesc: '您可以选择一个数据包进行回放。',
    perceptionPerceivedEffects: '感知效果',
    perceptionPerceivedEffectsDesc: '在数据回放过程中，您可以查看点云和摄像头图像，还可以针对当前布局和面板进行调整。',

    defaultSelectMode: '模式设置',
    defaultSelectModeDesc: '您当前处于默认模式，该模式提供基本的可视化面板显示。',
    defaultSelectModule: '选择模块',
    defaultSelectModuleDesc: '该模式包括所有的模块按钮，您可以根据需要打开或关闭。',
    defaultSelectOperations: '选择操作',
    defaultSelectOperationsDesc: '选择当前模式下相应的操作流程，如播包、场景仿真、实车调试、循迹演示。',
    defaultProfileManager: '资源管理',
    defaultProfileManagerDesc:
        '资源管理是Apollo提供的车云一体化资源管理中心，提供车辆、模型、数据包和仿真场景等素材的下载功能。您可以点击需要的数据包，车辆、模型、仿真场景等元素进行下载。',
    defaultSelectVariableRes: '选择资源',
    defaultSelectVariableResDesc: '您可以选择一个数据包进行回放。',
    defaultSelectFixedRes: '选择资源',
    defaultSelectFixedResDesc: '如果您正在使用数据包进行回放，请选择您需要使用的地图。',

    PNCSelectOperations: '模式设置',
    PNCSelectOperationsDesc:
        '您当前处于PNC模式。如果您想进行场景仿真操作，打开场景按钮开启仿真环境；如果您不需要仿真操作，请跳过这一步。',
    PNCSelectModules: '选择模块',
    PNCSelectModulesDesc: '我们已挑选出PNC中常用的模块，您可以根据需要打开或关闭模块。',
    PNCResourceManager: '资源管理',
    PNCResourceManagerDesc:
        '配置中心是Apollo提供的车云一体的配置中心，为大家提供车辆、模型、数据包、仿真场景等素材的下载功能。您可以点击需要的数据包、车辆、模型、仿真场景等元素进行下载。',
    PNCSelectScenario: '选择场景',
    PNCSelectScenarioDesc: '选择一个场景进行仿真，或选择一个数据包进行数据包回放。',
    PNCSelectMap: '选择地图',
    PNCSelectMapDesc: '如果您正在进行仿真或播包操作，在选择仿真场景或数据包后，请继续选择该场景相对应的地图。',
    PNCSelectADS: '选择车辆',
    PNCSelectADSDesc:
        '如果您正在进行仿真操作，请在仿真场景中选择模块延时控制台后继续选择该场景相关的车辆信息（如果您使用数据包进行播包操作，您只需要选择地图)。',
    PNCSetRoute: '设置路由',
    PNCSetRouteDesc:
        '如果您想要进行仿真操作，在选择场景、车辆和地图后，点击Routing Editing 设置路由，您还可以根据需要设置起点和终点。如果您正在使用数据包进行播包操作，请忽略该步骤。',
    PNCSimulationRecord: '启动仿真/播包',
    PNCSimulationRecordDesc: '执行完以上操作后，请点击运行/播放按钮执行仿真/播包操作。',

    VehicleSelectModules: '模式设置',
    VehicleSelectModulesDesc:
        '您当前处于实车路测模式，已为您梳理此模式下必要的modules，您可以根据自己的需要开启和关闭相应的module。',
    VehicleSelectOperations: '选择操作',
    VehicleSelectOperationsDesc: '选择当前模式下相应的操作流程，如自动驾驶、循迹。',
    VehicleResourceManager: '资源中心',
    VehicleResourceManagerDesc:
        '资源中心是Apollo提供的车云一体的配置中心，为大家提供车辆、模型、数据包、仿真场景等素材的下载功能。您可以点击需要的数据包、车辆、模型、仿真场景等元素进行下载。',
    VehicleSelectVehicle: '选择车辆',
    VehicleSelectVehicleDesc: '进入ADS Resources，您可以选择当前车辆对应的配置。',
    VehicleSelectMap: '选择地图',
    VehicleSelectMapDesc:
        '若您是进行自动驾驶调试，请在选择对应的地图信息。若您是进行循迹调试，则需要选择录制好的轨迹数据包。',
    VehicleRoutingEditing: '设置自动驾驶路径',
    VehicleRoutingEditingDesc: '进行自动驾驶调试时，需要在当前地图上选择路径。',
    VehicleStartAutoDrive: '启动自动驾驶',
    VehicleStartAutoDriveDesc: '进行完以上操作，请点击Start Auto按钮进行自动驾驶调试。',

    viewLoginStepOne: '从浏览器中打开https://apollo.baidu.com/workspace，进入Apollo Studio云端工作台',
    viewLoginStepTwo: '点击“个人中心”，打开“我的服务”',
    viewLoginStepThree: '选择“仿真”，在“插件安装”中点击“生成”，选择Apollo版本后点击“确定”',
    viewLoginStepFour: '选择“一键复制”，之后在您的docker环境中运行该指令，插件同步（Dreamview的登陆）就完成了',

    loginTip: '登录个人账户',
};
