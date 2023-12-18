export const guide = {
    welcome: 'Welcome',
    viewLoginSteps: 'View login steps',
    modeSelectDesc:
        'We provide you with the following visualization template. Choose one as the default interface to open.',
    defaultMode: 'Default Mode',
    perceptionMode: 'Perception Mode',
    enterThisMode: 'Enter this mode',
    modules: 'Modules',
    panel: 'Panel',

    DefaultDesc:
        'The default mode follows the old version of Dreamview layout and is applicable to all scenarios where debugging begins.',
    DefaultModules: 'Include all modules',
    DefaultPanel: 'Vehicle Visualization、Vehicl Dashboard、Module delay、Console',
    PerceptionDesc:
        'The perception mode is suitable for the development and debugging scenarios of perception algorithms. In this mode, developers can intuitively view the raw data of sensors and point clouds. It supports the synchronization of multiple sensors, and enables developers to intuitively view the output obstacle results during the perception process.',
    PerceptionModules:
        'Prediction、Perception、Transform、Lidar、Radar、CameraSigleStage、CameraMultiStage、Lane、TrafficLight',
    PerceptionPanel: 'Vehicle Visualization、Camera View、Point Cloud、Console、Module Delay',
    PncDesc:
        'The PnC mode is suitable for developers who develop planning and control modules. It provides data operation process options, visual data display panels and debugging information panels related to PnC development and debugging.',
    PncModules: 'planning、prediction、planning、control、routing、task manager、recorder',
    PncPanel: 'Vehicle Visualization、Console、Module delay、PnC Monitor、Vehicle Dashboard',
    'Vehicle TestDesc':
        'The vehicle test mode is suitable for the development and debugging scenarios based on the real vehicle. In this mode, developers can conveniently monitor the status of key equipment such as the vehicle chassis and positioning device, view the operation of the automatic driving system, and complete data collection, waypoint-following, automatic driving demonstration and other operations on the real vehicle.',
    'Vehicle TestModules':
        'Prediction、Camera 2D、Camera 3D、Perception、Traffic Light、Lane、Task Manager、Planning、Control、LiDAR、Radar、Canbus、GNSS、Localization、TF、Guardian',
    'Vehicle TestPanel': 'Vehicle Visualization、Vehicle Dashboard、Console、Module Delay、Components',

    skip: 'Skip',
    back: 'Back',
    next: 'Next',
    close: 'Close',

    perceptionSelectModule: 'Select Module',
    perceptionSelectModuleDesc:
        'You are currently in perception mode, and we have sorted out the commonly used modules in this mode. You can turn them on and off the modules you need to run as needed.',
    perceptionSelectOperations: 'Select Operations',
    perceptionSelectOperationsDesc:
        'Select the corresponding operations in the current mode. You can perform record playback in the perception mode.',
    perceptionProfileManager: 'Resource Manager',
    perceptionProfileManagerDesc:
        'The Resource Manager is a car cloud integrated resource manager provided by apollo, providing download functions for materials such as vehicles, models, data packages, and simulation scenarios. You can click on the required data record, vehicle model, simulation scenarios, and other elements to download.',
    perceptionSelectResources: 'Select Resources',
    perceptionSelectResourcesDesc: 'You can try selecting a record for data playback',
    perceptionPerceivedEffects: 'Perceived effects',
    perceptionPerceivedEffectsDesc:
        'During data records playback, you can check point cloud and camera images. You can also adjust the current layout and panel.',

    defaultSelectMode: 'Mode Settings',
    defaultSelectModeDesc: 'You are currently in the default mode. This mode provides the basic visualization panel.',
    defaultSelectModule: 'Select Module',
    defaultSelectModuleDesc: 'This mode includes all modules buttons, which you can turn on and off as needed.',
    defaultSelectOperations: 'Select Operations',
    defaultSelectOperationsDesc:
        'Select the corresponding operations in the current mode, such as data playback, scenario simulation, real vehicle debugging, and waypoint-following.',
    defaultProfileManager: 'Resource Manager',
    defaultProfileManagerDesc:
        'The Resource Manager is a car cloud integrated resource manager provided by apollo, providing download functions for materials such as vehicles, models, data packages, and simulation scenarios. You can click on the required data record, vehicle model, simulation scenarios, and other elements to download.',
    defaultSelectVariableRes: 'Select Resources',
    defaultSelectVariableResDesc: 'You can try selecting a data record for data playback.',
    defaultSelectFixedRes: 'Select Resources',
    defaultSelectFixedResDesc: 'If you are using data records for data playback, please select the map as needed.',

    PNCSelectOperations: 'Mode Settings',
    PNCSelectOperationsDesc:
        'You are currently in PnC mode. If you want to perform scenario simulation, turn on the Scenario button to start the simulation environment; if you do not need to perform simulation, skip this step.',
    PNCSelectModules: 'Select Module',
    PNCSelectModulesDesc:
        'We have sorted out the commonly used modules involved in PNC, and you can turn them on and off as needed.',
    PNCResourceManager: 'Resource Manager',
    PNCResourceManagerDesc:
        'Resource Manager is a car-cloud integrated configuration center provided by Apollo, providing download functions for materials such as vehicles, models, records, and simulation scenarios. You can click on the required records, vehicles, models, simulation scenarios, and other elements to download.',
    PNCSelectScenario: 'Select Scenario',
    PNCSelectScenarioDesc: 'Select a scene for simulation, or select a data package for data playback.',
    PNCSelectMap: 'Select Map',
    PNCSelectMapDesc:
        'If you are performing simulation/data playback, please continue to select the map corresponding to this scene after selecting the simulation scene/data package.',
    PNCSelectADS: 'Select Vehicle',
    PNCSelectADSDesc:
        'If you are performing simulation, please continue to select the vehicle information corresponding to this scene after selecting Module Delay Console in the simulation scene (If you use the data package for data playback, you only need to select the map).',
    PNCSetRoute: 'Set Route',
    PNCSetRouteDesc:
        'If you want to perform simulation, after selecting the scene, vehicle, and map, click Routing Editing to set the routing, and you can set the start point and end point information you need. If you are using data packets for data playback, please ignore this step.',
    PNCSimulationRecord: 'Start the simulation/record',
    PNCSimulationRecordDesc:
        'After perform the above operations, please click Run/Play button to perform simulation/data playback.',

    VehicleSelectModules: 'Mode Settings',
    VehicleSelectModulesDesc:
        'You are currently in the Vehicle Test mode. The necessary modules in this mode have been sorted out for you. You can enable and disable corresponding modules according to your needs.',
    VehicleSelectOperations: 'Select Operation',
    VehicleSelectOperationsDesc:
        'Select the corresponding operation process in the current mode, such as automatic driving and waypoint-following.',
    VehicleResourceManager: 'Resource Manager',
    VehicleResourceManagerDesc:
        'Resource Manager is a car-cloud integrated configuration center provided by Apollo, providing download functions for materials such as vehicles, models, records, and simulation scenarios. You can click on the required records, vehicles, models, simulation scenarios, and other elements to download.',
    VehicleSelectVehicle: 'Select Vehicle',
    VehicleSelectVehicleDesc:
        'In ADS Resources, you can select the configuration corresponding to the current vehicle.',
    VehicleSelectMap: 'Select Map',
    VehicleSelectMapDesc:
        'If you are debugging autonomous driving, please select the corresponding map information. If you are performing waypoint-following debugging, you need to select the recorded waypoint data records.',
    VehicleRoutingEditing: 'Set the path for Auto Drive',
    VehicleRoutingEditingDesc: 'When debugging autonomous driving, you need to select the path on the current map.',
    VehicleStartAutoDrive: 'Start Auto Drive',
    VehicleStartAutoDriveDesc:
        'After completing the above operations, please click START Auto button to perform automatic driving debugging.',

    viewLoginStepOne:
        'Open https://apollo.baidu.com/workspace in the browser to access the Apollo Studio cloud workspace.',
    viewLoginStepTwo: 'Click "Personal Center" ，then enter "My Services".',
    viewLoginStepThree:
        'Select "Simulation", click "Generate"in the "Plugin Installation", then choose the Apollo version and click "Confirm".',
    viewLoginStepFour:
        'Click "One-Click Copy", then run the command in your Docker environment, and the plugin synchronization (Dreamview login) will be completed.',
    loginTip: 'Log in to your personal account to use Cloud Profile',
};
