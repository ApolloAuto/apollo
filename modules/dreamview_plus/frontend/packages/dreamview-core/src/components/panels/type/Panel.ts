import React from 'react';

export interface IPanelMetaInfo {
    title: string;
    type: string;
    description: string;
    thumbnail?: string;
    renderToolbar?: (props: any) => React.JSX.Element;
    module: () => Promise<{ default: any }>;
}

export enum PanelType {
    Console = 'console',
    ModuleDelay = 'moduleDelay',
    VehicleViz = 'vehicleViz',
    CameraView = 'cameraView',
    PointCloud = 'pointCloud',
    DashBoard = 'dashBoard',
}
