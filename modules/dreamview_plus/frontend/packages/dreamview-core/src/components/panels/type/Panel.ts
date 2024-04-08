import React from 'react';

export interface IPanelMetaInfo {
    title: string;
    type: string;
    originType?: 'remote' | 'local';
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
    PncMonitor = 'pncMonitor',
    Components = 'components',
    MapCollect = 'MapCollect',
    Charts = 'charts',
    TerminalWin = 'terminalWin',
}
