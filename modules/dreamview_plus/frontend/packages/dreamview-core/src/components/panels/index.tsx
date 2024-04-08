/* eslint-disable prettier/prettier */
/* eslint-disable arrow-body-style */
import { TFunction } from 'i18next';
import React from 'react';
import { IPanelMetaInfo, PanelType } from './type/Panel';
import ChannelSelectFactory from './base/ChannelSelect/ChannelSelectFactory';
import { StreamDataNames } from '../../services/api/types';
import DemoChannelSelect from './base/ChannelSelect/demo';
import { DashBoardHelp } from './DashBoard/PanelHelp';
import { CameraViewHelp } from './CameraView/PanelHelp';
import { PointCloudHelp } from './PointCloud/PanelHelp';
import PanelHelp from './Console/PanelHelp';
import { VehicleVizPanelHelp } from './VehicleViz/PanelHelp';
import { loadComponent, loadRemoteEntry } from '../../util/moduleFederation';

interface RemotePanelJsonDesc {
    description: string;
    exposedModuleName: string;
    originType: 'remote' | 'local';
    remoteEntry: string;
    remoteName: string;
    title: string;
    type: string;
}

const loadRemotePanels = async (
    t: TFunction | ((str: string) => string),
    customChartIllustratorImg: string,
): Promise<IPanelMetaInfo[]> => {
    const remotePanels: IPanelMetaInfo[] = [];
    try {
        const remotePanelJsonDesc: RemotePanelJsonDesc[] = await fetch(
            '/proto/modules/dreamview_plus_plugin_panels/panel.json',
            {
                method: 'GET',
                headers: {
                    'Content-Type': 'application/json',
                    'Cache-Control': 'no-cache',
                },
                mode: 'cors',
            },
        ).then((res) => res.json());
        remotePanels.push(
            ...remotePanelJsonDesc.map((panel) => ({
                title: panel.title,
                type: panel.type,
                originType: panel.originType,
                thumbnail: customChartIllustratorImg,
                description: panel.description,
                module: () =>
                    loadRemoteEntry(panel.remoteEntry, panel.remoteName)
                        .then(() => loadComponent(panel.remoteName, panel.exposedModuleName))
                        .then((Component) => {
                            return Component;
                        }),
            })),
        );
        return remotePanels;
    } catch (e) {
        console.error(e);
    }
    return remotePanels;
};

export const getAllPanels = async (
    t: TFunction | ((str: string) => string),
    imgSrc: any,
): Promise<IPanelMetaInfo[]> => {
    const [
        consoleIllustratorImg,
        moudleDelayIllustratorImg,
        vehicleVizIllustratorImg,
        cameraViewIllustratorImg,
        pointCloudIllustratorImg,
        dashBoardIllustratorImg,
        chartIllustratorImg,
        componentsIllustratorImg,
        terminalIllustratorImg,
        customChartIllustratorImg,
    ] = imgSrc;
    let remotePanels: IPanelMetaInfo[] = [];
    try {
        remotePanels = await loadRemotePanels(t, customChartIllustratorImg);
    } catch (e) {
        console.error(e);
    }

    const localPanels: IPanelMetaInfo[] = [
        {
            title: t('consoleTitle'),
            type: PanelType.Console,
            thumbnail: consoleIllustratorImg,
            description: t('consoleDescription'),
            module: () => import('@dreamview/dreamview-core/src/components/panels/Console'),
            renderToolbar: ChannelSelectFactory({
                helpContent: <PanelHelp description={t('consoleDescription')} />,
            }),
        },
        {
            title: t('moduleDelayTitle'),
            type: PanelType.ModuleDelay,
            thumbnail: moudleDelayIllustratorImg,
            description: t('moduleDelayDescription'),
            module: () => import('@dreamview/dreamview-core/src/components/panels/ModuleDelay'),
            renderToolbar: ChannelSelectFactory({
                helpContent: <PanelHelp description={t('moduleDelayDescription')} />,
            }),
        },
        {
            title: t('vehicleVizTitle'),
            type: PanelType.VehicleViz,
            thumbnail: vehicleVizIllustratorImg,
            description: t('vehicleVizDescription'),
            renderToolbar: ChannelSelectFactory({
                helpContent: <VehicleVizPanelHelp />,
            }),
            module: () => import('@dreamview/dreamview-core/src/components/panels/VehicleViz'),
        },
        {
            title: t('cameraViewTitle'),
            type: PanelType.CameraView,
            thumbnail: cameraViewIllustratorImg,
            description: t('cameraViewDescription'),
            renderToolbar: ChannelSelectFactory({
                name: StreamDataNames.CAMERA,
                CustomToolBar: (props) => <DemoChannelSelect {...props} />,
                helpContent: <CameraViewHelp />,
            }),
            module: () => import('@dreamview/dreamview-core/src/components/panels/CameraView'),
        },
        {
            title: t('pointCloudTitle'),
            type: PanelType.PointCloud,
            thumbnail: pointCloudIllustratorImg,
            description: t('pointCloudDescription'),
            renderToolbar: ChannelSelectFactory({
                name: StreamDataNames.POINT_CLOUD,
                CustomToolBar: (props) => <DemoChannelSelect {...props} />,
                helpContent: <PointCloudHelp />,
            }),
            module: () => import('@dreamview/dreamview-core/src/components/panels/PointCloud'),
        },
        {
            title: t('dashBoardTitle'),
            type: PanelType.DashBoard,
            thumbnail: dashBoardIllustratorImg,
            description: t('dashBoardDescription'),
            renderToolbar: ChannelSelectFactory({
                helpContent: <DashBoardHelp />,
            }),
            module: async () => {
                const { DashBoard } = await import('@dreamview/dreamview-core/src/components/panels/DashBoard');
                return {
                    default: DashBoard,
                };
            },
        },
        {
            title: t('pncMonitorTitle'),
            type: PanelType.PncMonitor,
            thumbnail: chartIllustratorImg,
            description: t('pncMonitorDescription'),
            renderToolbar: ChannelSelectFactory({
                helpContent: <PanelHelp description={t('pncMonitorDescription')} />,
            }),
            module: () => import('@dreamview/dreamview-core/src/components/panels/PncMonitor'),
        },
        {
            title: t('componentsTitle'),
            type: PanelType.Components,
            thumbnail: componentsIllustratorImg,
            description: t('componentsDescription'),
            module: () => import('@dreamview/dreamview-core/src/components/panels/Components'),
        },
        {
            title: t('terminalWinTitle'),
            type: PanelType.TerminalWin,
            thumbnail: terminalIllustratorImg,
            description: t('terminalWinDescription'),
            module: () => import('@dreamview/dreamview-core/src/components/panels/Terminal'),
        },
        {
            title: t('chartsTitle'),
            type: PanelType.Charts,
            thumbnail: customChartIllustratorImg,
            description: t('chartsDescription'),
            module: () => import('@dreamview/dreamview-core/src/components/panels/Charts'),
        },
    ];
    return [...remotePanels, ...localPanels];
};
