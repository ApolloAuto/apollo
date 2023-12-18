import { TFunction } from 'i18next';
import React from 'react';
import consoleIllustratorImg from '@dreamview/dreamview-core/src/assets/console_hover_illustrator.png';
import moudleDelayIllustratorImg from '@dreamview/dreamview-core/src/assets/module_delay_hover_illustrator.png';
import vehicleVizIllustratorImg from '@dreamview/dreamview-core/src/assets/vehicle_viz_hover_illustrator.png';
import cameraViewIllustratorImg from '@dreamview/dreamview-core/src/assets/camera_view_hover_illustrator.png';
import pointCloudIllustratorImg from '@dreamview/dreamview-core/src/assets/pointcloud_hover_illustrator.png';
import dashBoardIllustratorImg from '@dreamview/dreamview-core/src/assets/dashboard_hover_illustrator.png';
import chartIllustratorImg from '@dreamview/dreamview-core/src/assets/image_Charts.png';
import componentsIllustratorImg from '@dreamview/dreamview-core/src/assets/Components.png';
import terminalIllustratorImg from '@dreamview/dreamview-core/src/assets/terminal_Illustrator.png';
import customChartIllustratorImg from '@dreamview/dreamview-core/src/assets/panel_chart.png';
import { IPanelMetaInfo, PanelType } from './type/Panel';
import ChannelSelectFactory from './base/ChannelSelect/ChannelSelectFactory';
import { StreamDataNames } from '../../services/api/types';
import DemoChannelSelect from './base/ChannelSelect/demo';
import { DashBoardHelp } from './DashBoard/PanelHelp';
import { CameraViewHelp } from './CameraView/PanelHelp';
import { PointCloudHelp } from './PointCloud/PanelHelp';
import PanelHelp from './Console/PanelHelp';
import { VehicleVizPanelHelp } from './VehicleViz/PanelHelp';

export const getAllPanels = (t: TFunction | ((str: string) => string)): IPanelMetaInfo[] => [
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
