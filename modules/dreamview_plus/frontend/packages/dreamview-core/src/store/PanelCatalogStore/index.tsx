/* eslint-disable no-return-await */
import React, { PropsWithChildren, useEffect, useMemo, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { useImagePrak } from '@dreamview/dreamview-ui';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { createDymaticPanel } from '../../components/panels/base/PanelFactory';
import { getPanelTypeByPanelId } from '../../util/layout';
import { getAllPanels } from '../../components/panels';
import useWebSocketServices from '../../services/hooks/useWebSocketServices';

export interface IPanelCatalogContext {
    allPanel: IPanelMetaInfo[];
    panelCatalog: {
        get: (panelType: string) => IPanelMetaInfo;
    };
    panelComponents: {
        get: (panelType: string) => React.LazyExoticComponent<any>;
    };
    panelToolBar: {
        get: (panelType: string) => React.LazyExoticComponent<any>;
    };
}

const panelCatalogContext = React.createContext<IPanelCatalogContext>(null);

export function usePanelCatalogContext() {
    return React.useContext(panelCatalogContext);
}

export function PanelCatalogProvider(props: PropsWithChildren) {
    const { t } = useTranslation('panels');
    const { mainApi, isMainConnected } = useWebSocketServices();

    const imgSrc = useImagePrak([
        'console_hover_illustrator',
        'module_delay_hover_illustrator',
        'vehicle_viz_hover_illustrator',
        'camera_view_hover_illustrator',
        'pointcloud_hover_illustrator',
        'dashboard_hover_illustrator',
        'image_Charts',
        'Components',
        'terminal_Illustrator',
        'panel_chart',
    ]);

    const [allPanel, setAllPanel] = useState([]);

    useEffect(() => {
        if (isMainConnected) {
            mainApi
                .getPanelPluginInitData()
                .then((remotePanels) =>
                    remotePanels.reduce((result: any, panels: any) => [...result, ...panels.value], []),
                )
                .then((remotePanels) => getAllPanels(remotePanels, t, imgSrc))
                .catch(() => getAllPanels([], t, imgSrc))
                .then((panels) => {
                    setAllPanel(panels);
                });
        }
    }, [isMainConnected, mainApi, t, imgSrc]);

    const panelUtils = useMemo(() => {
        const panelComponents = new Map( // 新增字段 panelInfo.originType 面板类型 remote | local
            allPanel.map((panelInfo) => [
                panelInfo.type,
                panelInfo.originType === 'local'
                    ? React.lazy(panelInfo.module)
                    : createDymaticPanel(React.lazy(panelInfo.module)),
            ]),
        );
        const panelToolBar = new Map(
            allPanel
                .filter((panel) => panel.renderToolbar)
                .map((panelInfo) => [panelInfo.type, React.lazy(panelInfo.renderToolbar)]),
        );
        return {
            panelComponents: {
                get(panelId: string) {
                    // panelId is like `${type}!${uuid}`
                    const panelType = getPanelTypeByPanelId(panelId);
                    return panelComponents.get(panelType);
                },
            },
            panelToolBar: {
                get(panelId: string) {
                    // panelId is like `${type}!${uuid}`
                    const panelType = getPanelTypeByPanelId(panelId);
                    return panelToolBar.get(panelType);
                },
            },
        };
    }, [allPanel]);

    const values: IPanelCatalogContext = useMemo(() => {
        const panelCatalog = new Map(allPanel.map((panelInfo) => [panelInfo.type, panelInfo]));
        return {
            panelCatalog: {
                get(panelId: string) {
                    // panelId is like `${type}!${uuid}`
                    const panelType = getPanelTypeByPanelId(panelId);
                    return panelCatalog.get(panelType);
                },
            },
            allPanel,
            ...panelUtils,
        };
    }, [allPanel, panelUtils]);

    return <panelCatalogContext.Provider value={values}>{props.children}</panelCatalogContext.Provider>;
}
