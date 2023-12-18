/* eslint-disable no-return-await */
import React, { PropsWithChildren, useMemo } from 'react';
import { useTranslation } from 'react-i18next';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { getPanelTypeByPanelId } from '../../util/layout';
import { getAllPanels } from '../../components/panels';

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

    const allPanel = useMemo(() => getAllPanels(t), [t]);

    const panelUtils = useMemo(() => {
        const panelComponents = new Map(allPanel.map((panelInfo) => [panelInfo.type, React.lazy(panelInfo.module)]));
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
    }, []);

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
