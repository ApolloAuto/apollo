import React, { useCallback } from 'react';
import shortUUID from 'short-uuid';
import { genereatePanelId } from '@dreamview/dreamview-core/src/util/layout';
import { PanelType } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePanelLayoutStore, addPanelFromOutside } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { OBJECT_STORE_TYPE } from '@dreamview/dreamview-core/src/services/api/types';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { usePanelCatalogContext } from '@dreamview/dreamview-core/src/store/PanelCatalogStore';
import { initChartValue } from '../const';

export function useAddNewChartPanel() {
    const { mainApi } = useWebSocketServices();
    const [, dispatch] = usePanelLayoutStore();
    const [hmi] = usePickHmiStore();
    const { panelCatalog } = usePanelCatalogContext();

    const onAddNew = useCallback(() => {
        const uid = shortUUID.generate();
        const newPanelId = genereatePanelId(PanelType.Charts);
        const configList = [
            {
                uid,
                // 这里是chart配置数据，根据需要自行替换相应的字段
                value: initChartValue(),
            },
        ];
        mainApi
            .putObjectStore({
                type: OBJECT_STORE_TYPE.CHART,
                panelId: newPanelId,
                value: configList,
            })
            .then(() => {
                dispatch(
                    addPanelFromOutside({
                        mode: hmi.currentMode,
                        panelId: newPanelId,
                        path: [],
                        position: 'left',
                        originPanelConfig: panelCatalog.get(PanelType.Charts),
                    }),
                );
            });
    }, []);

    return onAddNew;
}
