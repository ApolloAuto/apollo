import React, { useCallback, useState, useEffect } from 'react';
import debounce from 'lodash/debounce';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { OBJECT_STORE_TYPE } from '@dreamview/dreamview-core/src/services/api/types';
import shortUUID from 'short-uuid';
import { Modal } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import lodashCloneDeep from 'lodash/cloneDeep';
import { initChartValue, IChartListItem } from '../const';

export function useChartConfigManager(panelId: string, onInit: (data: Array<IChartListItem>) => void) {
    const [chartConfigList, setChartConfigList] = useState<Array<IChartListItem>>([]);
    const [activeChartConfig, setActiveChartConfig] = useState<IChartListItem>({
        uid: '',
        value: undefined,
    });
    const { t: tChartEditing } = useTranslation('chartEditing');

    const { mainApi } = useWebSocketServices();

    const putObjectStore = useCallback(
        debounce(
            (value: Array<IChartListItem>) => mainApi.putObjectStore({ type: OBJECT_STORE_TYPE.CHART, panelId, value }),
            300,
        ),
        [],
    );

    const onAddNewChart = useCallback(() => {
        const newChart = { uid: shortUUID.generate(), value: initChartValue() };
        const nextChartConfigList = [...chartConfigList, newChart];
        setActiveChartConfig(newChart);
        setChartConfigList(nextChartConfigList);
        putObjectStore(nextChartConfigList);
    }, [chartConfigList]);

    const resetActiveChartConfig = () => {
        setActiveChartConfig({
            uid: '',
            value: undefined,
        });
    };

    const onDeleteChart = useCallback(
        (chart: IChartListItem) => {
            Modal.confirm({
                content: tChartEditing('deleteConfirmText', { chartName: chart.value?.title }),
                icon: null,
                okText: tChartEditing('ok'),
                cancelText: tChartEditing('cancel'),
                zIndex: 100000,
                onOk() {
                    const nextChartConfigList = chartConfigList.filter((item) => item.uid !== chart.uid);
                    setChartConfigList(nextChartConfigList);
                    resetActiveChartConfig();
                    return mainApi.putObjectStore({
                        type: OBJECT_STORE_TYPE.CHART,
                        panelId,
                        value: nextChartConfigList,
                    });
                },
            });
        },
        [chartConfigList],
    );

    const changeActiveChartConfig = useCallback((config: IChartListItem) => {
        setActiveChartConfig(config);
    }, []);

    const updateChartConfigList = useCallback(
        (config: Array<IChartListItem>) => {
            setChartConfigList(lodashCloneDeep(config));
            putObjectStore(config);
        },
        [chartConfigList],
    );

    const initChartConfig = () => {
        mainApi.getObjectStore<Array<IChartListItem>>({ type: OBJECT_STORE_TYPE.CHART, panelId }).then((r) => {
            setChartConfigList(r);
            if (onInit) {
                onInit(r);
            }
        });
    };

    useEffect(() => {
        initChartConfig();
    }, []);

    return {
        onDeleteChart,
        onAddNewChart,
        activeChartConfig,
        changeActiveChartConfig,
        resetActiveChartConfig,
        chartConfigList,
        updateChartConfigList,
    };
}
