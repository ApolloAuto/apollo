import React, { useCallback, useState, useEffect } from 'react';
import debounce from 'lodash/debounce';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { OBJECT_STORE_TYPE } from '@dreamview/dreamview-core/src/services/api/types';
import shortUUID from 'short-uuid';
import { Modal } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import lodashCloneDeep from 'lodash/cloneDeep';
import { initChartValue, IChartListItem } from '../const';

/**
 * @description
 * 使用hooks管理图表配置，包括添加新图表、删除图表、更新图表列表等操作。
 *
 * @param {string} panelId - 面板ID，用于区分不同的面板。
 * @param {Function} onInit - 初始化时调用，传入一个数组参数，包含当前面板上所有的图表配置项。
 *
 * @returns {object} 返回一个对象，包含以下属性：
 * - {Function} onDeleteChart - 删除指定图表的函数，接收一个图表配置项作为参数。
 * - {Function} onAddNewChart - 添加新图表的函数，无参数。
 * - {IChartListItem} activeChartConfig - 当前选中的图表配置项。
 * - {Function} changeActiveChartConfig - 切换选中的图表配置项的函数，接收一个图表配置项作为参数。
 * - {Function} resetActiveChartConfig - 重置选中的图表配置项的函数，无参数。
 * - {Array<IChartListItem>} chartConfigList - 当前面板上所有的图表配置项列表。
 * - {Function} updateChartConfigList - 更新图表配置项列表的函数，接收一个图表配置项数组作为参数。
 */
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
            (value: Array<IChartListItem>) =>
                mainApi.putChartObjectStore({ type: OBJECT_STORE_TYPE.CHART, panelId, value }),
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
                    return mainApi.putChartObjectStore({
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
        mainApi.getChartObjectStore<Array<IChartListItem>>({ type: OBJECT_STORE_TYPE.CHART, panelId }).then((r) => {
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
