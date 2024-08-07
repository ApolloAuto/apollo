import React, { useMemo, useCallback, useEffect } from 'react';
import { OperatePopover, IconPark } from '@dreamview/dreamview-ui';
import CustomScroll from '@dreamview/dreamview-core/src/components/CustomScroll';
import { useTranslation } from 'react-i18next';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import { FieldData } from 'rc-field-form/lib/interface';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import cloneDeep from 'lodash/cloneDeep';
import BlueButton from './BlueButton';
import Panel from '../base/Panel';
import ChartEditing from './ChartEditing';
import useStyle from './useStyle';
import { IChartListItem, KEY } from './const';
import { Div } from './Div';
import CustomChartBase from './CustomChartBase';
import { useAutoPopoverPos, useChartConfigManager, useChartDataManager, ChartDataManagerProvider } from './hoc';
import { useCreateViewBoxPosInfo, viewBoxPosInfoContext } from '../PncMonitor/Chart/ChartBase';

function ChartsInner() {
    const { panelId } = usePanelContext();
    const { classes, cx } = useStyle();
    const { t } = useTranslation('panels');
    const { metadata } = useWebSocketServices();
    const { channelChangeHandler } = useChartDataManager();

    const {
        chartConfigList,

        onDeleteChart,
        onAddNewChart,
        updateChartConfigList,

        activeChartConfig,
        changeActiveChartConfig,
        resetActiveChartConfig,
    } = useChartConfigManager(panelId, (config) => {
        channelChangeHandler(
            config.reduce(
                (r, chart) => [
                    ...r,
                    ...chart.value.lineList
                        .filter((line) => line.lineChannel)
                        .reduce((result, item) => [...result, `${item.lineDataName}!${item.lineChannel}`], []),
                ],
                [],
            ),
        );
    });

    const isChartCountMax = chartConfigList.length >= 10;

    const isPopoverOpen = !!activeChartConfig.uid;

    // 自动调整Popover位置，防止popover出现在屏幕外
    useAutoPopoverPos(classes['fixed-left'], isPopoverOpen);

    const onFormChange = useCallback(
        (propChartItem: IChartListItem, fieldsList: FieldData[]) => {
            const nextChartConfigList = cloneDeep(chartConfigList);
            const changeItem = nextChartConfigList.find((item) => item.uid === propChartItem.uid);
            changeItem.value = propChartItem.value;

            const hasChannelChange = fieldsList.some((field) =>
                field.name.some((fieldName: any) =>
                    [KEY.lineChannel, KEY.lineChannelX, KEY.lineChannelY].includes(fieldName),
                ),
            );

            if (hasChannelChange) {
                channelChangeHandler(
                    // 获取所有需要订阅的channel
                    nextChartConfigList.reduce(
                        (r, chart) => [
                            ...r,
                            ...chart.value.lineList
                                .filter((line) => line.lineChannel)
                                .reduce((result, item) => [...result, `${item.lineDataName}!${item.lineChannel}`], []),
                        ],
                        [],
                    ),
                );
            }

            updateChartConfigList(nextChartConfigList);
        },
        [chartConfigList],
    );

    // 从metadata中获取channel列表
    const channelList = useMemo(
        () =>
            metadata
                .find((data) => data.dataName === StreamDataNames.Cyber)
                ?.channels.map((subItem: any) => ({
                    label: subItem.channelName,
                    value: subItem.channelName,
                    dataName: StreamDataNames.Cyber,
                    channelName: subItem.channelName,
                    msgType: subItem.msgType,
                    protoPath: subItem.protoPath,
                })) || [],
        [metadata],
    );

    const popOverContent = (
        <ChartEditing
            onCloseClick={resetActiveChartConfig}
            channelList={channelList}
            onDeleteChart={onDeleteChart}
            // key变更时强制更新ChartEditing组件
            key={activeChartConfig.uid}
            activeChartConfig={activeChartConfig}
            onChange={onFormChange}
        />
    );

    const { onRef, contextValue } = useCreateViewBoxPosInfo();

    return (
        <OperatePopover
            rootClassName={cx('js-chart-popover')}
            placement='right'
            destroyTooltipOnHide
            open={isPopoverOpen}
            content={popOverContent}
        >
            <viewBoxPosInfoContext.Provider value={contextValue}>
                <CustomScroll ref={onRef} className={cx(classes['charts-container'], 'js-chart-container')}>
                    {chartConfigList.map((item) => (
                        <CustomChartBase
                            onClick={changeActiveChartConfig}
                            key={item.uid}
                            config={item}
                            isActive={item.uid === activeChartConfig?.uid}
                        />
                    ))}
                    <Div rif={!isChartCountMax} className={classes['charts-operation']}>
                        <BlueButton onClick={onAddNewChart}>
                            <IconPark name='IcAddPanel' />
                            {t('newChart')}
                        </BlueButton>
                    </Div>
                </CustomScroll>
            </viewBoxPosInfoContext.Provider>
        </OperatePopover>
    );
}

function ChartWithProvider() {
    return (
        <ChartDataManagerProvider>
            <ChartsInner />
        </ChartDataManagerProvider>
    );
}

ChartWithProvider.displayName = 'Charts';

function Index(props: any) {
    const Component = useMemo(
        () =>
            Panel({
                PanelComponent: ChartWithProvider,
                panelId: props.panelId,
            }),
        [],
    );

    return <Component {...props} />;
}

export default React.memo(Index);
