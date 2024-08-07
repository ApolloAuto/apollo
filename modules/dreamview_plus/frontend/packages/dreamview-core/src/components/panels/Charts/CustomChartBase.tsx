import React, { useEffect, useState, useRef, useCallback } from 'react';
import { IconPark } from '@dreamview/dreamview-ui';
import ChartBase, { initOptions } from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';
import { makeStyles } from '@dreamview/dreamview-theme';
import differenceBy from 'lodash/differenceBy';
import lodashGet from 'lodash/get';
import cloneDeep from 'lodash/cloneDeep';
import intersectionBy from 'lodash/intersectionBy';
import { debounce } from 'lodash';
import lodashLast from 'lodash/last';
import { ILineConfig, HIDDEL_OPTION, IChartListItem, KEY } from './const';
import { useChartDataManager } from './hoc';

interface CustomChartBaseProps {
    config: IChartListItem;
    onClick?: (config: CustomChartBaseProps['config']) => void;
    isActive: boolean;
}

function getOption(data: Record<string, any>, config: CustomChartBaseProps['config']) {
    const { dataset, series } = Object.entries(data).reduce(
        (result, [key, value]: any) => {
            const lineConfig = config.value.lineList.find((item) => item.uid === key);

            if (!lineConfig || lineConfig.lineHidden === HIDDEL_OPTION.HIDE) {
                return result;
            }

            return {
                dataset: [...result.dataset, { id: key, source: [['x', 'y'], ...value] }],
                series: [
                    ...result.series,
                    {
                        datasetId: key,
                        smooth: true,
                        name: lineConfig.lineName,
                        type: 'line',
                        showSymbol: false,
                        lineStyle: {
                            color: lineConfig.lineColor,
                            width: lineConfig.lineWidth,
                        },
                        encode: {
                            x: 'x',
                            y: 'y',
                        },
                    },
                ],
            };
        },
        {
            dataset: [],
            series: [],
        },
    );
    return initOptions({
        scale: true,
        dataset,
        series,
        xAxis: {
            type: 'value',
            name: config.value.xAxisName,
        },
        yAxis: {
            name: config.value.yAxisName,
        },
    });
}

const useStyle = makeStyles((theme) => ({
    'chart-item': {
        marginBottom: theme.tokens.margin.speace,
    },
    'chart-item-active': {
        border: `1px solid ${theme.tokens.colors.brand3}`,
    },
}));

type noopFunc = () => void;

function pushIfChange(arr: any, item: any) {
    const lastTarget: any = lodashLast(arr) || [];
    if (item[0] !== lastTarget[0] && item[1] !== lastTarget[1]) {
        arr.push(item);
    }
}

const ifChannelPathHasArray = (path: string) => /\[0\]/.test(path);
function CustomChartBase(prop: CustomChartBaseProps) {
    const { config, onClick: propOnClick, isActive } = prop;
    const [option, setOptions] = useState<any>();
    const { classes, cx } = useStyle();
    const { consumerChannelData } = useChartDataManager();
    const unSubscribeCollection = useRef<Record<string, noopFunc>>({});
    const cacheChartValueRef = useRef<Record<string, any>>({});

    const onClick = () => {
        propOnClick(config);
    };

    const titleExtra = <IconPark name='IcCompile' onClick={onClick} />;

    const updatelineData = useCallback((line: ILineConfig, lineData: any) => {
        const channelX = line[KEY.lineChannelX];
        const channelY = line[KEY.lineChannelY];
        if (!channelX || !channelY) {
            return;
        }
        // channel存在两种情况
        // 1. channel中不包含数组 a.b
        // 2. channel中包含数组 a.b.[0].c
        if (!cacheChartValueRef.current[line.uid]) {
            cacheChartValueRef.current[line.uid] = [];
        }
        const isChannelXHasArrayPath = ifChannelPathHasArray(channelX);
        const isChannelYHasArrayPath = ifChannelPathHasArray(channelY);
        // 则认为用户想要展示的是实时数据，进行数据缓存
        const needCache = !isChannelXHasArrayPath && !isChannelYHasArrayPath;
        if (needCache) {
            // 需要缓存时默认用户选择的是实时的数据
            const prevChartValue = cacheChartValueRef.current[line.uid];
            const xValue = lodashGet(lineData, channelX);
            const yValue = lodashGet(lineData, channelY);
            if (xValue) {
                pushIfChange(prevChartValue, [xValue, yValue]);
            }
            return;
        }
        // 从 'a.b.[0].c' 中获取到 .[0].c 作为key
        const xKey = channelX.match(/\.\[0\]\.[A-z]+$/)?.[0];
        const yKey = channelY.match(/\.\[0\]\.[A-z]+$/)?.[0];
        // 不需要缓存时默认用户选择的是预测的数据
        // 默认认为用户想展示同一数组内的两个字段
        // 如果任何一个数据没找到，认为是异常情况，不做处理
        if (!xKey || !yKey) {
            return;
        }
        const formatXKeyToRegexp = xKey.replace('.', '\\.').replace('[', '\\[').replace(']', '\\]');
        // 根据channelX获取数据路径
        const arrPath = channelX.replace(new RegExp(`${formatXKeyToRegexp}$`), '');
        const array = lodashGet(lineData, arrPath);

        cacheChartValueRef.current[line.uid] = array
            .map((item: any) => [item[xKey.replace('.[0].', '')], item[yKey.replace('.[0].', '')]])
            .filter(([xValue]: any) => !!xValue);
    }, []);

    const prefConf = useRef<IChartListItem>();

    useEffect(() => {
        const debounceUpdate = debounce((chartoption: any) => {
            setOptions(chartoption);
        }, 10);
        const prev = cloneDeep(prefConf.current?.value?.lineList || []);
        const next = config?.value?.lineList || [];
        {
            // 曲线被移除时取消订阅
            const uselessChannels = differenceBy(
                prev,
                next,
                (line: ILineConfig) => `${line.uid}!${line.lineDataName}!${line.lineChannel}`,
            ).filter((line) => line.lineDataName && line.lineChannel);
            uselessChannels.forEach((line) => {
                unSubscribeCollection.current[line.uid]?.();
                // 删除chart图表数据
                delete cacheChartValueRef.current[line.uid];
            });
            if (uselessChannels.length) {
                prefConf.current = cloneDeep(config);
            }
        }

        {
            // 新增订阅
            const newChannels = differenceBy(
                next,
                prev,
                (line: ILineConfig) => `${line.uid}!${line.lineDataName}!${line.lineChannel}`,
            ).filter((line) => line.lineDataName && line.lineChannel);
            newChannels.forEach((line) => {
                unSubscribeCollection.current[line.uid] = consumerChannelData(
                    line.lineDataName,
                    line.lineChannel,
                    (lineData: any) => {
                        updatelineData(line, lineData);
                        debounceUpdate(getOption(cacheChartValueRef.current, config));
                    },
                );
            }, []);
            if (newChannels.length) {
                prefConf.current = cloneDeep(config);
            }
        }
        {
            // 未变更的channel需要更新订阅函数，因为config变了
            const unChangedChannels = intersectionBy(
                next,
                prev,
                (line: ILineConfig) => `${line.uid}!${line.lineDataName}!${line.lineChannel}`,
            );
            unChangedChannels.forEach((line) => {
                unSubscribeCollection.current[line.uid]?.();
                unSubscribeCollection.current[line.uid] = consumerChannelData(
                    line.lineDataName,
                    line.lineChannel,
                    (lineData: any) => {
                        updatelineData(line, lineData);
                        debounceUpdate(getOption(cacheChartValueRef.current, config));
                    },
                );
            });
        }
        setOptions(getOption(cacheChartValueRef.current, config));
    }, [config]);

    useEffect(
        () => () => {
            Object.values(unSubscribeCollection.current).forEach((unscribe) => {
                unscribe();
            });
        },
        [],
    );

    const onRefresh = useCallback(() => {
        cacheChartValueRef.current = {};
        setOptions(getOption(cacheChartValueRef.current, config));
    }, [config]);

    return (
        <ChartBase
            className={cx(classes['chart-item'], { [classes['chart-item-active']]: isActive })}
            titleExtra={titleExtra}
            onRefresh={onRefresh}
            labelRotateBoundary={550}
            title={config.value.title}
            options={option}
        />
    );
}
export default React.memo(CustomChartBase);
