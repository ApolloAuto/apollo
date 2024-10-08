import React, { useMemo, useRef, useEffect, useState, useCallback } from 'react';
import Legend from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/Legend';
import * as echarts from 'echarts';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { useThemeContext } from '@dreamview/dreamview-theme';
import shortUUID from 'short-uuid';
import useStyle from './useStyle';
import { useViewBoxPosInfo } from './viewBoxPosContext';

export * from './util';
export * from './viewBoxPosContext';

function ChartBaseInner(props: {
    title: string;
    onRef?: any;
    yAxisName: string;
    onLegendClick: (key: string) => void;
    onCanvasRef: React.RefObject<HTMLDivElement>;
    legends: Array<{ name: string; color: string }>;
    autoHeight: boolean;
    onReset: () => void;
    onRefresh?: () => void;
    titleExtra?: React.ReactNode;
    className?: string;
}) {
    const {
        onRefresh,
        onReset,
        yAxisName,
        onLegendClick,
        onCanvasRef,
        legends,
        title,
        autoHeight,
        titleExtra,
        className,
        onRef,
    } = props;
    const { classes, cx } = useStyle();
    return (
        <div ref={onRef} className={cx(classes['moniter-item-container'], className || '')}>
            <div className={classes['moniter-item-title']}>
                {title || '-'}
                <div className={classes['moniter-item-title-extra']}>{titleExtra}</div>
            </div>
            <Legend legends={legends} onClick={onLegendClick} />
            <div className={classes['moniter-item-toolbox']}>
                <span className={classes['moniter-item-yaxis']}>{yAxisName}</span>
                <div className={classes['moniter-item-operate']}>
                    <Popover trigger='hover' content='Clear view'>
                        <IconPark name='IcResetView' onClick={onReset} />
                    </Popover>
                    {!!onRefresh && (
                        <Popover trigger='hover' content='Refresh view'>
                            <IconPark name='Refresh' onClick={onRefresh} className={classes['refresh-ic']} />
                        </Popover>
                    )}
                </div>
            </div>
            <div className={cx(classes['moniter-item-chart-container'], { autoHeight })}>
                <div className={classes['moniter-item-chart']} ref={onCanvasRef} />
            </div>
        </div>
    );
}

const ChartBaseMemo = React.memo(ChartBaseInner);

function addRotate(options: any, degree: number) {
    const r = { ...options };
    if (degree) {
        options.xAxis.nameGap = 70;
        options.xAxis.axisLabel.rotate = degree;
    } else {
        options.xAxis.nameGap = 30;
        options.xAxis.axisLabel.rotate = 0;
    }
    return r;
}

export default function ChartBase(props: {
    autoHeight?: boolean;
    title: string;
    options: any;
    onClear?: () => void;
    labelRotateBoundary?: number;
    titleExtra?: React.ReactNode;
    onRefresh?: () => void;
    className?: string;
}) {
    const { onPanelResize } = usePanelContext();
    const { options, onRefresh, title, labelRotateBoundary, autoHeight, titleExtra, className, onClear } = props;
    const [forceUpdate, setForceUpdate] = useState(0);
    const [uid] = useState(shortUUID.generate);
    const domRef = useRef<HTMLDivElement>();
    const [legends, setLegends] = useState<Array<{ name: string; color: string }>>([]);
    const chartRef = useRef<echarts.ECharts>();
    const [yAxisName, setYAxisName] = useState('');
    const rotateRef = useRef(0);
    const context = useViewBoxPosInfo();
    const isInView = useRef(true);
    const { tokens } = useThemeContext();
    const onForceUpdate = () => {
        setForceUpdate((prev) => prev + 1);
    };

    const memoLegends = useMemo(() => legends, [JSON.stringify(legends)]);

    const onLegendClick = useCallback((name: string) => {
        chartRef.current.dispatchAction({
            type: 'legendToggleSelect',
            name,
        });
    }, []);

    useEffect(() => {
        const timer = setInterval(() => {
            if (domRef.current.clientWidth) {
                chartRef.current = echarts.init(domRef.current, null, {
                    // renderer: 'svg',
                });
                clearInterval(timer);
            }
        }, 50);

        onPanelResize((width) => {
            chartRef.current?.resize();
            if (labelRotateBoundary) {
                rotateRef.current = width < labelRotateBoundary ? 45 : 0;
                onForceUpdate();
            }
        });

        return () => {
            chartRef.current?.dispose();
            clearInterval(timer);
        };
    }, []);

    function convertToPixel(opt: any) {
        try {
            opt.graphic.forEach((item: any) => {
                if (item.type === 'polygon') {
                    item.shape.points = item.shape.points
                        .filter((point: any) => !!point)
                        .map((point: any) => chartRef.current?.convertToPixel({ seriesIndex: 0 }, point))
                        .filter((point: any) => !!point);
                } else if (item.type === 'text') {
                    const pos = chartRef.current?.convertToPixel({ seriesIndex: 0 }, [item.x, item.y]);
                    if (pos) {
                        item.x = pos[0];
                        item.y = pos[1];
                    } else {
                        // 没有坐标则把内容置空
                        item.style = {};
                    }
                }
            });
        } catch (err) {
            //
        }
    }

    function addToolTip(opt: any, fontColor: any, colors: any) {
        opt.tooltip = {
            position(pos: any, params: any, dom: any, rect: any, size: any) {
                //  防止tooltip溢出面板后被隐藏
                const maxLeft = size.viewSize[0] - size.contentSize[0];
                const minLeft = 0;
                const posLeft = pos[0];
                return [Math.max(Math.min(posLeft, maxLeft), minLeft), pos[1]];
            },
            trigger: 'axis',
            formatter(params: any) {
                const tit = params[0].value?.[0]; // 这里假设第一个数据是标题

                // 自定义标题的样式和内容
                const formattedTitle = `<div style="font-weight: bold; font-size: 16px;">${tit}</div>`;

                // 以下是示例，您可以根据您的需求自定义 Tooltip 内容
                const content = params.reduce((result: string, param: any) => {
                    const seriesColor =
                        opt.series[param.seriesIndex]?.lineStyle?.color ||
                        opt.series.color ||
                        colors[param.seriesIndex];
                    const marker = `<span style="display:inline-block;margin-right:4px;border-radius:10px;width:10px;height:10px;background-color:${seriesColor};"></span>`;
                    return `${result}${marker} <span></span>${param.seriesName}: ${param.value?.[1] || '-'}<br>`;
                }, '');
                return formattedTitle + content;
            },
            padding: [
                10, // 上
                16, // 右
                10, // 下
                16, // 左
            ],
            backgroundColor: 'rgba(255,255,255,0.21)',
            extraCssText: 'box-shadow:  0px 6px 12px 2px rgba(0,0,0,0.1);backdrop-filter: blur(5px);',
            borderColor: 'transparent',
            textStyle: {
                color: fontColor,
            },
        };
    }

    function addColor(opt: any, color: any) {
        opt.color = color;
    }

    const setOption = (opt: any) => {
        if (chartRef.current && isInView.current) {
            chartRef.current?.setOption(opt, { replaceMerge: ['dataset', 'graphic'] });
        }
    };

    const translateYAxisName = (opt: any) => {
        // 标题位置不对，自己用dom覆盖掉
        setYAxisName(opt.yAxis.name);
        delete opt.yAxis.name;
    };

    const onReset = useCallback(() => {
        if (onClear) {
            onClear();
        } else {
            chartRef.current.dispatchAction({
                type: 'dataZoom',
                start: 0,
                end: 100,
            });
        }
    }, []);

    useEffect(() => {
        if (options && isInView.current) {
            const opt = {
                ...options,
            };

            // x轴刻度名称倾斜
            if (labelRotateBoundary) {
                addRotate(opt, rotateRef.current);
            }
            // 多边形坐标转换绘制
            if (opt.graphic) {
                convertToPixel(opt);
            }

            if (opt.yAxis.name) {
                translateYAxisName(opt);
            }

            addToolTip(opt, tokens.components.pncMonitor.toolTipColor, tokens.components.pncMonitor.chartColors);
            addColor(opt, tokens.components.pncMonitor.chartColors);

            setOption(opt);
            setLegends((opt.series || []).map((item: any) => ({ name: item.name, color: item.lineStyle?.color })));
        }
    }, [options, forceUpdate, tokens]);

    const unDo = useRef({
        unDo: () => false,
    });

    const onRef = useCallback((elem: any) => {
        if (context) {
            unDo.current.unDo = context.regisitScrollEvent(uid, elem, (flag: boolean) => {
                isInView.current = flag;
            });
        }
    }, []);

    useEffect(
        () => () => {
            unDo.current.unDo() as any;
        },
        [],
    );

    return (
        <ChartBaseMemo
            onRef={onRef}
            autoHeight={autoHeight}
            onReset={onReset}
            onRefresh={onRefresh}
            onLegendClick={onLegendClick}
            title={title}
            yAxisName={yAxisName}
            onCanvasRef={domRef}
            legends={memoLegends}
            titleExtra={titleExtra}
            className={className}
        />
    );
}
