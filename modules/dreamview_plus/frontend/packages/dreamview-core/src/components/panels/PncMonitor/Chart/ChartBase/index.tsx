import React, { useMemo, useRef, useEffect, useState, useCallback, RefObject } from 'react';
import Legend from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/Legend';
import * as echarts from 'echarts';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import { IconIcResetView } from '@dreamview/dreamview-ui';
import CustomPopover from '@dreamview/dreamview-core/src/components/CustomPopover';
import useStyle from './useStyle';

export * from './util';

function ChartBase(props: {
    title: string;
    yAxisName: string;
    onLegendClick: (key: string) => void;
    onCanvasRef: React.RefObject<HTMLDivElement>;
    legends: string[];
    autoHeight: boolean;
    onReset: () => void;
}) {
    const { onReset, yAxisName, onLegendClick, onCanvasRef, legends, title, autoHeight } = props;
    const { classes, cx } = useStyle();
    return (
        <div className={classes['moniter-item-container']}>
            <div className={classes['moniter-item-title']}>{title}</div>
            <Legend legends={legends} onClick={onLegendClick} />
            <div className={classes['moniter-item-toolbox']}>
                <span className={classes['moniter-item-yaxis']}>{yAxisName}</span>
                <div className={classes['moniter-item-operate']}>
                    <CustomPopover trigger='hover' content='Reset view'>
                        <IconIcResetView onClick={onReset} />
                    </CustomPopover>
                </div>
            </div>
            <div className={cx(classes['moniter-item-chart-container'], { autoHeight })}>
                <div className={classes['moniter-item-chart']} ref={onCanvasRef} />
            </div>
        </div>
    );
}

const ChartBaseMemo = React.memo(ChartBase);

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
export default function ChartBaseProvider(props: {
    autoHeight?: boolean;
    title: string;
    options: any;
    labelRotateBoundary?: number;
}) {
    const { onPanelResize } = usePanelContext();
    const { options, title, labelRotateBoundary, autoHeight } = props;
    const [forceUpdate, setForceUpdate] = useState(0);
    const domRef = useRef<HTMLDivElement>();
    const [legends, setLegends] = useState([]);
    const chartRef = useRef<echarts.ECharts>();
    const [yAxisName, setYAxisName] = useState('');
    const rotateRef = useRef(0);

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
                chartRef.current = echarts.init(domRef.current);
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
                item.shape.points = item.shape.points
                    .filter((point: any) => !!point)
                    .map((point: any) => chartRef.current?.convertToPixel({ seriesIndex: 0 }, point))
                    .filter((point: any) => !!point);
            });
        } catch (err) {
            //
        }
    }

    const setOption = (opt: any) => {
        if (chartRef.current) {
            chartRef.current?.setOption(opt, { replaceMerge: ['dataset', 'graphic'] });
        }
    };

    const translateYAxisName = (opt: any) => {
        // 标题位置不对，自己用dom覆盖掉
        setYAxisName(opt.yAxis.name);
        delete opt.yAxis.name;
    };

    const onReset = useCallback(() => {
        chartRef.current.dispatchAction({
            type: 'dataZoom',
            start: 0,
            end: 100,
        });
    }, []);

    useEffect(() => {
        if (options) {
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

            setOption(opt);
            setLegends((opt.series || []).map((item: any) => item.name));
        }
    }, [options, forceUpdate]);

    return (
        <ChartBaseMemo
            autoHeight={autoHeight}
            onReset={onReset}
            onLegendClick={onLegendClick}
            title={title}
            yAxisName={yAxisName}
            onCanvasRef={domRef}
            legends={memoLegends}
        />
    );
}
