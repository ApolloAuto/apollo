/* eslint-disable import/no-webpack-loader-syntax */
// @ts-ignore
import WorkerPoolManagerWorker from 'worker-loader!./WorkerPoolManager.worker';
import React, {useCallback, useRef, useState} from 'react';
import {Meta, StoryObj} from '@storybook/react';
import WorkerPoolManager from '@dreamview/dreamview-core/src/worker/WorkerPoolManager';
import Logger from '@dreamview/log';
import {Button} from '@dreamview/dreamview-ui';
import {WorkerFactory} from './WorkerFactory';
import ReactECharts from 'echarts-for-react';

const logger = Logger.getInstance('WorkerPoolManager');

class TaskSimulator {
    private baselineTaskCount: number;
    private variationFrequency: number;
    private count: number;
    private stepSize: number;

    constructor() {
        this.baselineTaskCount = 8; // 初始基线任务数
        this.variationFrequency = 10; // 变化频率
        this.count = 0;
        this.stepSize = 15; // 阶跃大小
    }

    public getRandomTaskCount(): number {
        this.count++;
        if (this.count % this.variationFrequency === 0) {
            // 模拟阶跃，任务数量大幅增加或减少
            const stepChange = (Math.random() < 0.5 ? -1 : 1) * this.stepSize;
            this.baselineTaskCount += stepChange;
        }

        // 任务数量回归到某个稳态的水平
        this.baselineTaskCount = Math.min(Math.max(this.baselineTaskCount, 10), 90);

        // 模拟在基线任务数量附近上下波动
        const fluctuation = Math.floor(Math.random() * 20) - 10; // 在 -10 到 10 之间波动
        return this.baselineTaskCount + fluctuation;
    }
}


// 模拟任务耗时
class TimeSimulator {
    private baselineProcessingTime: number;
    private timeVariationFrequency: number;

    constructor() {
        this.baselineProcessingTime = 24; // 基线处理时间
        this.timeVariationFrequency = 30; // 长耗时任务出现频率
    }

    public getRandomProcessingTime(): number {
        if (Math.random() < 1 / this.timeVariationFrequency) {
            // 偶尔出现长耗时任务
            return Math.floor(Math.random() * 50);
        }
        // 大部分时间任务耗时较短
        return Math.floor(Math.random() * 10) + this.baselineProcessingTime;
    }
}

const taskSimulator = new TaskSimulator();
const timeSimulator = new TimeSimulator();

// 0-100随机任务数
const getRandomTaskCount = () => taskSimulator.getRandomTaskCount();

// 0-70ms随机任务耗时
const getRandomProcessingTime = () => timeSimulator.getRandomProcessingTime();

function WorkerPoolManagerStory() {
    const logElementRef = useRef(null);
    const workerPoolManagerRef = useRef<WorkerPoolManager<number>>();

    const [chartData, setChartData] = useState([]);

    // 采集定时器
    const intervalId = useRef<number>();

    const start = useCallback(() => {
        // @ts-ignore
        window.setLogLevel('WorkerPoolManager', 'debug');
        if (logElementRef.current) {
            logger.setLogElement(logElementRef.current);
        }

        workerPoolManagerRef.current = new WorkerPoolManager<number>({
            name: 'WorkerPoolManagerStory',
            workerFactory: new WorkerFactory<number>(() => new WorkerPoolManagerWorker()),
        });

        // @ts-ignore
        intervalId.current = setInterval(() => {
            workerPoolManagerRef.current?.visualize();

            workerPoolManagerRef.current.adjustWorkerSizeWithPID();
                // 获取当前线程数量和任务数量
                const currentThreads = workerPoolManagerRef.current?.getWorkerCount();
                const currentTasks = workerPoolManagerRef.current?.getTaskCount();

                // 更新 chartData
                setChartData((prevData) => [
                    ...prevData,
                    {
                        time: new Date(),
                        threadCount: currentThreads,
                        taskCount: currentTasks,
                        totalTaskCount: 0,
                    },
                ]);
        }, 100);

        return () => {
            workerPoolManagerRef.current?.terminateAllWorkers();
            clearInterval(intervalId.current);
        };
    }, []);

    const getOption = () => ({
        title: {
            text: '线程调度算法可视化',
        },
        tooltip: {
            trigger: 'axis',
        },
        legend: {
            data: ['线程数',
                // '实际任务数',
                '队列任务数'
            ],
        },
        xAxis: {
            type: 'time',
            boundaryGap: false,
            axisLabel: {
                formatter: (value: string | number | Date) => new Date(value).toLocaleTimeString(),
            },
        },
        yAxis: {
            type: 'value',
        },
        // 支持缩放
        dataZoom: [
            {
                type: 'slider',
                start: 0,
                end: 100,
            },
        ],
        series: [
            {
                name: '线程数',
                type: 'line',
                data: chartData.map((item) => [item.time, item.threadCount]),
                // 平滑
                smooth: true,
            },
            // {
            //     name: '实际任务数',
            //     type: 'line',
            //     data: chartData.map((item) => [item.time, item.totalTaskCount || 0]),
            // },
            {
                name: '队列任务数',
                type: 'line',
                data: chartData.map((item) => [item.time, item.taskCount]),
                smooth: true,
            },
        ],
    });

    return (
        <div>
            <h1>WorkerPool Manager Story</h1>
            <Button onClick={start}>Start</Button>
            <Button onClick={() => clearInterval(intervalId.current)}>Stop</Button>
            <Button
                onClick={() => {
                    workerPoolManagerRef.current
                        .dispatchTask({
                            type: 'add',
                            payload: getRandomProcessingTime(),
                        })
                        .then((res) => {
                            logger.info(res);
                        });
                }}
            >
                add one task
            </Button>
            <Button
                onClick={() => {
                    const promise = Array.from({length: 10}).map((_, index) =>
                        workerPoolManagerRef.current.dispatchTask({
                            type: 'add',
                            payload: getRandomProcessingTime(),
                        }),
                    );
                    Promise.all(promise).then((res) => {
                        logger.info(res);
                    });
                }}
            >
                add 10 task
            </Button>
            <Button
                onClick={() => {
                    const promise = Array.from({length: 100}).map((_, index) =>
                        workerPoolManagerRef.current.dispatchTask({
                            type: 'add',
                            payload: getRandomProcessingTime(),
                        }),
                    );
                    Promise.all(promise).then((res) => {
                        logger.info(res);
                    });
                }}
            >
                add 100 task
            </Button>
            <Button
                onClick={() => {
                    const i = setInterval(
                        () => {
                            const taskCount = getRandomTaskCount();
                            const promise = Array.from({length: taskCount}).map((_) =>
                                workerPoolManagerRef.current.dispatchTask({
                                    type: 'add',
                                    payload: getRandomProcessingTime(),
                                }),
                            );
                            // 获取当前线程数量和任务数量
                            const currentThreads = workerPoolManagerRef.current?.getWorkerCount();
                            const currentTasks = workerPoolManagerRef.current?.getTaskCount();

                            // 更新 chartData
                            setChartData((prevData) => [
                                ...prevData,
                                {
                                    time: new Date(),
                                    threadCount: currentThreads,
                                    taskCount: currentTasks,
                                    totalTaskCount: taskCount,
                                },
                            ]);
                        },
                        100,
                    )

                    setTimeout(
                        () => {
                            clearInterval(i)
                        },
                        10000
                    )
                }}
            >
                random task count 50-100/100ms
            </Button>
            <Button
                onClick={() => {
                    workerPoolManagerRef.current.adjustWorkerSize(10);
                }}
            >
                adjustWorkerSize 10
            </Button>
            <Button
                onClick={() => {
                    workerPoolManagerRef.current.adjustWorkerSize(20);
                }}
            >
                adjustWorkerSize 20
            </Button>
            <Button
                onClick={() => {
                    workerPoolManagerRef.current.adjustWorkerSize(30);
                }}
            >
                adjustWorkerSize 30
            </Button>
            <Button
                onClick={() => {
                    workerPoolManagerRef.current.terminateIdleWorkers();
                }}
            >
                terminateIdleWorkers
            </Button>
            <Button
                onClick={() => {
                    workerPoolManagerRef.current.terminateAllWorkers();
                }}
            >
                terminateAllWorkers
            </Button>
            {/*<div ref={logElementRef}/>*/}
            <ReactECharts option={getOption()}
                          style={{height: '400px', width: '800px', position: 'absolute', bottom: 0}}/>
        </div>
    );
}

const meta: Meta<typeof WorkerPoolManager> = {
    title: 'Dreamview/WorkerPoolManager',
    component: WorkerPoolManagerStory,
};

export const Primary: StoryObj<typeof WorkerPoolManager> = {};

export default meta;
