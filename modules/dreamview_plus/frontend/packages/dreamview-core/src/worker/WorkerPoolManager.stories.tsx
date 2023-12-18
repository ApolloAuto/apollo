/* eslint-disable import/no-webpack-loader-syntax */
// @ts-ignore
import WorkerPoolManagerWorker from 'worker-loader!./WorkerPoolManager.worker';
import React, { useEffect, useRef } from 'react';
import { Meta, StoryObj } from '@storybook/react';
import WorkerPoolManager from '@dreamview/dreamview-core/src/worker/WorkerPoolManager';
import Logger from '@dreamview/log';
import { Button } from '@dreamview/dreamview-ui';
import { WorkerFactory } from './WorkerFactory';

const logger = Logger.getInstance('WorkerPoolManager');

function WorkerPoolManagerStory() {
    const logElementRef = useRef(null);
    const workerPoolManagerRef = useRef<WorkerPoolManager<number>>();

    useEffect(() => {
        // @ts-ignore
        window.setLogLevel('WorkerPoolManager', 'debug');
        if (logElementRef.current) {
            logger.setLogElement(logElementRef.current);
        }

        workerPoolManagerRef.current = new WorkerPoolManager<number>({
            name: 'WorkerPoolManagerStory',
            workerFactory: new WorkerFactory<number>(() => new WorkerPoolManagerWorker()),
        });

        const intervalId = setInterval(() => {
            workerPoolManagerRef.current?.visualize();
        }, 1000);

        return () => {
            workerPoolManagerRef.current?.terminateAllWorkers();
            clearInterval(intervalId);
        };
    }, []);

    return (
        <div>
            <h1>WorkerPool Manager Story</h1>
            <Button
                onClick={() => {
                    workerPoolManagerRef.current
                        .dispatchTask({
                            type: 'add',
                            payload: 1000,
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
                    const promise = Array.from({ length: 10 }).map((_, index) =>
                        workerPoolManagerRef.current.dispatchTask({
                            type: 'add',
                            payload: index * 100 * Math.random() * 10,
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
                    const promise = Array.from({ length: 100 }).map((_, index) =>
                        workerPoolManagerRef.current.dispatchTask({
                            type: 'add',
                            payload: index * 100 * Math.random() * 10,
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
            <div ref={logElementRef} />
        </div>
    );
}

const meta: Meta<typeof WorkerPoolManager> = {
    title: 'Dreamview/WorkerPoolManager',
    component: WorkerPoolManagerStory,
};

export const Primary: StoryObj<typeof WorkerPoolManager> = {};

export default meta;
