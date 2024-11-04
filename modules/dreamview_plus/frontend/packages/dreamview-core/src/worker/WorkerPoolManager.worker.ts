/* eslint-disable no-restricted-globals */
import { TaskInternal } from './type';

self.onmessage = function (e: MessageEvent<TaskInternal<number>>) {
    const { type, id, payload } = e.data;

    // console.log(`Received task: ${id}`);

    if (type === 'add') {
        // 模拟耗时任务
        const processingTime = payload || 1000; // 默认耗时1秒
        setTimeout(() => {
            self.postMessage({
                id,
                success: true,
                result: `Task ${id} completed after ${processingTime} ms`,
            });
        }, processingTime);
    }
};
