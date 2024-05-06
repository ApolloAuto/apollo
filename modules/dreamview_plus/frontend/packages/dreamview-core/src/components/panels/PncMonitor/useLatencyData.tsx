import React, { useCallback, useRef } from 'react';

const TIME_RANGE_IN_SEC = 300;

export const initData: any = () => ({});

export default function useLatencyData() {
    const data = useRef<any>(initData());

    function updateLatencyGraph(moduleName: string, newLatency: any) {
        if (!newLatency) {
            return;
        }

        const currentTime = newLatency.timestampSec;
        let graph = data.current[moduleName];
        if (graph.length > 0) {
            const startTimeSec = graph[0][0];
            const endTimeSec = graph[graph.length - 1][0];
            const diff = currentTime - startTimeSec;
            if (currentTime < endTimeSec) {
                // new data set, clean up existing one
                data.current[moduleName] = [];
                graph = data.current[moduleName];
            } else if (diff > TIME_RANGE_IN_SEC) {
                // shift out old data
                graph.shift();
            }
        }

        if (graph.length === 0 || graph[graph.length - 1][0] !== currentTime) {
            graph.push([currentTime, newLatency.totalTimeMs]);
        }
    }

    const onSimData = useCallback((simdata: any) => {
        if (simdata.latency) {
            Object.keys(simdata.latency).forEach((moduleName) => {
                if (!(moduleName in data.current)) {
                    data.current[moduleName] = [];
                }
                updateLatencyGraph(moduleName, simdata.latency[moduleName]);
            });
        }
        return { ...data.current };
    }, []);

    const refresh = useCallback(() => {
        data.current = {};
    }, []);

    return {
        data,
        onSimData,
        onRefresh: refresh,
    };
}
