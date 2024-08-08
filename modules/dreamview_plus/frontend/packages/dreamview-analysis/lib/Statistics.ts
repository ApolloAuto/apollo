import {useStatisticsType} from "./types";

export default class Statistics {
    private data: {
        [key: string]: { value: number, max: number, min: number, sum: number, count: number, average: number }
    } = {};

    updateMetrics(source: string, value: number, useStatistics: useStatisticsType): void {
        const stats = this.data[source] || { value: 0, max: -Infinity, min: Infinity, sum: 0, count: 1, average: 0 };
        stats.value = value;
        if (typeof useStatistics === 'boolean' && useStatistics) {
            stats.max = Math.max(stats.max, value);
            stats.min = Math.min(stats.min, value);
            stats.sum += value;
            stats.count++;
            stats.average = stats.sum / stats.count;
        } else if (typeof useStatistics === 'object') {
            if (useStatistics.useMax) {
                stats.max = Math.max(stats.max, value);
            }
            if (useStatistics.useMin) {
                stats.min = Math.min(stats.min, value);
            }
            if (useStatistics.useSum) {
                stats.sum += value;
            }
            if (useStatistics.useCount) {
                stats.count++;
            }
            if (useStatistics.useAverage) {
                stats.sum += value;
                stats.count++;
                stats.average = stats.sum / stats.count;
            }
        }

        this.data[source] = stats;
    }

    getMetrics(source: string): any {
        return this.data[source];
    }
}
