import {BehaviorSubject} from 'rxjs'
import {MetricOptions} from "./types";
import Statistics from "./Statistics";

// debug
// window.setLogLevel('DreamviewAnalysis', 'debug')

export class DreamviewAnalysis {
    private static instance: DreamviewAnalysis | null = null;

    private dataStream = new BehaviorSubject<Record<string, any>>({});

    private stats = new Statistics();

    private constructor() {
    }

    public static getInstance(): DreamviewAnalysis {
        if (!DreamviewAnalysis.instance) {
            DreamviewAnalysis.instance = new DreamviewAnalysis();
        }
        return DreamviewAnalysis.instance;
    }

    logData(source: string, metric: Record<string, any> | any, options?: MetricOptions): void {
        let processedMetric: any;
        try {
            processedMetric = options?.transformFunc ? options.transformFunc(metric) : metric;
            // 处理可能为单一值或对象的情况
            // this.dataStream.value[source]不存在的情况下给一个空对象
            if (!this.dataStream.value[source]) {
                this.dataStream.value[source] = {};
            }
            if (typeof processedMetric === 'object' && !Array.isArray(processedMetric) && processedMetric !== null) {
                // 更新每个键
                Object.entries(processedMetric).forEach(([key, value]) => {
                    if (typeof value === 'number' && options?.useStatistics) {
                        this.stats.updateMetrics(`${source}_${key}`, value, options?.useStatistics);
                        this.dataStream.value[source][key] = this.stats.getMetrics(`${source}_${key}`);
                    } else {
                        this.dataStream.value[source][key] = value;
                    }
                });
            } else {
                this.dataStream.value[source] = processedMetric;
                if (typeof processedMetric === 'number' && options?.useStatistics) {
                    this.stats.updateMetrics(source, processedMetric, options?.useStatistics);
                    this.dataStream.value[source] = this.stats.getMetrics(source);
                }
            }
        } catch (error: any) {
            this.dataStream.value['error'] = error.message;
        }
        this.dataStream.next({...this.dataStream.value});
    }

    getDataStream(): BehaviorSubject<Record<string, any>> {
        return this.dataStream;
    }
}

export default DreamviewAnalysis.getInstance();
