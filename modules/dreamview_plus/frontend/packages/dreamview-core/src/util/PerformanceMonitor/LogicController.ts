import { throttleTime, map } from 'rxjs/operators';
// import Logger from '@dreamview/log';
import PerformanceMonitor from './PerformanceMonitor';

interface LogicControlOptions {
    // 高负载阈值
    highLoadThreshold: number;
    // 采样间隔
    sampleInterval: number;
}

// const logger = Logger.getInstance(__filename);

export default class LogicController {
    public logicController$;

    constructor(public monitor: PerformanceMonitor, private options: LogicControlOptions) {
        // this.monitor.fps$
        //     .pipe(
        //         throttleTime(this.options.sampleInterval),
        //         map((fps) => fps < this.options.highLoadThreshold),
        //     )
        //     .subscribe((isHighLoad) => {
        //         // 通知所有关联模块调整性能
        //         logger.debug(
        //             `当前处于${isHighLoad ? '高负载' : '正常'}状态，当前帧率为${this.monitor.fpsSubject.value}`,
        //         );
        //     });
        this.logicController$ = this.monitor.fps$.pipe(
            throttleTime(this.options.sampleInterval),
            map((fps) => fps < this.options.highLoadThreshold),
        );
    }
}
