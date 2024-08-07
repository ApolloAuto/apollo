import { EventEmitter } from 'events';

class PerformanceMonitor extends EventEmitter {
    private running = false;

    constructor(private threshold: number = 50) {
        super();
    }

    // 封装mark和measure
    public mark(name: string): void {
        if (this.running) {
            performance.mark(name);
        }
    }

    public measure(name: string, startMark: string, endMark: string): void {
        if (this.running) {
            performance.measure(name, startMark, endMark);
        }
    }

    public start(): void {
        if (!this.running) {
            this.running = true;
            this.emit('start');
            this.scheduleMonitoring();
        }
    }

    public stop(): void {
        if (this.running) {
            this.running = false;
            this.emit('stop');
            performance.clearMarks();
            performance.clearMeasures();
        }
    }

    private scheduleMonitoring(): void {
        if (this.running) {
            requestIdleCallback(() => {
                this.processMeasures();
                this.scheduleMonitoring();
            });
        }
    }

    private processMeasures(): void {
        const measures = performance.getEntriesByType('measure') as PerformanceMeasure[];
        measures.forEach((measure) => {
            this.emit('measure', measure);
            if (measure.duration > this.threshold) {
                this.emit('exceed', measure);
            }
        });

        performance.clearMeasures();
    }
}

export const perfMonitor = new PerformanceMonitor(50);
// perfMonitor.on('start', () => console.log('Monitoring started'));
// perfMonitor.on('stop', () => console.log('Monitoring stopped'));
// perfMonitor.on('measure', (measure: PerformanceMeasure) => {
//     console.log(`Measured ${measure.name}: ${measure.duration.toFixed(2)}ms`);
// });
// perfMonitor.on('exceed', (measure: PerformanceMeasure) => {
//     console.log(`Measured ${measure.name} threshold reached: ${measure.duration.toFixed(2)}ms`);
// });
//
// perfMonitor.start();
// 假设在某个条件下停止监控
// perfMonitor.stop();
