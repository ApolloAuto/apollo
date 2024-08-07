# 使用文档 - Web应用性能监控与调控器

## 1. 简介
Web应用性能监控与调控器是Dreamview的一个重要组件，用于监控Dreamview的性能指标，以及提供一些调控功能。

## 2. 使用方式
```typescript
import LogicController from './LogicController';
import PerformanceMonitor from './PerformanceMonitor';

const monitor = new PerformanceMonitor();
export const performanceController = new LogicController(monitor, {
    highLoadThreshold: 30,
    sampleInterval: 1000,
});

class DataProcessingModule {
    private isHighLoad = false;

    constructor(private performanceController: LogicController) {
        this.performanceController.monitor.fps$.subscribe(fps => {
            this.adjustPerformance(fps < 30);
        });
    }

    start() {
        console.log("数据处理模块启动");
        this.processData();
    }

    private processData() {
        if (!this.isHighLoad) {
            console.log("执行复杂数据处理");
            // 复杂数据处理逻辑
        } else {
            console.log("执行简化数据处理");
            // 简化数据处理逻辑
        }

        setTimeout(() => this.processData(), this.isHighLoad ? 2000 : 1000);
    }

    adjustPerformance(highLoad: boolean) {
        this.isHighLoad = highLoad;
        console.log(`数据处理模块已${highLoad ? '降低' : '增加'}负载`);
    }
}


const dataModule = new DataProcessingModule(performanceController);
dataModule.start();
```
