import LogicController from './LogicController';
import PerformanceMonitor from './PerformanceMonitor';

const monitor = new PerformanceMonitor();
export const performanceController = new LogicController(monitor, {
    highLoadThreshold: 30,
    sampleInterval: 1000,
});
