import { BehaviorSubject, interval, animationFrameScheduler } from 'rxjs';

export default class PerformanceMonitor {
    fpsSubject = new BehaviorSubject<number>(60);

    public fps$ = this.fpsSubject.asObservable();

    constructor() {
        this.monitorFrameRate();
    }

    private monitorFrameRate() {
        let lastFrameTime = performance.now();
        let frameCount = 0;
        let accumTime = 0; // 累计时间

        interval(0, animationFrameScheduler).subscribe(() => {
            const now = performance.now();
            const deltaTime = now - lastFrameTime;
            accumTime += deltaTime;
            frameCount += 1;

            // 每过一秒钟计算一次FPS
            if (accumTime >= 1000) {
                const fps = frameCount / (accumTime / 1000);
                this.fpsSubject.next(fps);
                frameCount = 0;
                accumTime = 0;
            }

            lastFrameTime = now;
        });
    }
}
