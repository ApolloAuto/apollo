import React, {useEffect, useState} from 'react';
import {DreamviewAnalysis} from '../lib/dreamview-analysis';
import {Subscription} from "rxjs";
import {perfMonitor} from "../lib/performance-monitor";

const PerformanceMonitorPluginAdaptor: React.FC = () => {
    const [visible, setVisible] = useState(false);
    const dreamviewAnalysis = DreamviewAnalysis.getInstance();
    const [subscription, setSubscription] = useState<Subscription | null>(null);

    useEffect(() => {
        if (visible) {
            const sub = dreamviewAnalysis.getDataStream().subscribe({
                // 发送给chrome插件
                next: (data) => {
                    // console.log('visible:', visible);
                    // channel.postMessage({
                    //     type: 'updateData',
                    //     payload: data,
                    // });
                    // // @ts-ignore
                    // chrome.runtime.sendMessage("gabdacigffgnofljadcobbbibcbcigfi", {type: "updateData", payload: data}, function(response) {
                    //     console.log(response.reply);
                    // });

                    const event = new CustomEvent('dreamview-analysis', {detail: data});
                    document.dispatchEvent(event);
                },
                error: (err) => {
                    console.error(err);
                },
            });
            setSubscription(sub);
        } else {
            subscription?.unsubscribe();
            setSubscription(null);
        }

        return () => {
            subscription?.unsubscribe();
        };
    }, [visible]);

    // 采集内存相关信息
    const collectMemoryInfo = () => {
        return requestIdleCallback(() => {
            // @ts-ignore
            if (!performance || !performance.memory) return;
            // @ts-ignore
            const memory = performance.memory;
            dreamviewAnalysis.logData('Memory', {
                TotalJSHeapSize: memory.totalJSHeapSize / 1024 / 1024,
                UsedJSHeapSize: memory.usedJSHeapSize / 1024 / 1024,
                JSHeapSizeLimit: memory.jsHeapSizeLimit / 1024 / 1024,
            }, {
                useStatistics: {
                    useMax: true,
                    useMin: true,
                }
            });
            collectMemoryInfo();
        });
    }

    // 通过requestIdleCallback采集DOM信息
    const collectDOMInfo = () => {
        return requestIdleCallback(() => {
            dreamviewAnalysis.logData('DOM', {
                NodeCount: document.getElementsByTagName('*').length,
            }, {
                useStatistics: {
                    useMax: true,
                }
            });
            collectDOMInfo();
        });
    }

    // 通过requestIdleCallback采集默认数据
    const collectDefaultInfo = () => {
        return requestIdleCallback(() => {
            try {
                if (!performance && !performance.timing) return;
                dreamviewAnalysis.logData('default', {
                    // 计算页面加载总时间
                    LoadTime: performance.timing.loadEventEnd - performance.timing.navigationStart + 'ms',
                    // 计算页面解析DOM树的时间
                    ParseDOMTime: performance.timing.domComplete - performance.timing.responseEnd + 'ms',
                    // 计算页面重定向时间
                    RedirectTime: performance.timing.redirectEnd - performance.timing.redirectStart + 'ms',
                    // 计算页面DNS查询时间
                    DNSLookupTime: performance.timing.domainLookupEnd - performance.timing.domainLookupStart + 'ms',
                    // 计算页面TCP连接时间
                    TCPConnectTime: performance.timing.connectEnd - performance.timing.connectStart + 'ms',
                    // 计算页面请求时间
                    RequestTime: performance.timing.responseEnd - performance.timing.requestStart + 'ms',
                    // 计算页面响应时间
                    ResponseTime: performance.timing.responseEnd - performance.timing.responseStart + 'ms',
                    // 计算页面解析时间
                    ParseTime: performance.timing.domInteractive - performance.timing.responseEnd + 'ms',
                });
                collectDefaultInfo();
            } catch (e) {
                console.error(e);
            }

        });

    }

    useEffect(() => {
        let collectDOMInfoId: number;
        let collectDefaultInfoId: number;
        let collectMemoryInfoId: number;
        if (visible) {
            collectDOMInfoId = collectDOMInfo();
            collectDefaultInfoId = collectDefaultInfo();
            collectMemoryInfoId = collectMemoryInfo();
            perfMonitor.start();
        }

        return () => {
            if (!visible) {
                collectDOMInfoId && cancelIdleCallback(collectDOMInfoId);
                collectDefaultInfoId && cancelIdleCallback(collectDefaultInfoId);
                collectMemoryInfoId && cancelIdleCallback(collectMemoryInfoId);
                perfMonitor.stop();
            }
        }
    }, [visible]);

    useEffect(() => {
        const toggleVisibility = (event: KeyboardEvent) => {
            if (event.shiftKey && event.key.toLowerCase() === 'd') {
                setVisible(v => !v);
            }
        };

        window.addEventListener('keydown', toggleVisibility);

        perfMonitor.on('measure', (measure: PerformanceMeasure) => {
            dreamviewAnalysis.logData('Performance', {
                [measure.name]: measure.duration,
            }, {
                useStatistics: true,
            });
        });
        return () => window.removeEventListener('keydown', toggleVisibility);
    }, []);


    return null;
};

export default PerformanceMonitorPluginAdaptor;
