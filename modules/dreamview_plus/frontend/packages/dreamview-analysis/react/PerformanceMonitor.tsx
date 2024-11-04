import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom';
import Draggable from 'react-draggable';
import ReactJson from 'react-json-view';
import { Subscription } from 'rxjs';
import { Resizable } from 're-resizable';
import { DreamviewAnalysis } from '../lib/dreamview-analysis';
import { perfMonitor } from '../lib/performance-monitor'; // 导入样式

const PerformanceMonitor: React.FC = () => {
    const [data, setData] = useState({});
    const [visible, setVisible] = useState(false);
    const dreamviewAnalysis = DreamviewAnalysis.getInstance();
    const [subscription, setSubscription] = useState<Subscription | null>(null);

    useEffect(() => {
        if (visible) {
            const sub = dreamviewAnalysis.getDataStream().subscribe(setData);
            setSubscription(sub);
        } else {
            setData({});
            subscription?.unsubscribe();
            setSubscription(null);
        }

        return () => {
            subscription?.unsubscribe();
        };
    }, [visible]);

    // 采集内存相关信息
    const collectMemoryInfo = () =>
        requestIdleCallback(() => {
            // @ts-ignore
            if (!performance || !performance.memory) return;
            // @ts-ignore
            const memory = performance.memory;
            dreamviewAnalysis.logData(
                'Memory',
                {
                    TotalJSHeapSize: memory.totalJSHeapSize / 1024 / 1024,
                    UsedJSHeapSize: memory.usedJSHeapSize / 1024 / 1024,
                    JSHeapSizeLimit: memory.jsHeapSizeLimit / 1024 / 1024,
                },
                {
                    useStatistics: {
                        useMax: true,
                        useMin: true,
                    },
                },
            );
            collectMemoryInfo();
        });

    // 通过requestIdleCallback采集DOM信息
    const collectDOMInfo = () =>
        requestIdleCallback(() => {
            dreamviewAnalysis.logData(
                'DOM',
                {
                    NodeCount: document.getElementsByTagName('*').length,
                },
                {
                    useStatistics: {
                        useMax: true,
                    },
                },
            );
            collectDOMInfo();
        });

    // 通过requestIdleCallback采集默认数据
    const collectDefaultInfo = () =>
        requestIdleCallback(() => {
            try {
                if (!performance && !performance.timing) return;
                dreamviewAnalysis.logData('default', {
                    // 计算页面加载总时间
                    LoadTime: `${performance.timing.loadEventEnd - performance.timing.navigationStart}ms`,
                    // 计算页面解析DOM树的时间
                    ParseDOMTime: `${performance.timing.domComplete - performance.timing.responseEnd}ms`,
                    // 计算页面重定向时间
                    RedirectTime: `${performance.timing.redirectEnd - performance.timing.redirectStart}ms`,
                    // 计算页面DNS查询时间
                    DNSLookupTime: `${performance.timing.domainLookupEnd - performance.timing.domainLookupStart}ms`,
                    // 计算页面TCP连接时间
                    TCPConnectTime: `${performance.timing.connectEnd - performance.timing.connectStart}ms`,
                    // 计算页面请求时间
                    RequestTime: `${performance.timing.responseEnd - performance.timing.requestStart}ms`,
                    // 计算页面响应时间
                    ResponseTime: `${performance.timing.responseEnd - performance.timing.responseStart}ms`,
                    // 计算页面解析时间
                    ParseTime: `${performance.timing.domInteractive - performance.timing.responseEnd}ms`,
                });
                collectDefaultInfo();
            } catch (e) {
                console.error(e);
            }
        });

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
        };
    }, [visible]);

    useEffect(() => {
        const toggleVisibility = (event: KeyboardEvent) => {
            if (event.shiftKey && event.key.toLowerCase() === 'd') {
                setVisible((v) => !v);
            }
        };

        window.addEventListener('keydown', toggleVisibility);

        perfMonitor.on('measure', (measure: PerformanceMeasure) => {
            dreamviewAnalysis.logData(
                'Performance',
                {
                    [measure.name]: measure.duration,
                },
                {
                    useStatistics: true,
                },
            );
        });
        return () => window.removeEventListener('keydown', toggleVisibility);
    }, []);

    if (!visible) return null;

    return ReactDOM.createPortal(
        <Draggable handle='.handle'>
            <Resizable
                defaultSize={{
                    width: 300,
                    height: 200,
                }}
                minWidth={300}
                minHeight={200}
                maxWidth={1800}
                maxHeight={1800}
                style={{
                    position: 'fixed',
                    bottom: 4,
                    right: 4,
                    backgroundColor: 'transparent',
                    padding: '10px',
                    zIndex: 1000,
                }}
            >
                <div
                    style={{
                        position: 'relative',
                        width: '100%',
                        height: '100%',
                        backgroundColor: 'rgba(15, 15, 20, .6)',
                        padding: '10px',
                        borderRadius: '6px',
                        boxShadow: '0 2px 6px rgba(26, 29, 36, .5)',
                        zIndex: 1000,
                        backdropFilter: 'blur(10px)',
                    }}
                >
                    <div
                        style={{
                            cursor: 'move',
                            display: 'flex',
                            justifyContent: 'space-between',
                            padding: '10px',
                            position: 'absolute',
                            width: '100%',
                            top: 0,
                            left: 0,
                            zIndex: 1000,
                            backgroundColor: 'rgba(26, 29, 36, 1)',
                            borderRadius: '6px',
                        }}
                        className='handle'
                    >
                        <strong style={{ color: 'rgba(128, 139, 157, 1.000)' }}>Dreamview Analysis</strong>
                    </div>
                    <div style={{ height: 30 }} />
                    <div
                        style={{
                            overflow: 'auto',
                            width: '100%',
                            height: 'calc(100% - 40px)',
                        }}
                    >
                        <ReactJson
                            src={data}
                            theme='monokai'
                            collapsed={2}
                            enableClipboard
                            style={{ backgroundColor: 'transparent', padding: '10px' }}
                        />
                    </div>
                </div>
            </Resizable>
        </Draggable>,
        document.body,
    );
};

export default PerformanceMonitor;
