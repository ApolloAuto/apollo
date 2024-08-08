import React, { useEffect, useMemo, useRef, useState } from 'react';
import { Terminal } from 'xterm';
import { AdventureTime } from 'xterm-theme';
import { WebLinksAddon } from 'xterm-addon-web-links';
import { FitAddon } from 'xterm-addon-fit';
import { AttachAddon } from 'xterm-addon-attach';
import 'xterm/css/xterm.css';
import { useTranslation } from 'react-i18next';
import { usePanelContext } from '../base/store/PanelStore';
import CustomScroll from '../../CustomScroll';
import Panel from '../base/Panel';
import useStyle from './useStyle';
import { adjustTerminalSize, getPid } from '../../../util/terminalService';
import './index.less';
import useWebSocketServices from '../../../services/hooks/useWebSocketServices';
import { usePickHmiStore, ENUM_DATARECORD_PROCESS_STATUS } from '../../../store/HmiStore';
import { getWsProtocol, getHost } from '../../../services/WebSocketManager/constant';

function getUrl() {
    return `${getWsProtocol()}${getHost()}:8889/terminals`;
}

function debounce(fn, delay) {
    // 记录定时器返回的ID
    let timer = null;

    return function () {
        const context = this;
        const args = arguments;
        // 当有事件触发时清除上一个定时任务
        if (timer) {
            clearTimeout(timer);
        }
        // 重新发起一个定时任务
        timer = setTimeout(() => {
            fn.apply(context, args);
        }, delay);
    };
}

function InnerTerminal() {
    const { onPanelResize, logger, setKeyDownHandlers } = usePanelContext();
    const { classes } = useStyle();
    const terminalContainerRef = useRef<HTMLDivElement>(null);
    const fitAddonRef = useRef<FitAddon>(null);
    const connectionRef = useRef<WebSocket>(null);
    const termRef = useRef<Terminal>(null);
    const retryCountRef = useRef<number>(0);
    const timerRef = useRef<NodeJS.Timeout>(null);
    const { mainApi, isMainConnected } = useWebSocketServices();
    const [hmi] = usePickHmiStore();
    const { t } = useTranslation('panels');

    const getTermSize = () => ({
        rows: termRef.current?.rows,
        cols: termRef.current?.cols,
    });

    useEffect(() => {
        if (isMainConnected) {
            mainApi.startTerminal();
            timerRef.current = setInterval(() => {
                mainApi.startTerminal();
            }, 2000);
        }
    }, [mainApi, isMainConnected]);

    // eslint-disable-next-line react-hooks/exhaustive-deps
    useEffect(() => {
        const asyncFunc = async () => {
            const container = terminalContainerRef.current;

            termRef.current = new Terminal({
                rendererType: 'canvas',
                fontSize: 14,
                fontFamily: 'Consolas, "Courier New", monospace',
                bellStyle: 'sound',
                cursorBlink: true,
                theme: {
                    ...AdventureTime,
                    background: '#0F1014',
                },
                windowsMode: true, // 根据窗口换行
            });

            fitAddonRef.current = new FitAddon();

            termRef.current.loadAddon(new WebLinksAddon());
            termRef.current.loadAddon(fitAddonRef.current);
            fitAddonRef.current.fit();

            termRef.current.open(container);

            let processId: string;
            try {
                const res = await getPid({
                    rows: termRef.current.rows,
                    cols: termRef.current.cols,
                });
                processId = res.data;
            } catch (error) {
                logger.debug(JSON.stringify(error));
            }

            const url = getUrl();
            const SOCKETURL = `${url}/${processId}`;
            connectionRef.current = new WebSocket(SOCKETURL);

            const addEventListener = () => {
                connectionRef.current.onopen = () => {
                    const attachAddon = new AttachAddon(connectionRef.current);
                    termRef.current.loadAddon(attachAddon);
                    logger.debug('terminal connection opened');
                };

                connectionRef.current.onmessage = (event) => {
                    // console.log(event.data);
                };

                connectionRef.current.onclose = (event) => {
                    logger.debug('terminal connection closed');
                    if (event) {
                        logger.debug(JSON.stringify(event));
                    }

                    setTimeout(() => {
                        if (retryCountRef.current <= 30) {
                            logger.debug('链接失败，重试');
                            connectionRef.current = new WebSocket(SOCKETURL);
                            addEventListener();
                            termRef.current.clear();
                            retryCountRef.current += 1;
                        } else {
                            logger.debug('链接失败，重试次数已达上限');
                        }
                    }, 300);
                };

                connectionRef.current.onerror = (event) => {
                    logger.debug('与服务器连接丢失异常');

                    if (event) {
                        logger.debug(JSON.stringify(event));
                    }
                };
            };

            addEventListener();

            const resizeScreen = debounce(async () => {
                if (!processId) return;
                try {
                    const size = getTermSize();
                    await adjustTerminalSize(processId, size);
                    fitAddonRef.current.fit();
                } catch (error) {
                    logger.debug('resize request fail');
                    logger.debug(JSON.stringify(error));
                }
            }, 3000);

            termRef.current.onResize(() => {
                resizeScreen();
            });

            onPanelResize((width, height) => {
                resizeScreen();
            });
        };

        setTimeout(() => {
            asyncFunc();
        }, 500);
    }, []);

    useEffect(() => {
        if (hmi?.Terminal.processStatus.status === ENUM_DATARECORD_PROCESS_STATUS.OK && timerRef.current) {
            clearInterval(timerRef.current);
        }
    }, [hmi]);

    return (
        <CustomScroll style={{ height: '100%' }}>
            <div className='terminal-container' ref={terminalContainerRef} />
        </CustomScroll>
    );
}

InnerTerminal.displayName = 'InnerTerminal';

function TerminalWin(props: any) {
    const C = useMemo(
        () =>
            Panel({
                PanelComponent: InnerTerminal,
                panelId: props.panelId,
            }),
        [],
    );

    return <C {...props} />;
}

export default React.memo(TerminalWin);
