import React, { useEffect, useRef } from 'react';
import { Meta, StoryObj } from '@storybook/react';
import { WebSocketManager } from '@dreamview/dreamview-core/src/services/WebSocketManager';
import Logger from '@dreamview/log';
import { Switch, Button } from '@dreamview/dreamview-ui';
import useWebSocketServices from '../hooks/useWebSocketServices';
import { SubscriptionRegistry } from '../api/apiUtil';

const logger = Logger.getInstance('WebSocketManagerStory');

function WebSocketManagerStory() {
    const logElementRef = useRef(null);

    const { mainApi, streamApi, metadata, isMainConnected } = useWebSocketServices();

    const registry = new SubscriptionRegistry();

    useEffect(() => {
        if (logElementRef.current) {
            logger.setLogElement(logElementRef.current);
        }
    }, []);

    return (
        <div>
            <h1>WebSocket Manager Story</h1>
            <span>
                <Switch
                    checked={isMainConnected}
                    // onChange={(checked) => {
                    //     if (checked) {
                    //         webSocketManagerRef.current.connectMain();
                    //         setConnected(true);
                    //     } else {
                    //         webSocketManagerRef.current.disconnect();
                    //         setConnected(false);
                    //     }
                    // }}
                />
                &nbsp; Switch to &nbsp;
                {isMainConnected ? 'disconnect' : 'connect'}
            </span>

            {/* {isMainConnected ? <p>Connected to WebSocket</p> : <p>Disonnected</p>} */}

            <div>
                <div>依次调用，否则报错。</div>
                <Button
                    onClick={() => {
                        mainApi?.loadRecords().then(() => {
                            logger.info('WebSocketManagerStory received data from loadRecords');
                        });
                    }}
                >
                    loadRecords
                </Button>
                <Button
                    onClick={() => {
                        mainApi?.changeRecord('sensor_rgb').then(() => {
                            logger.info('WebSocketManagerStory received data from changeRecord');
                        });
                    }}
                >
                    changeRecord
                </Button>
                <Button
                    onClick={() => {
                        mainApi.startPlayRecorder().then(() => {
                            logger.info('WebSocketManagerStory received data from startRecordPlay');
                        });
                    }}
                >
                    startRecordPlay
                </Button>
                <Button
                    onClick={() => {
                        mainApi.updateMetaData().then(() => {
                            logger.info('WebSocketManagerStory received data from updateMetaData');
                        });
                    }}
                >
                    updateMetaData
                </Button>
            </div>
            <br />
            <div>
                <div>播包启停。</div>
                <Button
                    onClick={() => {
                        mainApi.ctrRecorderAction('pause', 0).then(() => {
                            logger.info('WebSocketManagerStory received data from Pause Records');
                        });
                    }}
                >
                    Pause Records
                </Button>
                <Button
                    onClick={() => {
                        mainApi.ctrRecorderAction('continue', 0).then(() => {
                            logger.info('WebSocketManagerStory received data from  Continue Record');
                        });
                    }}
                >
                    Continue Record
                </Button>
            </div>
            <br />
            <div>
                <div>修改mode</div>
                <Button
                    onClick={() => {
                        mainApi.changeSetupMode('Demo Perception').then(() => {
                            logger.info('WebSocketManagerStory received data from changeSetupMode');
                        });
                    }}
                >
                    Change SetupMode
                </Button>
                <Button
                    onClick={() => {
                        mainApi.changeOperation('SIM_DEBUG').then(() => {
                            logger.info('WebSocketManagerStory received data from  changeOperation');
                        });
                    }}
                >
                    Change Operation
                </Button>
            </div>
            <br />
            <div>
                <div>profile 切换</div>
                <Button
                    onClick={() => {
                        mainApi.changeMap('Sunnyvale Big Loop').then(() => {
                            logger.info('WebSocketManagerStory received data from  changeMap');
                        });
                    }}
                >
                    Change Map
                </Button>
            </div>
            <br />
            <div>
                <div>dump/reset</div>
                <Button
                    onClick={() => {
                        mainApi?.dumpFrame().then(() => {
                            logger.info('WebSocketManagerStory received data from dumpFrame');
                        });
                    }}
                >
                    dumpFrame
                </Button>
                <Button
                    onClick={() => {
                        mainApi?.resetSimWorld().then(() => {
                            logger.info('WebSocketManagerStory received data from resetSimWorld');
                        });
                    }}
                >
                    resetSimWorld
                </Button>
            </div>
            <br />
            <div>
                <div>订阅数据</div>
                {metadata.length > 0 &&
                    metadata.map((item) => {
                        if (!item.differentForChannels) {
                            return (
                                <Button
                                    onClick={() => {
                                        if (item.websocketInfo) {
                                            const subscription = streamApi
                                                ?.subscribeToData(item.dataName)
                                                .subscribe((data) => {
                                                    logger.info(
                                                        `WebSocketManagerStory received data from ${item.websocketInfo.websocketName}`,
                                                    );
                                                    logger.info(data as any);
                                                });
                                            registry.push(item.dataName, subscription);
                                        }
                                    }}
                                    key={item.websocketInfo.websocketName}
                                >
                                    {item.websocketInfo.websocketName}
                                </Button>
                            );
                        }
                        return (
                            <>
                                {item.channels.map((channel) => (
                                    <Button
                                        key={channel.channelName}
                                        type={channel.channelName.includes('hmi') ? 'primary' : 'default'}
                                        onClick={() => {
                                            const subscription = streamApi
                                                ?.subscribeToDataWithChannel(item.dataName, channel.channelName)
                                                .subscribe((data) => {
                                                    logger.info(
                                                        `WebSocketManagerStory received data from ${item.dataName} ${channel}`,
                                                    );
                                                    logger.info(data as any);
                                                });
                                            registry.push(item.dataName, subscription);
                                        }}
                                    >
                                        {item.dataName}
                                        {channel.channelName}
                                    </Button>
                                ))}
                            </>
                        );
                    })}
            </div>
            <br />
            <div>
                {metadata.length > 0 &&
                    metadata.map((item) => {
                        if (!item.differentForChannels) {
                            return (
                                <Button
                                    onClick={() => {
                                        if (item.websocketInfo) {
                                            registry.shift(item.dataName)?.unsubscribe();
                                        }
                                    }}
                                    key={item.websocketInfo.websocketName}
                                >
                                    disconnect
                                    {item.websocketInfo.websocketName}
                                </Button>
                            );
                        }
                        return (
                            <>
                                {item.channels.map((channel) => (
                                    <Button
                                        key={channel.channelName}
                                        type={channel.channelName.includes('hmi') ? 'primary' : 'default'}
                                        onClick={() => {
                                            registry.shift(item.dataName)?.unsubscribe();
                                        }}
                                    >
                                        disconnect
                                        {item.dataName}
                                        {channel.channelName}
                                    </Button>
                                ))}
                            </>
                        );
                    })}
            </div>

            <div ref={logElementRef} />
        </div>
    );
}

const meta: Meta<typeof WebSocketManager> = {
    title: 'Dreamview/WebSocketManager',
    component: WebSocketManagerStory,
};

export const Primary: StoryObj<typeof WebSocketManager> = {};

export default meta;
