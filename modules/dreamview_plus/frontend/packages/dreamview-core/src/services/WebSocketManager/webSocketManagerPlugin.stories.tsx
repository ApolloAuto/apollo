import React, { useEffect, useRef } from 'react';
import { Meta, StoryObj } from '@storybook/react';
import { WebSocketManager } from '@dreamview/dreamview-core/src/services/WebSocketManager';
import Logger from '@dreamview/log';
import { Switch, Button } from '@dreamview/dreamview-ui';
import useWebSocketServices from '../hooks/useWebSocketServices';

const logger = Logger.getInstance('WebSocketManagerStory');

function WebSocketManagerStory() {
    const logElementRef = useRef(null);

    const { isPluginConnected, pluginApi } = useWebSocketServices();

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
                    checked={isPluginConnected}
                    // onChange={(checked) => {
                    //     if (checked) {
                    //         webSocketManagerRef.current.connectPlugin();
                    //         setConnected(true);
                    //     } else {
                    //         webSocketManagerRef.current.disconnect();
                    //         setConnected(false);
                    //     }
                    // }}
                />
                &nbsp; Switch to &nbsp;
                {isPluginConnected ? 'disconnect' : 'connect'}
            </span>

            {/* {connected ? <p>Connected to WebSocket</p> : <p>Disonnected</p>} */}

            <div>
                <Button
                    onClick={() => {
                        pluginApi?.checkCertStatus().then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    CheckCertStatus
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.getRecordsList().then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    GetRecordsList
                </Button>
                <Button
                    onClick={() => {
                        const subscription = pluginApi
                            ?.downloadRecord('demo_3.5.record', 'demo_3.5.record')
                            .subscribe((res) => {
                                logger.info(res);
                                if (res.percentage === 100) {
                                    subscription.unsubscribe();
                                }
                            });
                    }}
                >
                    DownloadRecord
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.refreshDownloadRecord('demo_3.5.record', 'demo_3.5.record').subscribe((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    refreshDownloadRecord
                </Button>
                {/* <Button */}
                {/*    onClick={() => { */}
                {/*        pluginApi?.delete('demo_3.5.record', 'demo_3.5.record').subscribe((res) => { */}
                {/*            logger.info(res); */}
                {/*        }); */}
                {/*    }} */}
                {/* > */}
                {/*    deleteRecord */}
                {/* </Button> */}
                <Button
                    onClick={() => {
                        pluginApi?.getAccountInfo().then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    getAccountInfo
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.getVehicleInfo().then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    getVehicleInfo
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.resetVehicleConfig('12').then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    resetVehicleConfig
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.refreshVehicleConfig('12').then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    refreshVehicleConfig
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.uploadVehicleConfig('12').then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    uploadVehicleConfig
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.getV2xInfo().then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    getV2xInfo
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.refreshV2xConf('12').then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    refreshV2xConf
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.uploadV2xConf('12').then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    uploadV2xConf
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.getDynamicModelList().then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    getDynamicModelList
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.downloadDynamicModel('Mkz_Model').subscribe((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    downloadDynamicModel
                </Button>
                <Button
                    onClick={() => {
                        pluginApi?.getScenarioSetList().then((res) => {
                            logger.info(res);
                        });
                    }}
                >
                    getScenarioSetList
                </Button>
                <Button
                    onClick={() => {
                        pluginApi
                            ?.downloadScenarioSet('639323199c8b315fe0a3be36', '639323199c8b315fe0a3be36')
                            .subscribe((res) => {
                                logger.info(res);
                            });
                    }}
                >
                    downloadScenarioSet
                </Button>
            </div>
            <div ref={logElementRef} />
        </div>
    );
}

const meta: Meta<typeof WebSocketManager> = {
    title: 'Dreamview/PluginWebSocketManager',
    component: WebSocketManagerStory,
};

export const Primary: StoryObj<typeof WebSocketManager> = {};

export default meta;
