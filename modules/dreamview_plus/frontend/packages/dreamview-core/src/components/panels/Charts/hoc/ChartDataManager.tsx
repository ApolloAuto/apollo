import React, { useRef, useCallback, useMemo, useEffect } from 'react';
import { Subscription } from 'rxjs';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import difference from 'lodash/difference';
import intersection from 'lodash/intersection';

type noopFunc = () => void;

interface IChartDataManagerContext {
    consumerChannelData: (dataName: string, channelName: string, handler: (data: any) => void) => noopFunc;
    channelChangeHandler: (channels: string[]) => void;
}

const ChartDataManagerContext = React.createContext<IChartDataManagerContext>(null);

export function ChartDataManagerProvider(props: React.PropsWithChildren) {
    // key is like `${dataName}!${channelName}`
    const { subscribeToData } = usePanelContext();
    const subscribeCollection = useRef<Record<string, Subscription>>({});
    const consumerCollection = useRef<Record<string, Array<(data: any) => void>>>({});

    const addSubscribeCollection = useCallback((dataName: string, channel: string) => {
        if (!dataName || !channel) {
            return;
        }
        const key = `${dataName}!${channel}`;
        if (subscribeCollection.current[key]) {
            return;
        }
        subscribeCollection.current[key] = subscribeToData({ name: dataName, channel, needChannel: true }).subscribe(
            (data: any) => {
                (consumerCollection.current[key] || []).forEach((handler) => {
                    handler(data);
                });
            },
        );
    }, []);

    const removeSubscribeCollection = useCallback((dataName: string, channel: string) => {
        const key = `${dataName}!${channel}`;
        subscribeCollection.current[key].unsubscribe();
        delete subscribeCollection.current[key];
    }, []);

    const removeAllSubscribeCollection = useCallback(() => {
        Object.values(subscribeCollection.current).forEach((subscribtion) => subscribtion.unsubscribe());
    }, []);

    const addConsumerCollection = useCallback((dataName: string, channel: string, handler: (data: any) => void) => {
        const key = `${dataName}!${channel}`;
        if (!consumerCollection.current[key]) {
            consumerCollection.current[key] = [];
        }
        consumerCollection.current[key].push(handler);
    }, []);

    const removeConsumerCollection = useCallback((dataName: string, channel: string, handler: (data: any) => void) => {
        const key = `${dataName}!${channel}`;
        consumerCollection.current[key] = consumerCollection.current[key].filter((item) => item !== handler);
    }, []);

    const consumerChannelData: IChartDataManagerContext['consumerChannelData'] = useCallback(
        (dataName, channelName, handler) => {
            addConsumerCollection(dataName, channelName, handler);
            return () => {
                removeConsumerCollection(dataName, channelName, handler);
            };
        },
        [],
    );

    /**
     * channels string[] 订阅channel的集合
     */
    const channelChangeHandler: IChartDataManagerContext['channelChangeHandler'] = useCallback((channels) => {
        const prevChannelKeys = Object.keys(subscribeCollection.current);
        const nextChannelsKeys = channels;
        {
            const newChannels = difference(nextChannelsKeys, prevChannelKeys);
            newChannels.forEach((key) => {
                const [dataName, channel] = key.split('!');
                addSubscribeCollection(dataName, channel);
            });
        }
        {
            const uselessChannels = difference(prevChannelKeys, nextChannelsKeys);
            uselessChannels.forEach((key) => {
                const [dataName, channel] = key.split('!');
                removeSubscribeCollection(dataName, channel);
            });
        }
        {
            const unchangedChannels = intersection(nextChannelsKeys, nextChannelsKeys);
            // do nothing
        }
    }, []);

    useEffect(
        () => () => {
            removeAllSubscribeCollection();
        },
        [],
    );

    const contextValue = useMemo(
        () => ({
            consumerChannelData,
            channelChangeHandler,
        }),
        [consumerChannelData, channelChangeHandler],
    );

    return <ChartDataManagerContext.Provider value={contextValue}>{props.children}</ChartDataManagerContext.Provider>;
}

export function useChartDataManager() {
    return React.useContext(ChartDataManagerContext);
}
