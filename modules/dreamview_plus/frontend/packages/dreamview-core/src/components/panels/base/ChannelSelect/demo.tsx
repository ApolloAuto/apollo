/* eslint-disable no-restricted-syntax */
import React, { useEffect, useLayoutEffect, useMemo, useState } from 'react';
// import { DefaultOptionType } from 'antd';
import ChannelSelect from '.';

import useWebSocketServices from '../../../../services/hooks/useWebSocketServices';
import { RenderToolbarProps } from '../../type/RenderToolBar';
import useRegisterNotifyInitialChanel from '../../../../hooks/useRegisterNotifyInitialChanel';
import { useLocalStorage } from '../../../../util/storageManager';

function DemoChannelSelect(props: RenderToolbarProps) {
    const { panelId, updateChannel } = props;
    const { metadata, isMainConnected } = useWebSocketServices();
    const localStorageManager = useLocalStorage(`${panelId}-selected-cn`);
    const [curVal, setCurVal] = useState(undefined);
    const curMeta = useMemo(() => metadata.find((meta) => meta.dataName === props.name), [metadata, isMainConnected]);
    const channels = useMemo(() => {
        if (!curMeta) {
            return [];
        }

        return curMeta.channels.map((channel) => ({
            label: channel.channelName,
            value: channel.channelName,
        }));
    }, [curMeta]);
    const notifyInitialChannel = useRegisterNotifyInitialChanel(panelId);

    const onChange = (value, option) => {
        setCurVal(value);
        updateChannel({
            name: props.name,
            channel: value,
            needChannel: true,
        });
        localStorageManager.set(value);
    };

    useEffect(() => {
        if (channels.length > 0) {
            const cacheChannel = localStorageManager.get();
            let cn = channels[0]?.value;
            for (const channel of channels) {
                if (cacheChannel === channel.value) {
                    cn = cacheChannel;
                    break;
                }
            }
            setCurVal(cn);
            notifyInitialChannel({
                name: curMeta.dataName,
                channel: cn,
                needChannel: true,
            });
        } else {
            setCurVal(undefined);
        }
    }, [channels]);

    return <ChannelSelect value={curVal} options={channels} onChange={onChange} />;
}

export default React.memo(DemoChannelSelect);
