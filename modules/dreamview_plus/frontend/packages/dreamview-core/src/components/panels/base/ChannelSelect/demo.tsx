import React, { useEffect, useLayoutEffect, useMemo, useState } from 'react';
// import { DefaultOptionType } from 'antd';
import ChannelSelect from '.';

import useWebSocketServices from '../../../../services/hooks/useWebSocketServices';
import { RenderToolbarProps } from '../../type/RenderToolBar';
import useRegisterNotifyInitialChanel from '../../../../hooks/useRegisterNotifyInitialChanel';

export function DemoChannelSelect(props: RenderToolbarProps) {
    const { panelId, updateChannel } = props;
    const { metadata, isMainConnected } = useWebSocketServices();
    const [curVal, setCurVal] = useState(undefined);
    const curMeta = useMemo(() => metadata.find((meata) => meata.dataName === props.name), [metadata, isMainConnected]);
    const channels = useMemo(() => {
        if (!curMeta) {
            return [];
        }

        return curMeta.channels.map((channel) => ({
            label: channel,
            value: channel,
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
    };

    useEffect(() => {
        if (channels.length > 0) {
            setCurVal(channels[0]?.value);
            notifyInitialChannel({
                name: curMeta.dataName,
                channel: channels[0]?.value,
                needChannel: true,
            });
        } else {
            setCurVal(undefined);
        }
    }, [channels]);

    return <ChannelSelect value={curVal} options={channels} onChange={onChange} />;
}
