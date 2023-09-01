/* eslint-disable react/prop-types */
import React from 'react';
import RenderToolbar from '../RenderToolbar';

export type ChannelSelectFactoryProps = {
    name?: string;
    CustomToolBar?: (props: any) => React.JSX.Element;
    helpContent?: React.ReactNode;
};

export default function ChannelSelectFactory(props: ChannelSelectFactoryProps) {
    const CustomToolBar = props.CustomToolBar;
    function ChannelSelectFactoryWrapper(wrapperProps) {
        return (
            <RenderToolbar name={props.name} helpContent={props.helpContent} {...wrapperProps}>
                {CustomToolBar && (
                    <CustomToolBar name={props.name} updateChannel={wrapperProps.updateChannel} {...wrapperProps} />
                )}
            </RenderToolbar>
        );
    }

    return ChannelSelectFactoryWrapper;
}
