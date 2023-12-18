import { MosaicPath } from 'react-mosaic-component';
import { IPanelMetaInfo } from './Panel';

export type SubscribeInfo = {
    name?: string;
    channel?: string;
    needChannel?: boolean;
};

export interface RenderToolbarProps {
    panel?: IPanelMetaInfo;
    path?: MosaicPath;
    panelId?: string;
    inFullScreen?: boolean;
    helpContent?: React.ReactNode;
    updateChannel?: (newChannel: SubscribeInfo) => void;
    name?: string;
    customToolBar?: React.JSX.Element;
}
